import board
import rp2pio
import adafruit_pioasm
import digitalio
import time

from adafruit_ticks import ticks_ms, ticks_diff

# *** Wiznet Common Registers ***
_REG_MR = 0x0000  # Mode Register
_REG_GAR = 0x0001  # Gateway Address Register
_REG_SUBR = 0x0005  # Subnet Mask Register
_REG_VERSIONR = 0x0039  # Version Register
_REG_SHAR = 0x0009  # Source Hardware Address Register
_REG_SIPR = 0x000F  # Source IP Address Register
_REG_LINK_FLAG = 0x002E  # PHY Configuration Register (for link status)
_REG_RCR = 0x001B  # Retry Count Register
_REG_RTR = 0x0019  # Retry Time Register

# *** Wiznet Socket Registers ***
_REG_SNMR = 0x0000  # Socket Mode Register
_REG_SNCR = 0x0001  # Socket Command Register
_REG_SNIR = 0x0002  # Socket Interrupt Register
_REG_SNSR = 0x0003  # Socket Status Register
_REG_SNPORT = 0x0004  # Socket Source Port Register
_REG_SNDIPR = 0x000C  # Socket Destination IP Address Register
_REG_SNDPORT = 0x0010  # Socket Destination Port Register
_REG_SNRX_RSR = 0x0026  # Socket RX Received Size Register
_REG_SNRX_RD = 0x0028  # Socket RX Read Pointer Register
_REG_SNTX_FSR = 0x0020  # Socket TX Free Size Register
_REG_SNTX_WR = 0x0024  # Socket TX Write Pointer Register

# Socket Status
SNSR_SOCK_CLOSED = 0x00
_SNSR_SOCK_INIT = 0x13
SNSR_SOCK_LISTEN = 0x14
_SNSR_SOCK_SYNSENT = 0x15
SNSR_SOCK_SYNRECV = 0x16
SNSR_SOCK_ESTABLISHED = 0x17
SNSR_SOCK_FIN_WAIT = 0x18
_SNSR_SOCK_CLOSING = 0x1A
SNSR_SOCK_TIME_WAIT = 0x1B
SNSR_SOCK_CLOSE_WAIT = 0x1C
_SNSR_SOCK_LAST_ACK = 0x1D
_SNSR_SOCK_UDP = 0x22
_SNSR_SOCK_IPRAW = 0x32
_SNSR_SOCK_MACRAW = 0x42
_SNSR_SOCK_PPPOE = 0x5F

# Socket Commands (CMD)
_CMD_SOCK_OPEN = 0x01
_CMD_SOCK_LISTEN = 0x02
_CMD_SOCK_CONNECT = 0x04
_CMD_SOCK_DISCON = 0x08
_CMD_SOCK_CLOSE = 0x10
_CMD_SOCK_SEND = 0x20
_CMD_SOCK_SEND_MAC = 0x21
_CMD_SOCK_SEND_KEEP = 0x22
_CMD_SOCK_RECV = 0x40

# Socket Interrupt Register Flags
_SNIR_SEND_OK = 0x10
SNIR_TIMEOUT = 0x08
_SNIR_RECV = 0x04
SNIR_DISCON = 0x02
_SNIR_CON = 0x01

_CH_SIZE = 0x100
_SOCK_SIZE = 0x800  # MAX W5k socket size
_SOCK_MASK = 0x7FF
# Register commands
_MR_RST = 0x80  # Mode Register RST
# Socket mode register
_SNMR_CLOSE = 0x00
_SNMR_TCP = 0x21
SNMR_UDP = 0x02
_SNMR_IPRAW = 0x03
_SNMR_MACRAW = 0x04
_SNMR_PPPOE = 0x05

_MAX_PACKET = 4000
_LOCAL_PORT = 0x400
# Default hardware MAC address
_DEFAULT_MAC = "DE:AD:BE:EF:FE:ED"

# Maximum number of sockets to support, differs between chip versions.
_MAX_SOCK_NUM = 0x08  # For w5500
_SOCKET_INVALID = 0xFF

# PIO 어셈블리 코드: SPI 마스터 구현
spi_master = """
.program spi_master
    pull block
    set x, 7
bitloop:
    out pins, 1
    set pins, 1
    nop [1]
    in pins, 1
    set pins, 0
    jmp x-- bitloop
    push block
"""

mac_address = [0x00, 0x08, 0xDC, 0x01, 0x02, 0x03]
ip_address = [192, 168, 11, 100]
gateway_ip = [192, 168, 11, 1]
subnet_mask = [255, 255, 255, 0]


def debug_msg(message: str, debug: bool):
    if debug:
        print(message)


class WIZNET5K:
    def __init__(
        self,
        spi_sm: rp2pio.StateMachine,
        cs: digitalio.DigitalInOut,
        reset=None,
        is_dhcp: bool = True,
        mac=_DEFAULT_MAC,
        hostname=None,
        debug: bool = False,
    ) -> None:
        """
        :param rp2pio.StateMachine spi_sm: WIZnet 모듈이 연결된 PIO SPI StateMachine.
        :param digitalio.DigitalInOut cs: 칩 선택 핀.
        :param digitalio.DigitalInOut reset: 선택적 리셋 핀, 기본값은 None.
        :param bool is_dhcp: DHCP를 자동으로 시작할지 여부, 기본값은 True.
        :param Union[MacAddressRaw, str] mac: WIZnet의 MAC 주소, 기본값은 (0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED).
        :param str hostname: 원하는 호스트 이름, MAC 주소를 채우기 위한 {} 포함 가능, 기본값은 None.
        :param bool debug: 디버깅 출력 활성화, 기본값은 False.
        """
        self._debug = debug
        self._chip_type = None
        self._sm = spi_sm

        # CS 핀 초기화
        self._cs = cs
        self._cs.direction = digitalio.Direction.OUTPUT
        self._cs.value = True

        # Reset WIZnet 모듈
        if reset:
            debug_msg("* Resetting WIZnet chip", self._debug)
            reset.switch_to_output()
            reset.value = False
            time.sleep(0.1)
            reset.value = True
            time.sleep(0.1)
        self._rst = reset

        # SPI 통신을 위한 버퍼 초기화
        self._pbuff = bytearray(8)
        self._rxbuf = bytearray(2048)  # 최대 패킷 크기 (예시로 2048 바이트)

        # MAC 주소 설정
        if isinstance(mac, str):
            # 문자열 형태의 MAC 주소를 파싱
            self._mac = tuple(int(x, 16) for x in mac.split(":"))
        else:
            self._mac = mac

        # 호스트 이름 설정 (필요 시 구현)
        self._hostname = hostname

        # DHCP 사용 여부 설정
        self._is_dhcp = is_dhcp

        # 추가적인 초기화 작업 수행
        self._ch_base_msb = 0
        self._src_ports_in_use = []
        self.max_sockets = 8

        # UDP 관련 초기화
        self.udp_from_ip = [b"\x00\x00\x00\x00"] * self.max_sockets
        self.udp_from_port = [0] * self.max_sockets

        # WIZnet 칩 초기화
        self.init()

        # Ethernet 링크가 초기화
        start_time = time.monotonic()
        timeout = 5  # 5초 타임아웃
        while time.monotonic() - start_time < timeout:
            if self.link_status():
                break
            debug_msg("Ethernet 링크가 다운되었습니다...", self._debug)
            time.sleep(0.5)
        self._dhcp_client = None

        # DHCP 설정
        if is_dhcp:
            self.set_dhcp(hostname)

    def link_status(self):
        # 링크 상태를 확인하는 메서드 구현 필요
        # 예시로 PHYCFGR 레지스터를 읽어서 링크 상태를 반환
        phycfgr = self.read_reg(_REG_LINK_FLAG, 0)
        return bool(phycfgr & 0x01)  # LNK 비트 확인

    def set_dhcp(self, hostname=None):
        # DHCP 클라이언트를 구현하거나 외부 라이브러리를 사용하여 DHCP 기능 추가
        debug_msg("DHCP 설정 중...", self._debug)
        # 실제 DHCP 구현은 복잡하므로 여기에 추가해야 합니다.
        pass

    def _select(self):
        self._cs.value = False

    def _deselect(self):
        self._cs.value = True

    def _transfer(self, data_out):
        data_in = bytearray(len(data_out))
        self._sm.write_readinto(data_out, data_in)
        return data_in

    def _read(self, addr, block, length=1):
        self._select()
        command = bytearray([(addr >> 8) & 0xFF, addr & 0xFF, block & 0xFF])
        data_out = command + bytearray([0x00] * length)
        data_in = self._transfer(data_out)
        self._deselect()
        return data_in[3:]

    def _write(self, addr: int, callback: int, data: Union[int, bytes]) -> None:
        self._select()
        # 명령어 구성
        command = bytearray(
            [
                (addr >> 8) & 0xFF,
                addr & 0xFF,
                (callback | 0x04) & 0xFF,
            ]
        )

        # 데이터 타입 처리
        if isinstance(data, int):
            try:
                data = data.to_bytes(1, "big")
            except OverflowError:
                data = data.to_bytes(2, "big")
        elif isinstance(data, bytes) or isinstance(data, bytearray):
            pass  # 이미 바이트 배열인 경우 그대로 사용
        else:
            raise TypeError("Data must be an integer or bytes-like object")

        # 전체 데이터 전송
        data_out = command + data
        data_in = self._transfer(data_out)
        self._deselect()

    def _write_socket_register(self, sock: int, address: int, data: int) -> None:
        """W5500 소켓 레지스터에 쓰기."""
        cntl_byte = (sock << 5) + 0x0C
        self._write(address, cntl_byte, data)

    def _read_socket_register(self, sock: int, address: int) -> int:
        """W5500 소켓 레지스터에서 읽기."""
        cntl_byte = (sock << 5) + 0x08
        register = self._read(address, cntl_byte)
        return int.from_bytes(register, "big")

    def read_reg(self, addr, block=0):
        data = self._read(addr, block, 1)
        return data[0]

    def write_reg(self, addr, block, data):
        self._write(addr, block, data)
        time.sleep(0.01)

    def reset(self):
        self._rst.value = False
        time.sleep(0.1)
        self._rst.value = True
        time.sleep(0.1)

    def init(self):
        self.reset()
        # Soft reset
        self.write_reg(_REG_MR, 0, _MR_RST)
        time.sleep(0.1)
        # Initialize Common Register
        self.write_reg(_REG_MR, 0, 0x00)
        # Set MAC Address
        for i, val in enumerate(mac_address):
            self.write_reg(_REG_SHAR + i, 0, val)
        # Set IP Address
        for i, val in enumerate(ip_address):
            self.write_reg(_REG_SIPR + i, 0, val)
        # Set Gateway IP
        for i, val in enumerate(gateway_ip):
            self.write_reg(_REG_GAR + i, 0, val)
        # Set Subnet Mask
        for i, val in enumerate(subnet_mask):
            self.write_reg(_REG_SUBR + i, 0, val)
        print("W5500 Initialization complete")

    def socket_set_buffer_size(self, txsize, rxsize):
        tx_total = sum(txsize)
        rx_total = sum(rxsize)

        if any(size not in [0, 1, 2, 4, 8, 16] for size in txsize + rxsize):
            raise ValueError("Buffer size must be one of [0, 1, 2, 4, 8, 16] KB")

        if tx_total > 16 or rx_total > 16:
            raise ValueError("Total TX or RX buffer size cannot exceed 16KB")

        for i in range(len(txsize)):
            self.write_reg(0x001E + i, 0, txsize[i])
            self.write_reg(0x001F + i, 0, rxsize[i])
            print(f"Socket {i} buffer sizes set: TX={txsize[i]}KB, RX={rxsize[i]}KB")

    def socket_init(self, sock_num):
        print("Initializing socket")
        # 소켓을 닫고 초기화
        self._write_socket_register(
            sock_num, _REG_SNCR, _CMD_SOCK_CLOSE
        )  # CLOSE command
        time.sleep(0.01)

        status = self._read_socket_register(sock_num, _REG_SNSR)
        print(f"Socket {sock_num} status after close: 0x{status:02X}")

        # 버퍼 크기 설정
        self._write_socket_register(sock_num, 0x001E, 2)  # TX buffer size to 2KB
        self._write_socket_register(sock_num, 0x001F, 2)  # RX buffer size to 2KB
        time.sleep(0.01)

        # TCP 모드 설정
        self._write_socket_register(sock_num, _REG_SNMR, _SNMR_TCP)  # Set to TCP mode
        time.sleep(0.01)

        # 소켓 열기
        self._write_socket_register(sock_num, _REG_SNCR, _CMD_SOCK_OPEN)  # OPEN command
        time.sleep(0.1)

        # 소켓 상태 확인
        status = self._read_socket_register(sock_num, _REG_SNSR)
        print(f"Socket {sock_num} status after open: 0x{status:02X}")
        if status != _SNSR_SOCK_INIT:  # SOCK_INIT 상태가 아니면
            raise RuntimeError(
                f"Socket {sock_num} failed to initialize, status: 0x{status:02X}"
            )

    def socket_listen(self, sock_num, port):
        print(f"Setting socket {sock_num} to listen on port {port}")
        self._write_socket_register(
            sock_num, _REG_SNPORT, port >> 8
        )  # Set port (high byte)
        self._write_socket_register(
            sock_num, _REG_SNPORT + 1, port & 0xFF
        )  # Set port (low byte)
        time.sleep(0.01)

        self._write_socket_register(
            sock_num, _REG_SNCR, _CMD_SOCK_LISTEN
        )  # LISTEN command
        time.sleep(0.1)

        while self._read_socket_register(sock_num, _REG_SNCR):
            time.sleep(0.01)  # Wait for command to complete

        status = self._read_socket_register(sock_num, _REG_SNSR)
        print(f"Socket {sock_num} status after listen: 0x{status:02X}")

    def socket_status(self, sock_num):
        return self._read_socket_register(sock_num, _REG_SNSR)

    def socket_recv(self, sock_num):
        rx_size_high = self._read_socket_register(sock_num, _REG_SNRX_RSR)
        rx_size_low = self._read_socket_register(sock_num, _REG_SNRX_RSR + 1)
        rx_size = (rx_size_high << 8) + rx_size_low

        if rx_size == 0:
            return None

        # RX 읽기 포인터 읽기
        rx_rd_high = self._read_socket_register(sock_num, _REG_SNRX_RD)
        rx_rd_low = self._read_socket_register(sock_num, _REG_SNRX_RD + 1)
        rx_rd = (rx_rd_high << 8) + rx_rd_low

        # 읽을 물리 주소 계산
        rx_buffer_base = 0x6000 + sock_num * 0x1000
        addr = rx_buffer_base + (rx_rd & 0x0FFF)

        # RX 버퍼에서 데이터 읽기
        data = self._read(addr, 0x18, rx_size)

        # RX 읽기 포인터 업데이트
        rx_rd += rx_size
        self._write_socket_register(sock_num, _REG_SNRX_RD, (rx_rd >> 8) & 0xFF)
        self._write_socket_register(sock_num, _REG_SNRX_RD + 1, rx_rd & 0xFF)

        # RECV 명령 실행
        self._write_socket_register(sock_num, _REG_SNCR, _CMD_SOCK_RECV)

        return data

    def socket_send(self, sock_num, data):
        data_length = len(data)

        # TX 쓰기 포인터 읽기
        tx_wr_high = self._read_socket_register(sock_num, _REG_SNTX_WR)
        tx_wr_low = self._read_socket_register(sock_num, _REG_SNTX_WR + 1)
        tx_wr = (tx_wr_high << 8) + tx_wr_low

        # 쓸 물리 주소 계산
        tx_buffer_base = 0x4000 + sock_num * 0x1000
        addr = tx_buffer_base + (tx_wr & 0x0FFF)

        # TX 버퍼에 데이터 쓰기
        self._write(addr, 0x14, data)

        # TX 쓰기 포인터 업데이트
        tx_wr += data_length
        self._write_socket_register(sock_num, _REG_SNTX_WR, (tx_wr >> 8) & 0xFF)
        self._write_socket_register(sock_num, _REG_SNTX_WR + 1, tx_wr & 0xFF)

        # SEND 명령 실행
        self._write_socket_register(sock_num, _REG_SNCR, _CMD_SOCK_SEND)

        # SEND 명령 완료 대기
        while self._read_socket_register(sock_num, _REG_SNCR):
            time.sleep(0.001)

    def socket_close(self, sock_num):
        self._write_socket_register(
            sock_num, _REG_SNCR, _CMD_SOCK_CLOSE
        )  # CLOSE command
        while self._read_socket_register(sock_num, _REG_SNCR):
            time.sleep(0.01)  # Wait for command to complete

    def print_socket_status(self, sock_num):
        status = self.socket_status(sock_num)
        print(f"Socket {sock_num} status: 0x{status:02X}")
        print(
            f"Socket {sock_num} mode: 0x{self._read_socket_register(sock_num, _REG_SNMR):02X}"
        )
        print(
            f"Socket {sock_num} command: 0x{self._read_socket_register(sock_num, _REG_SNCR):02X}"
        )
        print(
            f"Socket {sock_num} interrupt: 0x{self._read_socket_register(sock_num, _REG_SNIR):02X}"
        )
        port_high = self._read_socket_register(sock_num, _REG_SNPORT)
        port_low = self._read_socket_register(sock_num, _REG_SNPORT + 1)
        port = (port_high << 8) | port_low
        print(f"Socket {sock_num} port: {port}")

    def print_network_info(self):
        print("Network Information:")
        print(
            "IP Address:",
            ".".join(str(self.read_reg(_REG_SIPR + i, 0)) for i in range(4)),
        )
        print(
            "Gateway IP:",
            ".".join(str(self.read_reg(_REG_GAR + i, 0)) for i in range(4)),
        )
        print(
            "Subnet Mask:",
            ".".join(str(self.read_reg(_REG_SUBR + i, 0)) for i in range(4)),
        )
        print(
            "MAC Address:",
            ":".join(f"{self.read_reg(_REG_SHAR + i, 0):02X}" for i in range(6)),
        )

    def read_version(self):
        version = self.read_reg(_REG_VERSIONR, 0)
        return version

    @property
    def rcr(self) -> int:
        """Retry count register."""
        return self.read_reg(_REG_RCR)

    @rcr.setter
    def rcr(self, retry_count: int) -> None:
        """Retry count register."""
        if 0 > retry_count > 255:
            raise ValueError("Retries must be from 0 to 255.")
        self.write_reg(_REG_RCR, 0x04, retry_count)

    @property
    def rtr(self) -> int:
        """Retry time register."""
        high = self.read_reg(_REG_RTR)
        low = self.read_reg(_REG_RTR + 1)
        return (high << 8) + low

    @rtr.setter
    def rtr(self, retry_time: int) -> None:
        """Retry time register."""
        if not (0 <= retry_time < 2**16):
            raise ValueError("Retry time must be from 0 to 65535")

        high = (retry_time >> 8) & 0xFF
        low = retry_time & 0xFF
        self.write_reg(_REG_RTR, 0x00, high)
        self.write_reg(_REG_RTR + 1, 0x00, low)


# PIO 및 State Machine 설정
assembled = adafruit_pioasm.assemble(spi_master)
sm = rp2pio.StateMachine(
    assembled,
    frequency=1_000_000,
    first_out_pin=board.GP19,
    first_in_pin=board.GP16,
    first_set_pin=board.GP18,
    out_pin_count=1,
    in_pin_count=1,
    set_pin_count=1,
    in_shift_right=False,
    out_shift_right=False,
    push_threshold=8,
    pull_threshold=8,
)

# CS 및 RST 핀 설정
cs_pin = digitalio.DigitalInOut(board.GP17)
rst_pin = digitalio.DigitalInOut(board.GP20)

# WIZNET5K 초기화
wiznet = WIZNET5K(sm, cs_pin, rst_pin)
wiznet.init()
wiznet.print_network_info()

# WIZNET5K 인스턴스 생성 후
version = wiznet.read_version()
print(f"W5500 버전: 0x{version:02X}")

wiznet.rtr = 2000  # 원하는 재시도 시간 값으로 설정
current_rtr = wiznet.rtr
print(f"RTR : {current_rtr}")

wiznet.rcr = 8
current_rcr = wiznet.rcr
print(f"RCR : {current_rcr}")

# 소켓 버퍼 크기 설정
tx_buffer_sizes = [2, 2, 2, 2, 2, 2, 2, 2]  # 각 소켓의 TX 버퍼 크기 설정 (KB 단위)
rx_buffer_sizes = [2, 2, 2, 2, 2, 2, 2, 2]  # 각 소켓의 RX 버퍼 크기 설정 (KB 단위)
wiznet.socket_set_buffer_size(tx_buffer_sizes, rx_buffer_sizes)

# 소켓 초기화 및 리스닝
sock_num = 0
port = 5000

print("Socket Open")
wiznet.socket_init(sock_num)
wiznet.socket_listen(sock_num, port)

print(f"Listening on port {port}")

while True:
    status = wiznet.socket_status(sock_num)
    if status == SNSR_SOCK_ESTABLISHED:  # SOCK_ESTABLISHED
        # Receive data from the client
        data = wiznet.socket_recv(sock_num)
        if data:
            print(f"Received: {data}")
            # Echo the data back to the client
            wiznet.socket_send(sock_num, data)
    elif status == SNSR_SOCK_CLOSE_WAIT:  # SOCK_CLOSE_WAIT
        print("Client disconnected, closing socket")
        wiznet.socket_close(sock_num)
        wiznet.socket_init(sock_num)
        wiznet.socket_listen(sock_num, port)
    elif status == SNSR_SOCK_LISTEN:  # SOCK_LISTEN
        # print("Listening for incoming connections...")
        pass
    elif status == SNSR_SOCK_CLOSED:  # SOCK_CLOSED
        wiznet.socket_init(sock_num)
        wiznet.socket_listen(sock_num, port)
    else:
        print(f"Socket status: {status:02X}")
    # time.sleep(0.1)
