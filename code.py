import board
import rp2pio
import adafruit_pioasm
import digitalio
import time
from adafruit_ticks import ticks_ms, ticks_diff

_DEFAULT_MAC = "DE:AD:BE:EF:FE:ED"

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
        reset: Optional[digitalio.DigitalInOut] = None,
        is_dhcp: bool = True,
        mac: Union[MacAddressRaw, str] = _DEFAULT_MAC,
        hostname: Optional[str] = None,
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
            time.sleep(5)
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
        self.max_sockets = 8  # W5500은 최대 8개의 소켓을 지원

        # UDP 관련 초기화
        self.udp_from_ip = [b"\x00\x00\x00\x00"] * self.max_sockets
        self.udp_from_port = [0] * self.max_sockets

        # WIZnet 칩 초기화
        self.init()

        # Ethernet 링크가 초기화될 시간을 기다립니다.
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
        phycfgr = self.read_reg(0x002E, 0)
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
        command = bytearray([(addr >> 8) & 0xFF, addr & 0xFF, block])
        data_out = command + bytearray([0x00] * length)
        data_in = self._transfer(data_out)
        self._deselect()
        return data_in[3:]

    def _write(self, addr, block, data):
        self._select()
        command = bytearray([(addr >> 8) & 0xFF, addr & 0xFF, block | 0x04])
        if isinstance(data, int):
            data = bytearray([data])
        else:
            data = bytearray(data)
        self._transfer(command + data)
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
        self.write_reg(0x0000, 0, 0x80)
        time.sleep(0.1)
        # Initialize Common Register
        self.write_reg(0x0000, 0, 0x00)
        # Set MAC Address
        
        for i, val in enumerate(mac_address):
            self.write_reg(0x0009 + i, 0, val)
        # Set IP Address
        for i, val in enumerate(ip_address):
            self.write_reg(0x000F + i, 0, val)
        # Set Gateway IP
        for i, val in enumerate(gateway_ip):
            self.write_reg(0x0001 + i, 0, val)
        # Set Subnet Mask
        for i, val in enumerate(subnet_mask):
            self.write_reg(0x0005 + i, 0, val)
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
        self._write_socket_register(sock_num, 0x0401, 0x10)  # CLOSE command
        time.sleep(0.01)

        status = self._read_socket_register(sock_num, 0x0403)
        print(f"Socket {sock_num} status after close: 0x{status:02X}")

        # 버퍼 크기 설정
        self._write_socket_register(sock_num, 0x001E, 2)  # TX buffer size to 2KB
        self._write_socket_register(sock_num, 0x001F, 2)  # RX buffer size to 2KB
        time.sleep(0.01)

        # TCP 모드 설정
        self._write_socket_register(sock_num, 0x0400, 0x01)  # Set to TCP mode
        time.sleep(0.01)

        # 소켓 열기
        self._write_socket_register(sock_num, 0x0401, 0x01)  # OPEN command
        time.sleep(0.1)

        # 소켓 상태 확인
        status = self._read_socket_register(sock_num, 0x0403)
        print(f"Socket {sock_num} status after open: 0x{status:02X}")
        if status != 0x13:  # SOCK_INIT 상태가 아니면
            raise RuntimeError(
                f"Socket {sock_num} failed to initialize, status: 0x{status:02X}"
            )

    def socket_listen(self, sock_num, port):
        print(f"Setting socket {sock_num} to listen on port {port}")
        self._write_socket_register(sock_num, 0x0404, port >> 8)  # Set port (high byte)
        self._write_socket_register(
            sock_num, 0x0405, port & 0xFF
        )  # Set port (low byte)
        time.sleep(0.01)

        self._write_socket_register(sock_num, 0x0401, 0x02)  # LISTEN command
        time.sleep(0.1)

        while self._read_socket_register(sock_num, 0x0401):
            time.sleep(0.01)  # Wait for command to complete

        status = self._read_socket_register(sock_num, 0x0403)
        print(f"Socket {sock_num} status after listen: 0x{status:02X}")

    def socket_status(self, sock_num):
        return self._read_socket_register(sock_num, 0x0403)

    def socket_recv(self, sock_num):
        rx_size = self._read_socket_register(
            sock_num, 0x0426
        ) << 8 | self._read_socket_register(sock_num, 0x0427)
        if rx_size > 0:
            data = self._read(0x0000 + (sock_num * 0x100), 2, rx_size)
            self._write_socket_register(
                sock_num, 0x0428, rx_size >> 8
            )  # Update RX read pointer
            self._write_socket_register(sock_num, 0x0429, rx_size & 0xFF)
            self._write_socket_register(sock_num, 0x0401, 0x40)  # Receive
            return data
        return None

    def socket_send(self, sock_num, data):
        self._write(0x0000 + (sock_num * 0x100), 2, data)
        self._write_socket_register(
            sock_num, 0x0420, len(data) >> 8
        )  # Update TX write pointer
        self._write_socket_register(sock_num, 0x0421, len(data) & 0xFF)
        self._write_socket_register(sock_num, 0x0401, 0x20)  # Send

    def socket_close(self, sock_num):
        self._write_socket_register(sock_num, 0x0401, 0x10)  # CLOSE command
        while self._read_socket_register(sock_num, 0x0401):
            time.sleep(0.01)  # Wait for command to complete

    def print_socket_status(self, sock_num):
        status = self.socket_status(sock_num)
        print(f"Socket {sock_num} status: 0x{status:02X}")
        print(
            f"Socket {sock_num} mode: 0x{self._read_socket_register(sock_num, 0x0400):02X}"
        )
        print(
            f"Socket {sock_num} command: 0x{self._read_socket_register(sock_num, 0x0401):02X}"
        )
        print(
            f"Socket {sock_num} interrupt: 0x{self._read_socket_register(sock_num, 0x0402):02X}"
        )
        print(
            f"Socket {sock_num} port: {self._read_socket_register(sock_num, 0x0404) << 8 | self._read_socket_register(sock_num, 0x0405)}"
        )

    def print_network_info(self):
        print("Network Information:")
        print(
            "IP Address:", ".".join(str(self.read_reg(0x000F + i, 0)) for i in range(4))
        )
        print(
            "Gateway IP:", ".".join(str(self.read_reg(0x0001 + i, 0)) for i in range(4))
        )
        print(
            "Subnet Mask:",
            ".".join(str(self.read_reg(0x0005 + i, 0)) for i in range(4)),
        )
        print(
            "MAC Address:",
            ":".join(f"{self.read_reg(0x0009 + i, 0):02X}" for i in range(6)),
        )
    def socket_recv(self, sock_num):
        # 수신된 데이터 크기 읽기
        rx_size_high = self._read_socket_register(sock_num, 0x0026)
        rx_size_low = self._read_socket_register(sock_num, 0x0027)
        rx_size = (rx_size_high << 8) + rx_size_low

        if rx_size == 0:
            return None

        # RX 읽기 포인터 읽기
        rx_rd_high = self._read_socket_register(sock_num, 0x0028)
        rx_rd_low = self._read_socket_register(sock_num, 0x0029)
        rx_rd = (rx_rd_high << 8) + rx_rd_low

        # 읽을 물리 주소 계산
        rx_buffer_base = 0x6000 + sock_num * 0x1000
        addr = rx_buffer_base + (rx_rd & 0x0FFF)

        # RX 버퍼에서 데이터 읽기
        data = self._read(addr, 0x18, rx_size)

        # RX 읽기 포인터 업데이트
        rx_rd += rx_size
        self._write_socket_register(sock_num, 0x0028, (rx_rd >> 8) & 0xFF)
        self._write_socket_register(sock_num, 0x0029, rx_rd & 0xFF)

        # RECV 명령 실행
        self._write_socket_register(sock_num, 0x0001, 0x40)

        return data

    def socket_send(self, sock_num, data):
        data_length = len(data)

        # TX 쓰기 포인터 읽기
        tx_wr_high = self._read_socket_register(sock_num, 0x0024)
        tx_wr_low = self._read_socket_register(sock_num, 0x0025)
        tx_wr = (tx_wr_high << 8) + tx_wr_low

        # 쓸 물리 주소 계산
        tx_buffer_base = 0x4000 + sock_num * 0x1000
        addr = tx_buffer_base + (tx_wr & 0x0FFF)

        # TX 버퍼에 데이터 쓰기
        self._write(addr, 0x14, data)

        # TX 쓰기 포인터 업데이트
        tx_wr += data_length
        self._write_socket_register(sock_num, 0x0024, (tx_wr >> 8) & 0xFF)
        self._write_socket_register(sock_num, 0x0025, tx_wr & 0xFF)

        # SEND 명령 실행
        self._write_socket_register(sock_num, 0x0001, 0x20)

        # SEND 명령 완료 대기
        while self._read_socket_register(sock_num, 0x0001):
            time.sleep(0.001)


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
    if status == 0x17:  # SOCK_ESTABLISHED
        print("Connection established")
        # Receive data from the client
        data = wiznet.socket_recv(sock_num)
        if data:
            print(f"Received: {data}")
            # Echo the data back to the client
            wiznet.socket_send(sock_num, data)
            print("Data sent back to client")
    elif status == 0x1C:  # SOCK_CLOSE_WAIT
        print("Client disconnected, closing socket")
        wiznet.socket_close(sock_num)
        wiznet.socket_init(sock_num)
        wiznet.socket_listen(sock_num, port)
    elif status == 0x14:  # SOCK_LISTEN
        print("Listening for incoming connections...")
    elif status == 0x00:  # SOCK_CLOSED
        print("Socket closed, re-initializing...")
        wiznet.socket_init(sock_num)
        wiznet.socket_listen(sock_num, port)
    else:
        print(f"Socket status: {status:02X}")
    time.sleep(0.5)