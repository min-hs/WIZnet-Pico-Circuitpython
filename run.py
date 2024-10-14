import board
import rp2pio
import adafruit_pioasm
import digitalio
import time
import struct
import random

from adafruit_ticks import ticks_ms, ticks_diff
from micropython import const

# *** Wiznet Common Registers ***
_REG_MR = const(0x0000)
_REG_GAR = const(0x0001)
_REG_SUBR = const(0x0005)
_REG_SHAR = const(0x0009)
_REG_SIPR = const(0x000F)
_REG_VERSIONR = const(0x0039)
_REG_LINK_FLAG = const(0x002E)
_REG_RTR = const(0x0019)
_REG_RCR = const(0x001B)
_REG_PHYCFGR = const(0x002E)

# *** Wiznet Socket Registers ***
_REG_SNMR = const(0x0000)
_REG_SNCR = const(0x0001)
_REG_SNIR = const(0x0002)
_REG_SNSR = const(0x0003)
_REG_SNPORT = const(0x0004)
_REG_SNTX_FSR = const(0x0020)
_REG_SNTX_WR = const(0x0024)
_REG_SNRX_RSR = const(0x0026)
_REG_SNRX_RD = const(0x0028)
_REG_SNDIPR = const(0x000C)
_REG_SNDPORT = const(0x0010)

# Socket Commands (Sn_CR values)
_CMD_SOCK_OPEN = const(0x01)
_CMD_SOCK_LISTEN = const(0x02)
_CMD_SOCK_CONNECT = const(0x04)
_CMD_SOCK_DISCON = const(0x08)
_CMD_SOCK_CLOSE = const(0x10)
_CMD_SOCK_SEND = const(0x20)
_CMD_SOCK_RECV = const(0x40)

# Socket Modes (Sn_MR values)
_SNMR_CLOSE = const(0x00)
_SNMR_TCP = const(0x01)
_SNMR_UDP = const(0x02)

# Socket Status Register values (Sn_SR)
SNSR_SOCK_CLOSED = const(0x00)
_SNSR_SOCK_INIT = const(0x13)
SNSR_SOCK_LISTEN = const(0x14)
SNSR_SOCK_ESTABLISHED = const(0x17)
SNSR_SOCK_CLOSE_WAIT = const(0x1C)
_SNSR_SOCK_UDP = const(0x22)

# Other constants
_MR_RST = const(0x80)
_DEFAULT_MAC = "DE:AD:BE:EF:FE:ED"
_MAX_SOCK_NUM = const(0x08)
_SOCKET_INVALID = const(0xFF)
_SOCK_SIZE = const(0x800)
_SOCK_MASK = const(0x7FF)
_CH_SIZE = const(0x100)

# Buffer size registers
_REG_Sn_TXBUF_SIZE = const(0x001E)
_REG_Sn_RXBUF_SIZE = const(0x0022)

# PIO assembly code: SPI master implementation
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
        self._debug = debug
        self._chip_type = None
        self._sm = spi_sm

        # CS pin initialization
        self._cs = cs
        self._cs.direction = digitalio.Direction.OUTPUT
        self._cs.value = True

        # Reset WIZnet module
        if reset:
            debug_msg("* Resetting WIZnet chip", self._debug)
            reset.switch_to_output()
            reset.value = False
            time.sleep(0.1)
            reset.value = True
            time.sleep(0.1)
        self._rst = reset

        # SPI communication buffer initialization
        self._pbuff = bytearray(8)
        self._rxbuf = bytearray(2048)  # Maximum packet size (example: 2048 bytes)

        # Set MAC address
        if isinstance(mac, str):
            # Parse MAC address string
            self._mac = tuple(int(x, 16) for x in mac.split(":"))
        else:
            self._mac = mac

        # Set hostname (if needed)
        self._hostname = hostname

        # Set DHCP usage
        self._is_dhcp = is_dhcp

        # Additional initialization
        self._ch_base_msb = 0
        self._src_ports_in_use = []
        self.max_sockets = 8

        # UDP initialization
        self.udp_from_ip = [b"\x00\x00\x00\x00"] * self.max_sockets
        self.udp_from_port = [0] * self.max_sockets

        # Initialize WIZnet chip
        self.init()

        # Ethernet link initialization
        start_time = time.monotonic()
        timeout = 5  # 5 seconds timeout
        while time.monotonic() - start_time < timeout:
            if self.link_status():
                break
            debug_msg("Ethernet link is down...", self._debug)
            time.sleep(0.5)
        self._dhcp_client = None

        # DHCP setup
        if is_dhcp:
            self.set_dhcp(hostname)

    def dns_query(self, domain_name, dns_server_ip, sock_num=2, timeout=5):
        print(f"Performing DNS query for: {domain_name}")
        transaction_id = random.randint(0, 65535)

        # DNS Request Header
        header = struct.pack(
            ">HHHHHH",
            transaction_id,
            0x0100,
            1,
            0,
            0,
            0,  # 기본 DNS 요청 헤더 (표준 질의, 재귀적 요청)
        )

        # DNS Question Section
        question = b""
        for label in domain_name.split("."):
            question += struct.pack("B", len(label)) + label.encode("utf-8")
        question += struct.pack("B", 0)  # 도메인 이름 종료
        question += struct.pack(">HH", 1, 1)  # Type A (IPv4 주소), Class IN

        # DNS 요청 메시지 생성
        dns_request = header + question

        # UDP 소켓을 열어서 DNS 서버로 전송
        self.socket_open_udp(sock_num, 0)  # 로컬 포트 0을 사용하여 UDP 소켓 열기
        self.udp_sendto(
            sock_num, dns_server_ip, 53, dns_request
        )  # 포트 53 (DNS 서버 포트)로 전송

        # 응답 대기
        start_time = time.monotonic()
        while time.monotonic() - start_time < timeout:
            response = self.udp_recvfrom(sock_num)
            if response:
                # 응답 파싱
                recv_transaction_id = struct.unpack(">H", response[:2])[0]
                if recv_transaction_id == transaction_id:
                    # DNS 응답 메시지 파싱
                    answers_count = struct.unpack(">H", response[6:8])[0]
                    if answers_count > 0:
                        # Answer Section 파싱
                        answer_start = len(header) + len(question)
                        _, _, _, _, data_len = struct.unpack(
                            ">HHHLH", response[answer_start : answer_start + 12]
                        )
                        ip_start = answer_start + 12
                        ip_address = struct.unpack(
                            "BBBB", response[ip_start : ip_start + data_len]
                        )
                        print(
                            f"Domain {domain_name} resolved to: {'.'.join(map(str, ip_address))}"
                        )
                        return ip_address

        print("DNS query failed or timed out.")
        return None

    def link_status(self):
        # Check link status using PHYCFGR register
        phycfgr = self.read_reg(_REG_PHYCFGR, 0)
        return bool(phycfgr & 0x01)  # Check LNK bit

    def set_dhcp(self, hostname=None):
        # Implement DHCP client or use external library
        debug_msg("Setting up DHCP...", self._debug)
        # Actual DHCP implementation needs to be added
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

    def _write(self, addr: int, callback: int, data) -> None:
        self._select()
        # Construct command
        command = bytearray(
            [
                (addr >> 8) & 0xFF,  # Address high byte
                addr & 0xFF,  # Address low byte
                (callback | 0x04) & 0xFF,  # Control byte (set write bit)
            ]
        )

        # Data type handling
        if isinstance(data, int):
            try:
                data = data.to_bytes(1, "big")
            except OverflowError:
                data = data.to_bytes(2, "big")
        elif isinstance(data, bytes) or isinstance(data, bytearray):
            pass  # Already bytes-like
        else:
            raise TypeError("Data must be an integer or bytes-like object")

        # Transfer data
        data_out = command + data
        self._transfer(data_out)
        self._deselect()

    def _write_socket_register(self, sock: int, address: int, data: int) -> None:
        """Write to W5500 socket register."""
        cntl_byte = (sock << 5) + 0x0C  # Set write bit
        self._write(address, cntl_byte, data)

    def _read_socket_register(self, sock: int, address: int) -> int:
        """Read from W5500 socket register."""
        cntl_byte = (sock << 5) + 0x08  # Read operation
        register = self._read(address, cntl_byte)
        return int.from_bytes(register, "big")

    def read_reg(self, addr, block=0):
        data = self._read(addr, block, 1)
        return data[0]

    def write_reg(self, addr, block, data):
        self._write(addr, block, data)
        time.sleep(0.01)

    def reset(self):
        if self._rst is not None:
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
            self.write_reg(_REG_Sn_TXBUF_SIZE + i, 0, txsize[i])
            self.write_reg(_REG_Sn_RXBUF_SIZE + i, 0, rxsize[i])
            print(f"Socket {i} buffer sizes set: TX={txsize[i]}KB, RX={rxsize[i]}KB")

    def socket_init(self, sock_num):
        print("Initializing socket")
        # Close and initialize the socket
        self._write_socket_register(
            sock_num, _REG_SNCR, _CMD_SOCK_CLOSE
        )  # CLOSE command
        time.sleep(0.01)

        status = self._read_socket_register(sock_num, _REG_SNSR)
        print(f"Socket {sock_num} status after close: 0x{status:02X}")

        # Set to TCP mode
        self._write_socket_register(sock_num, _REG_SNMR, _SNMR_TCP)  # Set to TCP mode
        time.sleep(0.01)

        # Open the socket
        self._write_socket_register(sock_num, _REG_SNCR, _CMD_SOCK_OPEN)  # OPEN command
        time.sleep(0.1)

        # Check socket status
        status = self._read_socket_register(sock_num, _REG_SNSR)
        print(f"Socket {sock_num} status after open: 0x{status:02X}")
        if status != _SNSR_SOCK_INIT:
            raise RuntimeError(
                f"Socket {sock_num} failed to initialize, status: 0x{status:02X}"
            )

    def socket_listen(self, sock_num, port):
        print(f"Setting socket {sock_num} to listen on port {port}")
        # Set port (Sn_PORT is 0x0004, two bytes)
        self._write_socket_register(sock_num, _REG_SNPORT, (port >> 8) & 0xFF)
        self._write_socket_register(sock_num, _REG_SNPORT + 1, port & 0xFF)
        time.sleep(0.01)

        self._write_socket_register(
            sock_num, _REG_SNCR, _CMD_SOCK_LISTEN
        )  # LISTEN command
        time.sleep(0.1)

        # Wait for command to complete
        while self._read_socket_register(sock_num, _REG_SNCR):
            time.sleep(0.01)

        status = self._read_socket_register(sock_num, _REG_SNSR)
        print(f"Socket {sock_num} status after listen: 0x{status:02X}")

    def socket_status(self, sock_num):
        return self._read_socket_register(sock_num, _REG_SNSR)

    def _read_data(self, sock_num, addr, length):
        # Read data from RX buffer
        cntl_byte = (sock_num << 5) + 0x18  # 0x18 for RX buffer read
        data = self._read(addr, cntl_byte, length)
        return data

    def _write_data(self, sock_num, addr, data):
        # Write data to TX buffer
        cntl_byte = (sock_num << 5) + 0x14  # 0x14 for TX buffer write
        self._write(addr, cntl_byte, data)

    def socket_recv(self, sock_num):
        rx_size = self._read_socket_register(
            sock_num, _REG_SNRX_RSR
        ) << 8 | self._read_socket_register(sock_num, _REG_SNRX_RSR + 1)
        if rx_size > 0:
            # Read RX read pointer
            rx_rd = self._read_socket_register(
                sock_num, _REG_SNRX_RD
            ) << 8 | self._read_socket_register(sock_num, _REG_SNRX_RD + 1)

            # Calculate the physical address
            addr = rx_rd & _SOCK_MASK

            # Read data from RX buffer
            data = self._read_data(sock_num, addr, rx_size)

            # Update RX read pointer
            rx_rd = (rx_rd + rx_size) & 0xFFFF
            self._write_socket_register(sock_num, _REG_SNRX_RD, (rx_rd >> 8) & 0xFF)
            self._write_socket_register(sock_num, _REG_SNRX_RD + 1, rx_rd & 0xFF)

            # Issue RECV command
            self._write_socket_register(sock_num, _REG_SNCR, _CMD_SOCK_RECV)
            return data
        return None

    def socket_send(self, sock_num, data):
        data_length = len(data)

        # Read TX write pointer
        tx_wr = self._read_socket_register(
            sock_num, _REG_SNTX_WR
        ) << 8 | self._read_socket_register(sock_num, _REG_SNTX_WR + 1)

        # Calculate the physical address
        addr = tx_wr & _SOCK_MASK

        # Write data to TX buffer
        self._write_data(sock_num, addr, data)

        # Update TX write pointer
        tx_wr = (tx_wr + data_length) & 0xFFFF
        self._write_socket_register(sock_num, _REG_SNTX_WR, (tx_wr >> 8) & 0xFF)
        self._write_socket_register(sock_num, _REG_SNTX_WR + 1, tx_wr & 0xFF)

        # Issue SEND command
        self._write_socket_register(sock_num, _REG_SNCR, _CMD_SOCK_SEND)

        # Wait for SEND command to complete
        while self._read_socket_register(sock_num, _REG_SNCR):
            time.sleep(0.001)

    def socket_open_udp(self, sock_num, port):
        print(f"Opening UDP socket {sock_num} on port {port}")
        # Close the socket first
        self._write_socket_register(sock_num, _REG_SNCR, _CMD_SOCK_CLOSE)
        time.sleep(0.01)

        # Set to UDP mode
        self._write_socket_register(sock_num, _REG_SNMR, _SNMR_UDP)
        time.sleep(0.01)

        # Set the local port for the UDP socket
        self._write_socket_register(sock_num, _REG_SNPORT, (port >> 8) & 0xFF)
        self._write_socket_register(sock_num, _REG_SNPORT + 1, port & 0xFF)
        time.sleep(0.01)

        # Open the socket
        self._write_socket_register(sock_num, _REG_SNCR, _CMD_SOCK_OPEN)
        time.sleep(0.1)

        # Check socket status
        status = self._read_socket_register(sock_num, _REG_SNSR)
        if status != _SNSR_SOCK_UDP:
            raise RuntimeError(
                f"Failed to open UDP socket {sock_num}, status: 0x{status:02X}"
            )
        print(f"UDP socket {sock_num} opened on port {port}")

    def udp_sendto(self, sock_num, dest_ip, dest_port, data):
        # Set destination IP
        for i in range(4):
            self._write_socket_register(sock_num, _REG_SNDIPR + i, dest_ip[i])

        # Set destination port
        self._write_socket_register(sock_num, _REG_SNDPORT, (dest_port >> 8) & 0xFF)
        self._write_socket_register(sock_num, _REG_SNDPORT + 1, dest_port & 0xFF)

        # Send data
        self.socket_send(sock_num, data)

    def udp_recvfrom(self, sock_num):
        rx_size = self._read_socket_register(
            sock_num, _REG_SNRX_RSR
        ) << 8 | self._read_socket_register(sock_num, _REG_SNRX_RSR + 1)
        if rx_size > 0:
            # Read RX read pointer
            rx_rd = self._read_socket_register(
                sock_num, _REG_SNRX_RD
            ) << 8 | self._read_socket_register(sock_num, _REG_SNRX_RD + 1)

            # Calculate the physical address
            addr = rx_rd & _SOCK_MASK

            # Read data from RX buffer
            data = self._read_data(sock_num, addr, rx_size)

            # Update RX read pointer
            rx_rd = (rx_rd + rx_size) & 0xFFFF
            self._write_socket_register(sock_num, _REG_SNRX_RD, (rx_rd >> 8) & 0xFF)
            self._write_socket_register(sock_num, _REG_SNRX_RD + 1, rx_rd & 0xFF)

            # Issue RECV command
            self._write_socket_register(sock_num, _REG_SNCR, _CMD_SOCK_RECV)

            # Remove header (8 bytes: source IP, source port, and length)
            return data[8:]
        return None

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
        print(f"Socket {sock_num} port: {port_high << 8 | port_low}")

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
        version_reg_addr = _REG_VERSIONR
        version = self._read(version_reg_addr, 0x00)[0]
        return version

    @property
    def rcr(self) -> int:
        """Retry count register."""
        addr = _REG_RCR
        return int.from_bytes(self._read(addr, 0x00), "big")

    @rcr.setter
    def rcr(self, retry_count: int) -> None:
        """Retry count register."""
        addr = _REG_RCR
        if not (0 <= retry_count <= 255):
            raise ValueError("Retries must be from 0 to 255.")
        self._write(addr, 0x04, retry_count)

    @property
    def rtr(self) -> int:
        """Retry time register."""
        addr = _REG_RTR
        return int.from_bytes(self._read(addr, 0x00, 2), "big")

    @rtr.setter
    def rtr(self, retry_time: int) -> None:
        """Retry time register."""
        addr = _REG_RTR
        if not (0 <= retry_time < 2**16):
            raise ValueError("Retry time must be from 0 to 65535")
        self._write(addr, 0x00, retry_time)


# PIO and State Machine setup
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

# CS and RST pin setup
cs_pin = digitalio.DigitalInOut(board.GP17)
rst_pin = digitalio.DigitalInOut(board.GP20)

# Initialize WIZNET5K
wiznet = WIZNET5K(sm, cs_pin, rst_pin)
wiznet.init()
wiznet.print_network_info()

# After creating the WIZNET5K instance
version = wiznet.read_version()
print(f"W5500 Version: 0x{version:02X}")

wiznet.rtr = 2000  # Set desired retry time value
current_rtr = wiznet.rtr
print(f"RTR: {current_rtr}")

wiznet.rcr = 8
current_rcr = wiznet.rcr
print(f"RCR: {current_rcr}")

# Set socket buffer sizes
tx_buffer_sizes = [2, 2, 2, 2, 2, 2, 2, 2]  # TX buffer size for each socket (in KB)
rx_buffer_sizes = [2, 2, 2, 2, 2, 2, 2, 2]  # RX buffer size for each socket (in KB)
wiznet.socket_set_buffer_size(tx_buffer_sizes, rx_buffer_sizes)

# Initialize socket and start listening
sock_num_tcp = 0
sock_num_udp = 1
port_tcp = 5000
port_udp = 6000

dns_server_ip = [8, 8, 8, 8]

resolved_ip = wiznet.dns_query("google.com", dns_server_ip)

if resolved_ip:
    print(f"Resolved IP: {'.'.join(map(str, resolved_ip))}")

print("Socket Open (TCP)")
wiznet.socket_init(sock_num_tcp)
wiznet.socket_listen(sock_num_tcp, port_tcp)

print("Socket Open (UDP)")
wiznet.socket_open_udp(sock_num_udp, port_udp)

print(f"Listening on TCP port {port_tcp} and UDP port {port_udp}")

while True:
    # Handle TCP socket
    status_tcp = wiznet.socket_status(sock_num_tcp)
    if status_tcp == SNSR_SOCK_ESTABLISHED:  # SOCK_ESTABLISHED
        # Receive data from the client
        data = wiznet.socket_recv(sock_num_tcp)
        if data:
            print(f"[TCP] Received: {data}")
            # Echo the data back to the client
            wiznet.socket_send(sock_num_tcp, data)
    elif status_tcp == SNSR_SOCK_CLOSE_WAIT:  # SOCK_CLOSE_WAIT
        print("[TCP] Client disconnected, closing socket")
        wiznet.socket_close(sock_num_tcp)
        wiznet.socket_init(sock_num_tcp)
        wiznet.socket_listen(sock_num_tcp, port_tcp)
    elif status_tcp == SNSR_SOCK_LISTEN:  # SOCK_LISTEN
        pass  # Listening for incoming connections
    elif status_tcp == SNSR_SOCK_CLOSED:  # SOCK_CLOSED
        wiznet.socket_init(sock_num_tcp)
        wiznet.socket_listen(sock_num_tcp, port_tcp)

    # Handle UDP socket
    data_udp = wiznet.udp_recvfrom(sock_num_udp)
    if data_udp:
        print(f"[UDP] Received: {data_udp}")
        # Echo the data back to the sender (for demonstration purposes)
        wiznet.udp_sendto(
            sock_num_udp, [192, 168, 11, 63], port_udp, data_udp
        )  # Replace with the appropriate destination IP and port
        time.sleep(1)
