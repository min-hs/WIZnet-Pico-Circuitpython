import board
import rp2pio
import adafruit_pioasm
import digitalio
import time

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


class WIZNET5K:
    def __init__(self, sm, cs_pin, rst_pin):
        self._sm = sm
        self._cs = digitalio.DigitalInOut(cs_pin)
        self._cs.direction = digitalio.Direction.OUTPUT
        self._cs.value = True
        self._rst = digitalio.DigitalInOut(rst_pin)
        self._rst.direction = digitalio.Direction.OUTPUT
        self._chip_type = "w5500"  # W5500만 사용

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
        mac_address = [0x00, 0x08, 0xDC, 0x01, 0x02, 0x03]
        for i, val in enumerate(mac_address):
            self.write_reg(0x0009 + i, 0, val)
        # Set IP Address
        ip_address = [192, 168, 11, 100]
        for i, val in enumerate(ip_address):
            self.write_reg(0x000F + i, 0, val)
        # Set Gateway IP
        gateway_ip = [192, 168, 11, 1]
        for i, val in enumerate(gateway_ip):
            self.write_reg(0x0001 + i, 0, val)
        # Set Subnet Mask
        subnet_mask = [255, 255, 255, 0]
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
cs_pin = board.GP17
rst_pin = board.GP20

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
    wiznet.print_socket_status(sock_num)
    status = wiznet.socket_status(sock_num)
    print(f"socket status: {status:02X}")
    if status == 0x13:  # SOCK_INIT
        print("Socket initialized, listening...")
        wiznet.socket_listen(sock_num, port)
    elif status == 0x14:  # SOCK_LISTEN
        print("Waiting for connection...")
    elif status == 0x17:  # SOCK_ESTABLISHED
        print("Connection established")
        data = wiznet.socket_recv(sock_num)
        if data:
            print(f"Received: {data}")
            wiznet.socket_send(sock_num, data)  # Echo back
    elif status == 0x1C:  # SOCK_CLOSE_WAIT
        print("Connection closing, reinitializing socket")
        wiznet.socket_close(sock_num)
        wiznet.socket_init(sock_num)
    time.sleep(1)  # Increase delay for debugging
