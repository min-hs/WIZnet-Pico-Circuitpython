import board
import digitalio
import adafruit_pioasm
import rp2pio
import adafruit_wiznet5k.adafruit_wiznet5k_socketpool as socketpool
from adafruit_wiznet5k.adafruit_wiznet5k import WIZNET5K

print("Wiznet5k SimpleServer Test")

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

# PIO 및 State Machine 설정
assembled = adafruit_pioasm.assemble(spi_master)
spi_sm = rp2pio.StateMachine(
    assembled,
    frequency=1_000_000,
    first_out_pin=board.GP19,  # MOSI
    first_in_pin=board.GP16,  # MISO
    first_set_pin=board.GP18,  # SCK
    out_pin_count=1,
    in_pin_count=1,
    set_pin_count=1,
    in_shift_right=False,
    out_shift_right=False,
    push_threshold=8,
    pull_threshold=8,
)

# CS 핀 설정
cs = digitalio.DigitalInOut(board.GP17)
cs.direction = digitalio.Direction.OUTPUT
cs.value = True

# WIZNET5K 초기화
eth = WIZNET5K(spi_sm, cs, is_dhcp=True)
print(eth.chip)

# # 소켓 풀과 서버 초기화
# pool = socketpool.SocketPool(eth)
# server = pool.socket()  # 서버 소켓 할당
# server_ip = eth.pretty_ip(eth.ip_address)  # 서버의 IP 주소 가져오기
# server_port = 5000  # 사용할 포트 번호
# server.bind((server_ip, server_port))  # IP와 포트 바인딩
# server.listen()  # 클라이언트 연결 대기 시작

# while True:
#     print(f"Accepting connections on {server_ip}:{server_port}")
#     conn, addr = server.accept()  # 클라이언트 연결 대기
#     print(f"Connection accepted from {addr}, reading exactly 1024 bytes from client")
#     with conn:
#         data = conn.recv(1024)
#         if data:  # 데이터를 받을 때까지 대기
#             print(data)
#             conn.send(data)  # 받은 데이터를 클라이언트에게 에코
#     print("Connection closed")
