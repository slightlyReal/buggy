import socket

UDP_IP = "0.0.0.0"
UDP_PORT = 14550

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((UDP_IP, UDP_PORT))

print(f"Listening for UDP packets on {UDP_IP}:{UDP_PORT}...")
while True:
    data, addr = sock.recvfrom(1024)
    try:
        message = data.decode('utf-8')
        print(f"Received message: {message} from {addr}")
    except UnicodeDecodeError:
        print(f"Received non-UTF8 data: {data} from {addr}")
