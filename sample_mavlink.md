# esp32_freertos_wifi_udp
Send sample  UDP messages through WiFi with ESP32
Burada Mavlink mesajlarini nasil gönderebiliriz ona bakacagiz

## Referaslar
- https://github.com/technopolistv/ESP32-MAVLink-Arduino-Example/tree/main

Kullanilan araclar 
- 1 Python
- 2 Mavlink mesajini

## 

- Windows var hotspot olusturmak icin:  
![](docs/1_mobile_hotspot.png)

- "Devices Connected"  hep 0 görünüyor, sorun etmedim.
![](docs/2_mobile_hotspot.png)

- Host (Windows) bilgisayarin IP'si icin

```bash
ipconfig
```
![](docs/3_mobile_hotspot.png)


- To run open "PlatformIO: New Terminal" and 

```bash
# pio run -t upload -e [env_defined_in_platformio.ini] in our case :
pio run -t upload -e sampleUDP
```

- Kod yüklendikten  sonra "Ctrl+Alt+S"

![](docs/4_serial.png)

- Ayni zamanda  [python/udp_read.py](python/udp_read.py)
```
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
```

![](docs/5_python.png)

- Wireshark'ta da cikti görülebilir.
![](docs/6_wireshark.png)