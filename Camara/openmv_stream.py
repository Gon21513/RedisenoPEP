#Usar OpenMV IDE para caragar ste codgo a la camara 
#Ir a tools , eraseOnboard dataflash por si hubiera datos previos
#Reset open Mv
#Save open script as main .py
#Reset Open MV
#Deberia estar listo para funcionar 

import socket, time
import numpy as np
import matplotlib.pyplot as plt

HOST = "192.168.4.1"
PORT = 3333
FRAME_BYTES = 4800

def recv_all(sock, n, timeout=5.0):
    sock.settimeout(timeout)
    data = bytearray()
    while len(data) < n:
        chunk = sock.recv(n - len(data))
        if not chunk:
            raise TimeoutError("Conexión cerrada")
        data.extend(chunk)
    return bytes(data)

s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.connect((HOST, PORT))

# primer frame
s.sendall(b'A')
raw = recv_all(s, FRAME_BYTES, timeout=5.0)
img = np.frombuffer(raw, dtype=np.uint8).reshape(60, 80)

plt.ion()
fig, ax = plt.subplots()
h = ax.imshow(img, cmap='gray')
plt.show()

while True:
    s.sendall(b'A')
    raw = recv_all(s, FRAME_BYTES, timeout=5.0)
    img = np.frombuffer(raw, dtype=np.uint8).reshape(60, 80)
    h.set_data(img)
    plt.pause(0.01)
