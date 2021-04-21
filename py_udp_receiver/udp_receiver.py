import socket

sock = socket.socket(socket.AF_INET,    # Internet
                     socket.SOCK_DGRAM)     # UDP

# bind to port
sock.bind(('', 55726))

while True:
    data, addr = sock.recvfrom(2048)    # buffer size is 1024 bytes
    print("received message:", data)