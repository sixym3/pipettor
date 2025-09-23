import socket
host = '192.168.1.101'
port = 54321         # Remains the same, because it is specified as default port in URCaps code
s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.connect((host, port))
s.sendall(b'V')
data = s.recv(1024)
s.close()
print('Received', repr(data))