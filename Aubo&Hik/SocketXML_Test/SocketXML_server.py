import socket

# 设置服务器地址和端口
server_address = ('localhost', 9999)

# 创建TCP socket对象
sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

# 绑定服务器地址和端口
sock.bind(server_address)

# 开始监听连接
sock.listen(1)

while True:
    print("Waiting for a connection...")
    connection, client_address = sock.accept()
    try:
        print("Connection from", client_address)

        # 接收数据
        data = b''
        while True:
            chunk = connection.recv(1024)
            if not chunk:
                break
            data += chunk

        print("Received:", data.decode('utf'))

    finally:
        # 关闭连接
        connection.close()
