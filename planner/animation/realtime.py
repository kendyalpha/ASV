import socket               # 导入 socket 模块
import array


# import unicode

HOST = '127.0.0.1'
PORT = 9340                # 设置端口号
with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
    s.connect((HOST, PORT))
    while True:
        s.sendall(b'socket')
        data = s.recv(50)
        print(type(data))
        # data = unicode(data, errors='ignore')
        doubles_sequence = array.array('d', data)
        # stringdata = data.decode(encoding='utf-8')
        print('Received', doubles_sequence[1])
        # stringdata = data.decode()
        # print('Received', b)
