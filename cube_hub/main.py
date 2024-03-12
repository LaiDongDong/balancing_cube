import threading
import time
import serial


# 定义一个串口类
class SerialPort:
    def __init__(self, port, baud_rate):
        self.port = port
        self.baud_rate = baud_rate
        self.ser = serial.Serial(self.port, self.baudrate)
        self.lock = threading.Lock()

    def send_data(self, data):
        with self.lock:
            self.ser.write(data.encode())

    def receive_data(self):
        while True:
            data = self.ser.readline().decode()
            print("[%s]Received data:" % (time.time().__str__()), data)


# 创建一个串口对象
serial_port = SerialPort("COM7", 115200)

# 创建发送数据线程
send_thread = threading.Thread(target=serial_port.send_data, args=("Hello, World!",))

# 创建接收数据线程
receive_thread = threading.Thread(target=serial_port.receive_data)

# 启动线程
send_thread.start()
receive_thread.start()

# 等待线程结束
send_thread.join()
receive_thread.join()
