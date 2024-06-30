import sys
import json
import logging
import queue
import threading
import time
import serial
import traceback
import struct
import numpy as np
import time

datalen = 133
angles = []
def calculate_quaternions(datarec):
    """根据字节数据计算四元数"""

    # 转换为字节数组
    byte_data = bytes.fromhex(datarec)
    if len(byte_data) != 8:
        # print(f"Invalid data length: {len(byte_data)}")
        return None

    quaternions = []
    for i in range(0, len(byte_data), 2):
        low_byte = byte_data[i]
        high_byte = byte_data[i + 1]
        value = (high_byte << 8) | low_byte
        if value & 0x8000:  # 检查符号位
            value = -((~value & 0x7FFF) + 1)  # 将值转换为负数
        quaternion = value / 32768.0
        quaternions.append(quaternion)

    return quaternions


def extract_between_markers(data='5559c13949ffefff3872885559c13949ffefff387288', marker='5559'):
    """从数据中提取两个标记之间的数据"""
    try:
        # 查找第一个标记的位置
        first_marker_idx = data.index(marker)
        # 查找第二个标记的位置
        second_marker_idx = data.index(marker, first_marker_idx + len(marker))

        # 提取两个标记之间的数据
        data_between = data[first_marker_idx + len(marker):second_marker_idx - 2]

        return data_between
    except ValueError:
        # print("Marker not found twice in the data")
        return None
def quaternion_conjugate(q):
    return np.array([q[0], -q[1], -q[2], -q[3]])
for i in range(10):
    angles.append([1,0,0,0])
class UARTTable:
    def __init__(self, port, baudrate, port_wit, baudrate_wit, logging_level=logging.INFO):
        self.port = port
        self.baudrate = baudrate
        self.port_wit = port_wit
        self.baudrate_wit = baudrate_wit
        self.ser = None
        self.ser_wit = None
        self.stop_sig = threading.Event()
        self.stop_sig.clear()
        self.buffer = b''  # 初始化数据缓冲区

        self.data_table = {}

        self.logger = logging.getLogger(self.__class__.__name__)
        self.logger.setLevel(logging_level)
        self.float_numbers = None
        self.i = 0
        self.calibration_quaternions = []
        self.initial_angles = []
        self.q = [1,0,0,0]
        self.ser1 = 0
        self.ser2 = 0

        if not self.logger.handlers:
            ch = logging.StreamHandler()
            ch.setStream(sys.stdout)
            ch.setLevel(logging.DEBUG)
            # ch.setFormatter(logging.Formatter("%(asctime)s %(levelname)s [%(name)s::%(threadName)s]: %(message)s", datefmt="%Y-%m-%d %H:%M:%S"))
            ch.setFormatter(
                logging.Formatter("%(asctime)s %(levelname)s [%(name)s]: %(message)s", datefmt="%Y-%m-%d %H:%M:%S"))
            self.logger.addHandler(ch)

    def connect(self):
        self.logger.info("Connecting to serial port {0}...".format(self.port))
        # close any exising connection
        if self.ser:
            self.ser.close()
        while 1:
            # if main want to exit, we no longer try connect to port
            if self.stop_sig.is_set():
                return
            try:
                self.ser = serial.Serial(self.port, baudrate=self.baudrate)
            except serial.serialutil.SerialException as e:
                self.logger.error(str(e))
                self.logger.debug("Still trying...")
                time.sleep(0.5)
                continue

            # we only reach here if serial connection is established
            if self.ser.is_open:
                break

        self.logger.info("Connected.")
        self.ser1 = 1

    def connect2(self):
        self.logger.info("Connecting to serial port {0}...".format(self.port))
        # close any exising connection
        if self.ser_wit:
            self.ser_wit.close()
        while 1:
            # if main want to exit, we no longer try connect to port
            if self.stop_sig.is_set():
                return
            try:
                self.ser_wit = serial.Serial(self.port_wit, baudrate=self.baudrate_wit)
            except serial.serialutil.SerialException as e:
                self.logger.error(str(e))
                self.logger.debug("Still trying...")
                time.sleep(0.5)
                continue

            # we only reach here if serial connection is established
            if self.ser_wit.is_open:
                break

        self.logger.info("Connected.")
        self.ser2 = 1

    def stop(self):
        self.stop_sig.set()
        if self.ser:
            self.ser.close()
        self.logger.info("Stopped.")

    def start(self):
        self.logger.debug("Starting...")
        self.recvPacket()

    def startThreaded(self):
        self.logger.debug("Starting thread...")
        self.t = threading.Thread(target=self.recvPacket)
        self.t2 = threading.Thread(target=self.recvPacket2)
        self.t2.start()
        self.t.start()

    def get(self, key):
        return self.data_table.get(key)

    def recvPacket(self):
        self.connect()
        while 1:
            if self.stop_sig.is_set():
                return
            try:
                data = self.ser.read(datalen)
                # print(data)
                self.buffer += data
                # datareadhex = self.ser_wit.read(22).hex()
            except:
                traceback.print_exc()
                self.logger.warning("receive format error.")
            try:
                if len(self.buffer) >= datalen:
                    start_index = self.buffer.find(b'\xaa')
                    if start_index != -1:
                        # 从包头开始读取到包尾结束
                        end_index = start_index + datalen

                        if end_index <= len(self.buffer):
                            packet = self.buffer[start_index:end_index]
                            # print(len(packet))
                            data_chunk = packet[4:datalen - 1]
                            self.float_numbers = struct.unpack('f' * 32, data_chunk)
                            # print(self.float_numbers[8:12])
                            self.buffer = b''
                    else:
                        # 丢弃无效数据
                        self.buffer = b''

                # wit_data = extract_between_markers(datareadhex)
                # q = calculate_quaternions(wit_data)
            except:
                traceback.print_exc()
                self.logger.warning("packet format error.")
            if self.float_numbers:
                angles[0] = self.float_numbers[0:4] if (sum(self.float_numbers[0:4])!=0) else [1,0,0,0]
                angles[1] = self.float_numbers[4:8] if (sum(self.float_numbers[4:8])!=0) else [1,0,0,0]
                angles[2] = self.float_numbers[8:12] if (sum(self.float_numbers[8:12])!=0) else [1,0,0,0]
                angles[3] = self.float_numbers[12:16] if (sum(self.float_numbers[12:16])!=0) else [1,0,0,0]
                angles[4] = [1.000, 0, 0, 0]
                angles[5] = self.float_numbers[16:20] if (sum(self.float_numbers[16:20])!=0) else [1,0,0,0]
                angles[6] = self.float_numbers[20:24] if (sum(self.float_numbers[20:24])!=0) else [1,0,0,0]
                angles[7] = self.float_numbers[24:28] if (sum(self.float_numbers[24:28])!=0) else [1,0,0,0]
                angles[8] = self.q
                angles[9] = [1.000, 0, 0, 0]
            if self.ser1 and self.ser2 :
                if self.i < 300:
                    self.i += 1
                    if len(self.initial_angles) == 0:
                        self.initial_angles = np.array(angles)
                    else:
                        self.initial_angles = (self.initial_angles + np.array(angles)) / 2
                if self.i == 300:
                    self.i += 1
                    for initial_angle in self.initial_angles:
                        self.calibration_quaternions.append(quaternion_conjugate(initial_angle))
                # print(angles)
                if self.i == 301:
                    self.data_table['angles'] = angles
                    self.data_table['calibration_quaternions'] = self.calibration_quaternions
                else:
                    self.data_table['angles'] = None
                    self.data_table['calibration_quaternions'] = None
            else:
                self.data_table['angles'] = None
                self.data_table['calibration_quaternions'] = None


    def recvPacket2(self):
        self.connect2()
        while 1:
            if self.stop_sig.is_set():
                return
            try:
                datareadhex = self.ser_wit.read(22).hex()
                wit_data = extract_between_markers(datareadhex)
                q = calculate_quaternions(wit_data)
                self.q = q
                self.data_table['wit'] = q
                
            except:
                traceback.print_exc()
                self.logger.warning("receive format error.")

#                pass
                # print(q)



if __name__ == "__main__":
    
    FPS = 60
    port = 'COM9'  # 指定串口号，根据实际情况修改
    baudrate = 256000  # 波特率，根据实际情况修改
    port_wit = 'COM15'
    baudrate_wit = 9600  # 波特率，根据实际情况修改
    uart_table = UARTTable(port=port, logging_level=logging.DEBUG,baudrate=baudrate, port_wit = port_wit
    ,baudrate_wit = baudrate_wit)
    print('uart init----------------')
    uart_table.startThreaded()
    while 1:
        time.sleep(1)
        # print(uart_table.get("wit"))
        print(uart_table.get("angles"))

