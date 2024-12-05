import os
import socket
import struct
import pickle
import sys
import time
from multiprocessing import Process, Queue, shared_memory
from signal import *
from device_list import device_list, hierarchy
from mathfunc import quatmath
import kinect

client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
node_server_connected = False
device_count = 0
packet_size = 17
start_time = 0
received_packet = 0

# 8 bytes server flag + 8 bytes client flag + 4 bytes data length + 20 devices data at max
maya_pipe_size = 8+8+4+20*packet_size
maya_pipe = shared_memory.SharedMemory(name="/.pipe/maya_pipe", create=True, size=maya_pipe_size)
maya_pipe_buf = maya_pipe.buf

node_server_addr = ("192.168.137.214", 1234)

kinect_process = Process()
q = Queue()


def clean(*args):
    try:
        write_pipe_server_flag("shutdown")
        maya_pipe.close()
        maya_pipe.unlink()
        client.send("StopRead".encode("ascii"))
        client.close()
        kinect_process_stop()
        print("Shutdown")
    except:
        pass


for sig in (SIGABRT, SIGBREAK, SIGILL, SIGINT, SIGSEGV, SIGTERM):
    signal(sig, clean)


def read_pipe_client_flag():
    cmd = maya_pipe_buf[8:16].tobytes().decode()
    return cmd.replace("\0", "")


def read_pipe_data():
    # read data length
    data_len = int.from_bytes(maya_pipe_buf[16:20].tobytes(), sys.byteorder)
    data = maya_pipe_buf[20:20+data_len].tobytes()
    return data


def write_pipe_server_flag(cmd: str):
    maya_pipe_buf[0:8] = cmd.encode() + bytes([0]*(8-len(cmd)))


def write_pipe_data(data: bytes):
    # data length in int
    maya_pipe_buf[16:20] = len(data).to_bytes(4, byteorder=sys.byteorder)
    # data
    maya_pipe_buf[20:20+len(data)] = data


def read():
    _data = b''
    data_len = int.from_bytes(client.recv(4), "little")
    while len(_data) < data_len:
        _data += client.recv(data_len - len(_data))
    return _data


def count_packet():
    global start_time, received_packet
    if time.perf_counter() - start_time >= 1.0:
        print(f"Received {received_packet} in 1s")
        received_packet = 0
        start_time = time.perf_counter()


def node_server_connect():
    global node_server_connected
    if not node_server_connected:
        try:
            client.connect(node_server_addr)
            node_server_connected = True
            print("Node server connected!")
            get_info()
        except Exception as e:
            print(e)
            exit()


def node_server_disconnect():
    global node_server_connected
    try:
        client.close()
        node_server_connected = False
    except Exception as e:
        print(e)


def get_info():
    global device_count
    client.send("GetInfo".encode("ascii"))
    info = read().decode("ascii")
    device_count = int(info[info.find("Devices: ")+9:info.find(" [")])
    print(info)


def maya_ready():
    if read_pipe_client_flag() == "ready":
        return True
    else:
        return False


def rescan():
    client.send("ReScan".encode("ascii"))
    time.sleep(10)
    get_info()


def reset():
    client.send("Reset".encode("ascii"))
    node_server_disconnect()


def start_read():
    global start_time, received_packet
    client.send("StartRead|30".encode("ascii"))
    received_packet = 0
    start_time = 0
    while True:
        maya_data = {}
        data = read()
        if len(data) != device_count * packet_size:
            print(f"Data error: {data}")
            break
        received_packet += 1
        for i in range(device_count):
            node_data = data[i * packet_size:(i + 1) * packet_size]
            w = struct.unpack(">f", node_data[0:4])[0]
            x = struct.unpack(">f", node_data[4:8])[0]
            y = struct.unpack(">f", node_data[8:12])[0]
            z = struct.unpack(">f", node_data[12:16])[0]
            node_name = int(node_data[16])
            joint_name = device_list.get(node_name).get("joint_name")
            maya_data.update({joint_name: (w, x, y, z)})

        for joint in maya_data:
            parent = hierarchy.get(joint, {}).get("parent")
            if parent is not None:
                maya_data[joint] = quatmath.getchildlocalrot(maya_data[parent], maya_data[joint])

        for joint in maya_data:
            maya_data[joint] = zup2MayaYup(maya_data[joint])

        # get pelvis/root position from kinect
        #root_pos, has_body = q.get()
        #maya_data.update({"root_pos": root_pos})

        sendToMaya(maya_data)

        count_packet()


def sendToMaya(data: dict):
    while True:
        if maya_ready():
            write_pipe_data(pickle.dumps(data))
            write_pipe_server_flag("newdata")
            break


def zup2MayaYup(t: tuple):
    # z-up to y-up: x=y, y=z, z=x
    # Maya quat (x, y, z, w)
    return t[2], t[3], t[1], t[0]


def kinect_process_start():
    global kinect_process
    kinect_process = Process(target=kinect.get_root_pos, args=(q,))
    kinect_process.daemon = True
    kinect_process.start()


def kinect_process_stop():
    kinect_process.terminate()


def promt():
    while True:
        node_server_connect()
        cmd = input("Enter command (start, reset, rescan): ")
        if cmd == "start":
            print("Waiting for maya client")
            while True:
                if not maya_ready():
                    time.sleep(0.1)
                else:
                    print("Maya client connected!")
                    break
            kinect_process_start()
            try:
                start_read()
            except Exception as e:
                print(e)
                clean()
            break
        elif cmd == "reset":
            reset()
        elif cmd == "rescan":
            rescan()
        else:
            print("Wrong command!")


if __name__ == '__main__':
    promt()
