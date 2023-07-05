#!/usr/bin/python3
# -*- coding: utf-8 -*-
import socket
import struct
import numpy
import multiprocessing
import time
import sys
import signal
try:
    import exceptions
except ImportError:
    import builtins as exceptions

import HarkTFreader
import levelmeter

def handler(signum, frame):
    print("shutdown server script!")
    sys.exit()

class HARKServer():
    def __init__(self, use_meter=True, use_filter=False, tf_file="hark_conf/tamago_rectf.zip"):
        self.HDHeader_size = struct.calcsize("<iiiqq")
        self.HDHMicData_size = struct.calcsize("<iii")
        self.NumSources_size = struct.calcsize("<i")
        self.HDHSrcInfo_size = struct.calcsize("<iffff")
        self.HDHSrcData_size = struct.calcsize("<ii")
        self.use_meter = use_meter
        if self.use_meter:
            self.meter = levelmeter.SpetrumPowerMeter()
        self.use_filter = use_filter
        if self.use_filter:
            self.filter = HarkTFreader.HarkTFreader(tf_file)
            if self.filter.valid:
                self.filter_ids = [int(x["id"]) for x in self.filter.source_search_by_angles(-120, -60)]

    def __del__(self):
        if self.use_meter:
            del self.meter
        if self.use_filter:
            del self.filter

    def cart2polar(self, cart):
        return 180.0 / numpy.pi * numpy.arctan2(cart[1], cart[0])

    def get_HDHeader(self, packed_data, debug=False):
        h_dict = {}
        h_array = struct.unpack("<iiiqq", packed_data)
        h_dict = {
                  "type": h_array[0],
                  "advance": h_array[1],
                  "count": h_array[2],
                  "tv_sec": h_array[3],
                  "tv_usec": h_array[4]
                 }
        if debug:
            print("HDHeader:: packet_len: {},".format(len(packed_data)))
            print("type: {type}, advance: {advance}, count: {count}, tv_sec: {tv_sec}, tv_usec: {tv_usec}".format(**h_dict))
        return h_dict

    def get_HDHMicData(self, packed_data, debug=False):
        h_dict = {}
        h_array = struct.unpack("<iii", packed_data)
        h_dict = {
                  "nch": h_array[0],
                  "length": h_array[1],
                  "data_bytes": h_array[2]
                 }
        if debug:
            print("HDHMicData:: packet_len: {},".format(len(packed_data)))
            print("nch: {nch}, length: {length}, data_bytes: {data_bytes}".format(**h_dict))
        return h_dict

    def get_NumSources(self, packed_data, debug=False):
        h_dict = {}
        h_array = struct.unpack("<i", packed_data)
        h_dict = {
                  "sources": h_array[0]
                 }
        if debug:
            print("NumSources:: packet_len: {},".format(len(packed_data)))
            print("sources: {sources}".format(**h_dict))
        return h_dict

    def get_HDHSrcInfo(self, packed_data, debug=False):
        h_dict = {}
        h_array = struct.unpack("<iffff", packed_data)
        h_dict = {
                  "source_id": h_array[0],
                  "x": [h_array[1], h_array[2], h_array[3]],
                  "power": h_array[4]
                 }
        if debug:
            print("HDHSrcInfo:: packet_len: {},".format(len(packed_data)))
            print("source_id: {source_id}, x0: {x[0]}, x1: {x[1]}, x2: {x[2]}, power: {power}".format(**h_dict))
        return h_dict

    def get_HDHSrcData(self, packed_data, debug=False):
        h_dict = {}
        h_array = struct.unpack("<ii", packed_data)
        h_dict = {
                  "length": h_array[0],
                  "data_bytes": h_array[1]
                 }
        if debug:
            print("HDHSrcData:: packet_len: {},".format(len(packed_data)))
            print("length: {length}, data_bytes: {data_bytes}".format(**h_dict))
        return h_dict

    def sockReceiver(self, sock=None, size=None, buflimit=4096):
        """
        socket revicer
        """
        if (sock is None) or (size is None):
            return (False, None)
        else:
            split_size = size
            split_recvs = []
            while (split_size > 0):
                if (split_size > buflimit):
                    split_recvs.append(buflimit)
                else:
                    split_recvs.append(split_size)
                split_size -= buflimit
            recv_data = []
            for remain in split_recvs:
                complete = False
                continue_loop = True
                while continue_loop:
                    packed_data = sock.recv(remain)
                    recv_data.append(packed_data)
                    if len(packed_data) == remain:
                        continue_loop = False
                        complete = True
                    elif len(packed_data) == 0:
                        continue_loop = False
                    remain -= len(packed_data)
            return (complete, b''.join(recv_data))

    def portVector(self, sock, debug=False, dump=False):
        if self.use_meter:
            # Disabled print function, because curses will handled terminal control.
            debug = False
            dump = False
        hdheader = {}
        hdhsrcdata = {}
        stage = 0
        HDHeader_size = self.HDHeader_size
        HDHSrcData_size = self.HDHSrcData_size
        try:
            while True:
                if stage == 0:
                    if debug:
                        print("[DEBUG]:: reciving...")
                    packed_data = sock.recv(HDHeader_size)
                    if len(packed_data) == HDHeader_size:
                        hdheader = self.get_HDHeader(packed_data, debug=debug)
                        stage = 1
                    elif len(packed_data) == 0:
                        break
                    else:
                        time.sleep(0.01)
                elif stage == 1:
                    packed_data = sock.recv(HDHSrcData_size)
                    hdhsrcdata = self.get_HDHSrcData(packed_data, debug=debug)
                    length = hdhsrcdata["length"]
                    size = hdhsrcdata["data_bytes"]
                    frame_data = []
                    if debug:
                        print("[DEBUG]:: correct_size: {}".format(size))
                    while size != 0:
                        part_data = sock.recv(size)
                        frame_data.append(part_data)
                        size -= len(part_data)
                        if debug:
                            print("[DEBUG]:: packet_len: {}, remain_size: {}".format(len(part_data), size))
                    unpacked_data = struct.unpack("f"*(length), b''.join(frame_data))
                    if sum(unpacked_data) != 0.0:
                        if self.use_meter:
                            if self.use_filter and self.filter and self.filter.valid:
                                for i in [unpacked_data[i] for i in self.filter_ids]:
                                    self.meter.update(i)
                            else:
                                for i in unpacked_data:
                                    self.meter.update(i)
                            self.meter.draw()
                        if dump:
                            print(unpacked_data)
                    if debug:
                        print("[DEBUG]:: frame data recived!!")
                    stage = 0
                else:
                    raise exceptions.ValueError(stage)
        finally:
            if debug:
                print("[DEBUG]:: closed")
            sock.close()

if __name__ == "__main__":
    debug = False
    dump = False
    signal.signal(signal.SIGINT, handler)

    hserver = HARKServer(use_meter=(not (debug or dump)))
    hark_ip = "127.0.0.1" if len(sys.argv) < 2 else str(sys.argv[1])
    hark_port = 4321 if len(sys.argv) < 3 else int(sys.argv[2])

    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    if debug:
        print("[DEBUG]:: Socket created")

    # Bind socket to local host and port
    try:
        s.bind((hark_ip, hark_port))
    except socket.error as msg:
        if debug:
            print("[DEBUG]:: Bind failed. Error Code : {} Message {}".format(str(msg[0]), msg[1]))
        sys.exit()
    if debug:
        print("[DEBUG]:: Socket bind complete")

    # Start listening on socket
    s.listen(10)
    if debug:
        print("[DEBUG]:: Socket now listening")

    # wait to accept a connection - blocking call
    (conn, addr) = s.accept()
    if debug:
        print("[DEBUG]:: Connected with {}:{}".format(addr[0], str(addr[1])))

    procs = []
    procs.append(multiprocessing.Process(target=hserver.portVector, args=(conn, debug, dump,)))
    for proc in procs:
        proc.start()
    del hserver

