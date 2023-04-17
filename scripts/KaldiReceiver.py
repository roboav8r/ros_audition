#!/usr/bin/python3
# -*- coding: utf-8 -*-

import socket
import re
import time

def time2name():
    x = time.localtime()
    return "%04d_%02d%02d_%02d%02d%02d" % (x.tm_year, x.tm_mon, x.tm_mday, x.tm_hour, x.tm_min, x.tm_sec)


class KaldiReceiver:
    """ KaldiReceiver. Connects to kaldi and parses the recognition result."""
    def __init__(self, host="localhost", port=10500, tosec=30):
        """ Initialize the proxy. connect to kaldi -module."""
        self.sock = socket.socket(socket.AF_INET,
                                  socket.SOCK_STREAM)
        for x in range(tosec):
            try:
                self.sock.connect((host, port))
                break
            except socket.error as e:
                time.sleep(1)
        print("KaldiReceiver connected.")
        self.pattern = re.compile('([A-Z]+=".*?")')

        self.isKaldiRunning = True
        self.logname = "KaldiReceiver" + time2name() + ".txt"
        #self.loghandle = open(self.logname, "w")
        self.srcinfbuf = {}

    def getResult(self):
        """ Receive result as XML format."""
        msg = []
        # receive all messages
        while True:
            msg.append(self.sock.recv(1024).decode('utf-8'))
            if len(msg[-1]) == 0:
                time.sleep(0.001)
                continue

            # check if kaldi is died
            print("waiting", len(msg))
            if len([m for m in msg if m == '']) > 10000:
                return None

            if "</RECOGOUT>" in msg[-1]:
                break
        # connect them all, and split with \n
        self.msg = "".join([m.replace(".\n", "") for m in msg])
        self.msg = self.msg.split("\n")
        #print(self.msg)
        srcinfs = [line for line in self.msg if "<SOURCEINFO" in line]
        for srcinf in srcinfs:
            common = {}
            srcinf = srcinf[12:-2].split()  # remove "<SOURCEINFO " and "/>" after split
            key = ["AZIMUTH", "ELEVATION", "SOURCEID", "SEC", "USEC"]
            fun = [float, float, int, int, int]
            for k, f in zip(key, fun):
                prop = [line for line in srcinf if k in line][0]
                common[k] = f(prop.split("=")[1][1:-1])
            self.srcinfbuf[common["SOURCEID"]] = common
        #self.loghandle.writelines(self.msg)
        return self.msg

    def parseResult(self):
        """ run after getResult. it parses the reseult
        and returns a dictionary having results"""

        # srcinf
        srcinfs = [line for line in self.msg if "<RECOGOUT" in line]
        is_shypo = True if [line for line in self.msg if "<SHYPO" in line] else False
        #print(srcinf, self.msg)
        common = {}
        for srcinf in srcinfs:
            srcinf = srcinf[10:-1].split() # remove "<RECOGOUT " and ">" after split
            srcid = int([line for line in srcinf if "SOURCEID" in line][0].split("=")[1][1:-1])
            if srcid in self.srcinfbuf:
                common = self.srcinfbuf[srcid]
                if is_shypo: # if delete source information data, when you received final result
                    del self.srcinfbuf[srcid]
            else:
                common = {}

        # parse all WHYPO tags
        result = {"SENTENCE":""}
        # add words
        for msg in [m for m in self.msg if "WHYPO" in m]:

            for prop in self.pattern.findall(msg):
                key = prop.split("=")[0]
                value = prop.split('"')[1]

                if key == "WORD":
                    if result["SENTENCE"]:
                        result["SENTENCE"] += " "
                    result["SENTENCE"] += value

        for prop in common.keys():
            result[prop] = common[prop]
        return result

if __name__ == "__main__":
    proxy = KaldiReceiver()
    print("\n".join(proxy.getResult()))
    print("[")
    for result in proxy.parseResult():
        print(result)
    print("]")

