#!/usr/bin/python3
# -*- coding: utf-8 -*-

import KaldiReceiver
import json
import subprocess
import time

import os
import os.path

class KaldiDecoderDetector:
    def __init__(self, kaldiport=10500, options=["--config=kaldi_conf/online.conf"], path=""):
        self.python2_mode = False
        try:
            subprocess.DEVNULL
        except AttributeError:
            subprocess.DEVNULL = open(os.devnull, 'wb')
            self.python2_mode = True
        self.options = list(options)
        self.runKaldi(path, jconfs=self.options)
        time.sleep(10)
        self.proxy = KaldiReceiver.KaldiReceiver(port=kaldiport)

    def __del__(self):
        if self.python2_mode:
            subprocess.DEVNULL.close()

    def runKaldi(self, path, jconfs):
        cmd = "{}kaldidecoder {}".format(path, " ".join(jconfs))

        self.proc = subprocess.Popen(cmd, shell=True, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        print("waiting for kaldi...")

    def mainloop(self, interval):
        self.outfilehandle = open("Kaldi_out.txt", "w")
        while True:
            # invoke if kaldi is dead
            self.proxy.getResult()
            result = self.proxy.parseResult()
            #resultdump = json.dumps(result2)
            #print(resultdump)
            if ("SOURCEID" in result) and ("AZIMUTH" in result) and ("SENTENCE" in result):
                print("SourceID:{}, Azimuth:{}, Sentence:{}".format(result["SOURCEID"], result["AZIMUTH"], result["SENTENCE"]))
                self.outfilehandle.write(('%d, %f, %s\n' % (result["SOURCEID"], result["AZIMUTH"], result["SENTENCE"])))
        #self.outfilehandle.close()



if __name__ == "__main__":
    obj = KaldiDecoderDetector()
    obj.mainloop(0.01)

