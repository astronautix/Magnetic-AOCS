import sys
import os
sys.path.append(os.path.join(*['..']*3))
from src.scao.quaternion import Quaternion
from flask import Flask
import time
from threading import Thread

class Server(Thread):
    def __init__(self):
        self.app=Flask(__name__)
        Thread.__init__(self)
        self.buffer = []
        self.timestamps = []
        self.toffset = time.time()

    def queue(self, M, W, B, Q):
        ts = time.time() - self.toffset
        self.buffer.append(str(ts)+"<br/>"+repr(M)+"<br/>"+repr(W)+"<br/>"+repr(B)+"<br/>"+repr(Q.vec()))
        self.timestamps.append(ts)

    def index(self):
        return '<br/><br/>'.join(self.buffer)

    def run(self):
        self.app.add_url_rule('/', 'index', lambda: self.index())
        self.app.run(host='0.0.0.0')
