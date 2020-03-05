from flask import Flask
from threading import Thread

class Server:
    def __init__(self, Thread):
        Thread.__init__()
        self.app=Flask(__name__)
        self.buffer = []
        self.timestamps = []

    def queue(self, M, W, B, Q):
        ts = time.time()
        self.buffer.append(str(ts)+"<br/>"+repr(runner.M)+"<br/>"+repr(W)+"<br/>"+repr(runner.B)+"<br/>"+repr(Q.vec()))
        self.timestamps.append(ts)

    @self.app.route('/')
    def index():
        return '<br/><br/>'.join(self.buffer)

    def run(self):
        app.run(host='0.0.0.0')
