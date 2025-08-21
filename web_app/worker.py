# Based on https://github.com/Terrabits/tkinter-multiprocessing-example/blob/master/lib/worker.py

import ctypes
import multiprocessing
import time
import subprocess
import signal

ARGOS = '/usr/bin/argos3'

class Worker(object):
    def __init__(self):
        self._is_alive  = multiprocessing.Value(ctypes.c_bool, True)
        self.process = None

    def get_is_alive(self):
        with self._is_alive.get_lock():
            return self._is_alive.value
    def set_is_alive(self, value):
        with self._is_alive.get_lock():
            self._is_alive.value = value
    is_alive = property(get_is_alive, set_is_alive)

    def run(self):
        while True:
            if not self.is_alive:
                self.process.send_signal(signal.SIGINT)
                return
            time.sleep(0.1)


class SimulationWorker(Worker):
    def __init__(self, scenario):
        super().__init__()
        self.scenario   = scenario
    
    def run(self):
        print("starting simulation: {}".format(self.scenario))
        self.process = subprocess.Popen([ARGOS, '-c', self.scenario])
        super().run()


class WebClientWorker(Worker):
    def run(self):
        print("starting webclient")
        self.process = subprocess.Popen(['python3', '-m', 'http.server', '8000'], cwd='web_client')
        super().run()


class Process(object):
    def __init__(self):
        self.worker   = None
        self.process  = None

    def __delete__(self):
        self.stop()

    def start(self, worker):
        self.stop()
        self.worker = worker
        self.process = multiprocessing.Process(target=self.worker.run)
        self.process.start()

    def stop(self):
        if self.is_alive:
            self.worker.is_alive = False
            self.process.join()
            self.worker  = None
            self.process = None

    def get_is_alive(self):
        return bool(self.worker and self.process)
    is_alive = property(get_is_alive)


class SimulationProcess(Process):
    def __init__(self, scenario):
        super().__init__()
        self.scenario = scenario

    def start(self):
        super().start(SimulationWorker(self.scenario))


class WebClientProcess(Process):
    def start(self):
        super().start(WebClientWorker())
