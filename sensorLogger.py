""" Logging framework for sensor data. Logs data to a file with specified
time interval. Data is saved every time the buffer is full in a comma seperated
file. I like the data sensor reader for pi and arduino seperated this is
why this is in a separated file. That can be used both frameworks.
"""
import time
import signal
import numpy as np
import threading

stop = False
def _collect(file, sensorDataF, filterF, stopF, loopMinTime=0, loopSize=200):
    total = 0
    while True:
        i = 0
        buf = np.zeros((loopSize, 3))
        while i < loopSize:
            ts = time.monotonic()
            data = sensorDataF()
            if filterF(data, buf, i):
                buf[i] = data
                i = i + 1

                if i % 10 == 0:
                    print("Added no:{}".format(total+i))

            dt = time.monotonic() - ts
            if dt < loopMinTime:
                time.sleep(loopMinTime - dt)
            if stopF():
                break
        if i > 0:
            np.savetxt(file, buf[:i], delimiter=",")
            file.flush()
            print("saved {} to {} ".format(total, total + i))
            total = total + i
        if stopF():
            break


def log(logFileName, sensorFunc, filterFunc, sampleTime, bufferSize):

    global stop
    with open(logFileName, "w") as file:
        work = threading.Thread(
            target=_collect,
            args=(
                file,
                sensorFunc,
                filterFunc,
                lambda: stop,
            ),
            kwargs={
                "loopMinTime": sampleTime,
                "loopSize": bufferSize,
            },
        )
        work.start()
        signals = {signal.SIGINT, signal.SIGTERM}
        s = signal.sigwait(signals)
        print("Program was interumpted signo: {}".format(s))
        stop = True
        work.join()
