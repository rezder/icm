"""
Senser server data recived from arduino nano esp32
"""
import serial
import numpy as np
import netifaces
from subprocess import Popen, PIPE, STDOUT
import socket
import time
import asyncio
import bleak
import contextlib


CMD_WIFI_CLIENT = "W"
CMD_WIFI_SERVER = "V"
CMD_SER = "S"
CMD_END = "E"
CMD_RESTART = "R"
CMD_SER_OD = "O"
CMD_DATA = "D"
CMD_BLE = "B"

DEFAULT_BAUDRATE = 115200


def bytesToString(bytes):
    return bytes.decode("ascii")


def esp32HavsmolfData(buffer):
    floatSize = 4
    res = esp32HavsmolfDataBme(buffer[:3*floatSize])
    res.update(esp32HavsmolfDataIcm(buffer[3*floatSize:]))
    return res


def esp32HavsmolfDataIcm(buffer):
    floatsNO = 9
    floatSize = 4
    dt = int.from_bytes(buffer[floatsNO * floatSize:], "little")
    sensor = np.frombuffer(buffer[:floatsNO * floatSize], dtype=np.float32)
    res = {"dt": dt}
    res["mag"] = (sensor[0], sensor[1], sensor[2])
    res["acc"] = (sensor[3], sensor[4], sensor[5])
    res["gyro"] = (sensor[6], sensor[7], sensor[8])
    return res


def esp32HavsmolfDataBme(buffer):
    floatsNO = 3
    floatSize = 4
    sensor = np.frombuffer(buffer[:floatsNO * floatSize], dtype=np.float32)
    res = {}
    res["hum"] = sensor[0]
    res["temp"] = sensor[1]
    res["press"] = sensor[2]
    return res


def esp32DataSize():
    floatsNO = 12
    floatSize = 4
    intsNo = 1
    intsSize = 4
    return floatsNO * floatSize + intsNo * intsSize


def getSsid() -> str:
    cmdIwgetid = "/usr/bin/iwgetid"
    cmd = [cmdIwgetid, "-r"]
    pCmd = Popen(cmd, text=True, stdout=PIPE, stderr=STDOUT)
    txt, err = pCmd.communicate()
    if err:
        raise Exception("Fail to get wifi ssid")
    res = txt.strip()
    return res


class TimeOutErr(Exception):
    pass


async def event_wait(evt, timeout):
    # suppress TimeoutError because we'll return False in case of timeout
    with contextlib.suppress(asyncio.TimeoutError):
        await asyncio.wait_for(evt.wait(), timeout)
    return evt.is_set()


class BleEsp:
    def __init__(self,
                 path,
                 delay,
                 interval,
                 deviceAddr,
                 ):
        self.path = path
        self.delay = delay
        self.interval = interval
        self.serial = None
        self.deviceAddr = deviceAddr
        self.device = None
        self.devices = dict()
        self.stopRunEvent = None
        self.stopScanEvent = None

    def __enter__(self):
        self.startSerialCon()
        return self

    def __exit__(self, exc_type, exc_value, traceback):
        """
        return false raise exceptions
        return true suppresses exceptions
        exc_xxx is the exception info
        from the block.
        """
        self.closeSerialCon()
        return False

    def _scanCallback(self, device, advData):
        if device.address == self.deviceAddr:
            self.device = device
            self.stopScanEvent.set()
        else:
            self.devices[device.address] = device

    async def _scanBle(self, timeOut):
        self.stopScanEvent = asyncio.Event()
        async with bleak.BleakScanner(self._scanCallback):
            isTimeOut = await event_wait(self.stopScanEvent, timeOut)
        return isTimeOut

    async def _runBle(self, bmeCallable, icmCallable):
        intsSize = 4
        async with bleak.BleakClient(self.device) as client:
            print("Connected to {}".format(self.device))
            speedCh = bleak.uuids.normalize_uuid_32(101)
            buffer = await client.read_gatt_char(speedCh)
            dt = int.from_bytes(buffer[:intsSize], "little")
            interval = int.from_bytes(buffer[intsSize:], "little")
            print("current dt: {}, interval: {}".format(dt, interval))
            if dt != self.delay or interval != self.interval:
                data = bytearray(self.delay.to_bytes(4, byteorder="little"))
                for b in self.interval.to_bytes(4, byteorder="little"):
                    data.append(b)
                await client.write_gatt_char(speedCh, data)
                await asyncio.sleep(1)

            txt = "Delay is {}, and interval is {}"
            print(txt.format(self.delay, self.interval))
            bmeCh = bleak.uuids.normalize_uuid_32(102)
            icmCh = bleak.uuids.normalize_uuid_32(103)
            await client.start_notify(bmeCh, bmeCallable)
            await client.start_notify(icmCh, icmCallable)
            print("Starting notifying on bme and icm")
            self.stopRunEvent = asyncio.Event()
            await self.stopRunEvent.wait()
            print("Stop notifying signal recieved")
            await client.stop_notify(bmeCh)
            await client.stop_notify(icmCh)

    async def runBle(self, bmeCallable, icmCallable):
        """
        Connect to a bluetooth device and
        Call the functions with the data.
        icm after every delay and bme after
        delay * interval.
        """
        timeOut = 20
        print("Start ble scan for {} seconds".format(timeOut))
        isFound = await self._scanBle(timeOut)

        if not isFound:
            errTxt = "Search for {} seconds. Failed to find: {} found:\n {}"
            raise TimeOutErr(errTxt.format(timeOut,
                                           self.deviceAddr,
                                           self.devices))
        else:
            print("Found {}".format(self.deviceAddr))

        await self._runBle(bmeCallable, icmCallable)

    def stopBle(self):
        """
        Expect to be set from a external thread
        it is thread safe for now because
        it only have one waitor and use the event
        as a lock.
        """
        while self.stopRunEvent is None:
            time.sleep(2)

        print("Send stop ble to asyncio thread")
        self.stopRunEvent.set()  # this is not thread safe as self.wait
        #  needs a lock to make sure it gets a messages. I use stopRunEvent
        # as a lock and only add one waiter. Stopping in the middle of a scan
        # is not possible or you have to wait until it is done

    def startSerialCon(self):
        self.serial = serial.Serial(self.path, DEFAULT_BAUDRATE, timeout=2)
        data = bytearray(CMD_BLE.encode(encoding="ascii"))
        self.serial.write(data)
        self.serial.flush()
        print("Setting up Ble on arduino")
        #  self.serial.close()  # TODO remove

    def closeSerialCon(self):
        data = bytearray(CMD_RESTART.encode(encoding="ascii"))
        self.serial.write(data)
        self.serial.flush()
        self.serial.close()
        print("Restart esp")


class SerialEspOnDemand:
    def __init__(
        self, path, dataSize, dataFunc=bytesToString, baudrate=DEFAULT_BAUDRATE
    ):
        self.serial = serial.Serial(path, baudrate=DEFAULT_BAUDRATE, timeout=2)
        self.dataFunc = dataFunc
        self.dataSize = dataSize
        data = bytearray(CMD_SER_OD.encode(encoding="ascii"))
        data2 = baudrate.to_bytes(4, byteorder="little")
        for b in data2:
            data.append(b)
        self.serial.write(data)
        self.serial.flush()
        if baudrate != DEFAULT_BAUDRATE:
            self.serial.baudrate = baudrate  # open and close the connection
            time.sleep(1)
        _ = self.readSensorData()
        print("Connect to arduino serial on demand")

    def readSensorData(self):
        data = bytearray(CMD_DATA.encode(encoding="ascii"))
        self.serial.write(data)
        self.serial.flush()
        readData = self.serial.read(self.dataSize)  # blocks with timeout
        missing = self.dataSize - len(readData)
        if missing != 0:
            txt = "Sensor data read missing {} bytes".format(missing)
            raise TimeOutErr(txt)
        return self.dataFunc(readData)

    def stop(self):
        data = bytearray(CMD_RESTART.encode(encoding="ascii"))
        self.serial.write(data)
        self.serial.flush()
        self.serial.close()


class SerialEsp:
    def __init__(
        self,
        path,
        dataSize,
        dataFunc=bytesToString,
        baudrate=DEFAULT_BAUDRATE,
        dataSpeed=200,
    ):
        self.serial = serial.Serial(path, baudrate=DEFAULT_BAUDRATE, timeout=2)
        self.dataFunc = dataFunc
        self.dataSize = dataSize
        self.dataSpeed = dataSpeed
        data = bytearray(CMD_SER.encode(encoding="ascii"))
        dataTmp = baudrate.to_bytes(4, byteorder="little")

        for b in dataTmp:
            data.append(b)

        for b in self.dataSpeed.to_bytes(4, byteorder="little"):
            data.append(b)

        self.serial.write(data)
        self.serial.flush()
        if baudrate != DEFAULT_BAUDRATE:
            self.serial.baudrate = baudrate  # open and close the connection
            time.sleep(1)

        _ = self.serial.read(self.dataSize)  # wait setup arduino
        print("Connected to arduino serial")

    def readSensorData(self):
        readData = self.serial.read(self.dataSize)  # blocks with timeout

        peekNo = self.serial.inWaiting()
        # normal serial kernel buffer size  is 4096
        if peekNo > 3 * self.dataSize:
            txt = ("Sensor data read falling behind"
                   "buffer contain after read  {} bytes")
            raise TimeOutErr(txt.format(peekNo))

        missing = self.dataSize - len(readData)
        if missing != 0:
            txt = "Sensor data read missing {} bytes".format(missing)
            raise TimeOutErr(txt)

        return self.dataFunc(readData)

    def stop(self):
        data = bytearray(CMD_RESTART.encode(encoding="ascii"))
        self.serial.write(data)
        self.serial.flush()
        self.serial.close()


class UdpClientEsp:
    def __init__(
        self,
        path,
        dataSize,
        pw,
        ssid=None,
        dataFunc=bytesToString,
        dataSpeed=200,
        interface="wlp2s0",
        port=12101,  # need firewall acces for upd and ip
    ):
        self.serial = serial.Serial(path, baudrate=DEFAULT_BAUDRATE, timeout=2)
        self.dataFunc = dataFunc
        self.dataSize = dataSize
        self.interface = interface
        self.dataSpeed = dataSpeed
        #  This assumes first ip address is the one this may not be the case
        #  if more exist on the same interface.
        ifInfo = netifaces.ifaddresses(interface)
        self.ip = ifInfo[netifaces.AF_INET][0]["addr"]
        if ssid is None:
            self.ssid = getSsid()
        else:
            self.ssid = ssid
        self.pw = pw
        self.port = port
        self.udpSocket = socket.socket(
            socket.AF_INET, socket.SOCK_DGRAM  # Internet and udp
        )
        self.udpSocket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.udpSocket.bind((self.ip, self.port))
        txt = "Start server listening on ip:{} port:{}"
        print(txt.format(self.ip, self.port))

        txt = CMD_WIFI_CLIENT + self.ssid + "\n"
        txt = txt + self.pw + "\n" + self.ip + "\n"
        data = bytearray(txt.encode(encoding="ascii"))

        dataTmp = self.port.to_bytes(4, byteorder="little")
        for b in dataTmp:
            data.append(b)
        for b in self.dataSpeed.to_bytes(4, byteorder="little"):
            data.append(b)

        self.serial.write(data)
        self.serial.flush()
        self.serial.close()
        self.udpSocket.settimeout(120.0)
        _ = self.udpSocket.recv(self.dataSize)
        print("Connected to Udp arduino")
        self.udpSocket.settimeout(2)

    def readSensorData(self):
        readData = self.udpSocket.recv(self.dataSize)  # blocks for 2  seconds
        return self.dataFunc(readData)

    def stop(self):
        self.serial.open()
        data = bytearray(CMD_RESTART.encode(encoding="ascii"))
        self.serial.write(data)
        self.serial.flush()
        self.serial.close()
        self.udpSocket.close()


class UdpServerEsp:
    def __init__(
        self,
        path,
        dataSize,
        pw,
        ssid=None,
        dataFunc=bytesToString,
        interface="wlp2s0",
        ip="192.168.0.111",
    ):
        self.serial = serial.Serial(path, baudrate=DEFAULT_BAUDRATE, timeout=2)
        self.dataFunc = dataFunc
        self.dataSize = dataSize
        self.interface = interface
        #  This assumes first ip address is the one this may not be the case
        #  if more exist on the same interface.
        iface = netifaces.ifaddresses(interface)
        self.ip = iface[netifaces.AF_INET][0]["addr"]
        if ssid is None:
            self.ssid = getSsid()
        else:
            self.ssid = ssid
        self.pw = pw
        self.port = 12100
        self.ip = ip
        self.udpSocket = socket.socket(
            socket.AF_INET, socket.SOCK_DGRAM  # Internet and udp
        )
        self.udpSocket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        # self.udpSocket.bind((self.ip, self.port))
        txt = "Start client to server on ip:{} port:{}"
        print(txt.format(self.ip, self.port))

        txt = CMD_WIFI_SERVER + self.ssid + "\n" + self.pw + "\n"
        data = bytearray(txt.encode(encoding="ascii"))
        self.serial.write(data)
        self.serial.flush()
        self.serial.close()
        time.sleep(8)
        self.udpSocket.settimeout(5.0)
        data = bytearray(CMD_DATA.encode(encoding="ascii"))
        tries = 1
        while True:
            try:
                self.udpSocket.sendto(data, (self.ip, self.port))
                _ = self.udpSocket.recv(self.dataSize)
                time.sleep(0.12)
                break
            except TimeoutError:
                print("Try {}".format(tries))
                if tries == 30:
                    raise TimeOutErr("Tried 30 times to get a connecting")
                tries = tries + 1
            except OSError as oerr:
                print("Try {}".format(tries))
                print(oerr)
                if tries == 30:
                    raise TimeOutErr("Tried 30 times to get a connecting")
                tries = tries + 1
            time.sleep(2)
        print("Connected to Udp arduino")

        self.udpSocket.settimeout(3)

    def readSensorData(self):
        data = bytearray(CMD_DATA.encode(encoding="ascii"))
        self.udpSocket.sendto(data, (self.ip, self.port))
        readData = self.udpSocket.recv(self.dataSize)  # blocks for 2  seconds
        return self.dataFunc(readData)

    def stop(self):
        self.serial.open()
        data = bytearray(CMD_RESTART.encode(encoding="ascii"))
        self.serial.write(data)
        self.serial.flush()
        self.serial.close()
        self.udpSocket.close()
