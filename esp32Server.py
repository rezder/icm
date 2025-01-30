""" Serves data from an arduino to signal k server
Connect to the arduino with a serial(usb) connection
and send data to the signal k server with UDP
"""
import sys
import json
import argparse
import time
import socket
from ahrs.utils import WMM
import threading
import signal
import os
import numpy as np

import sensors as sen
##sys.path.append('/home/rho/Python/pi/havsmolf/')
sys.path.insert(0, '/home/rho/Python/pi/havsmolf/')
import utill
import filters


def serve(arduino, skAddr, dt, fi, fb, cfile, isVerbuse):
    """
    Serve data from arduino to the signal k server.
    The time (dt) between sensor update is sat on the
    arduino connection so the python code must be able to
    keep up.
    :param arduino: The arduino serial path ex. /dev/ttyACM0
    :param skAddr: The signal server adrress (ip,port)(127.0.0.1,10129)
    :param dt: Time between data from the arduino sensor reads
    :param fi: Frequency of icm udp message send to signal k server
    :param fb: Frequency of bme udp message send to signal k server
    :param cFile: The configuraion file
    :param isVerbuse: If true dump alot of information for debuging
    """
    skCache, decl, sock, duinoCon, icm = setup(arduino,
                                               skAddr,
                                               dt,
                                               cfile
                                               )
    print("Ip address: {} port: {}".format(skAddr[0], skAddr[1]))
    print("Arduino: {}".format(arduino))
    print("Sensor update frequens: {} milli seconds".format(dt))
    print("Attitude update frequens: every {} sensor update".format(fi))
    print("Invironment update frequens: every {} sensor update".format(fb))
    print("Configuration file: {}".format(cfile))
    sys.stdout.flush()

    work = start(skCache,
                 decl,
                 sock,
                 skAddr,
                 isVerbuse,
                 fi,
                 fb,
                 duinoCon,
                 icm)

    signals = {signal.SIGINT, signal.SIGTERM}
    s = signal.sigwait(signals)
    print("Server was interumpted signo: {}".format(s))
    stop(work)
    sock.close()
    duinoCon.stop()
    print("Server was closed properly")


def setup(duinoPath, addr, dt, cfFile):
    """
    Every thing that can be setup before the server starts must be here
    and every thing that can be check before starting the server must
    be here. When exiting this function the clock is running.
    """
    skCache = utill.SkCache()  # waits for the signalk server 3.5 minuts
    ix = 0
    while True:
        ix = ix + 1
        try:
            height = skCache.getValue("navigation.gnss.antennaAltitude")
            posDict = skCache.getValue("navigation.position")
            lon = posDict["longitude"]
            lat = posDict["latitude"]
            validateSkData(lat, lon, height)
            break
        except (KeyError, ValueError) as err:
            timePast = ix*(ix+1)/2
            print(
                "Faild to get gps data: {} from signal k server waited {} seconds".format(
                    err, timePast
                )
            )
            sys.stdout.flush()
            time.sleep(ix)
            if ix > 34:  # 10 min wait for gps data
                newErr = LookupError("Numbers of lookup exceeded: {}".format(ix))
                raise newErr from err
            try:
                skCache.update()  # This should not happen unless SK has gone down
            except Exception as e:
                print("failed to update signalk data: {}".format(e))

    wmm = WMM()
    print("Wmm model: {} date: {}".format(wmm.model, wmm.modeldate))
    icm = setupIcm(cfFile)
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    wmmMsg = wmmCreateSkMsg()
    wmmTxt, decl = wmmUpdateSkMsg(wmmMsg, skCache, wmm)
    sock.sendto(wmmTxt.encode("utf-8"), addr)
    duinoCon = sen.SerialEsp(duinoPath,
                             sen.esp32DataSize(),
                             sen.esp32HavsmolfData,
                             sen.DEFAULT_BAUDRATE,
                             dataSpeed=dt
                             )
    _ = duinoCon.readSensorData()  # wait for arduino setup

    return skCache, decl, sock, duinoCon, icm


def start(skCache, decl, sock, addr, isVerbuse, fi, fb, duinoCon, icm):
    global stoplx
    stoplx = False

    work = threading.Thread(
        target=run,
        args=(
            skCache,
            decl,
            sock,
            addr,
            lambda: stoplx,
            isVerbuse,
            fi,
            fb,
            duinoCon,
            icm)
    )
    work.start()
    return work


def run(skCache,
        decl,
        sock,
        addr,
        stopf,
        isVerbuse,
        fi,
        fb,
        duinoCon,
        icm):

    bmeSkMsg = bmeInitSkMsg()
    icmSkMsg = icmInitSkMsg()
    if isVerbuse:
        simpleFilter = filters.Simple()
    mainFilter = filters.TiltFilter()
    ix = 0
    while True:

        if stopf():
            break

        data = duinoCon.readSensorData()
        (a, g, m) = (icm.acc(data), icm.gyro(data), icm.mag(data))
        if isVerbuse:
            simpleFilter.update(a, g, m)
        mainFilter.update(a, g, m)
        if 0 == ix % fi:
            roll, pitch, heading = mainFilter.attitude()
            roll, pitch, heading = icm.level(roll, pitch, heading)
            txt = icmUpdSkMsg(icmSkMsg, pitch, roll, heading, decl)
            if isVerbuse:
                icmFilterPrint(icm, simpleFilter, heading, pitch, roll, a, g, m)
            sock.sendto(txt.encode("utf-8"), addr)

        if 0 == ix % fb:
            txt = bmeUpdSkMsg(bmeSkMsg, data)
            if isVerbuse:
                print(txt)
            sock.sendto(txt.encode("utf-8"), addr)

        if ix == fi*fb:
            ix = 0
        else:
            ix = ix + 1


def stop(work):
    global stoplx
    stoplx = True
    work.join()


def initConf():
    Ainv = [
        [0.024731844616575994, 0.0003709303997715101, -0.00017813477897935746],
        [0.0003709303997715119, 0.023994179622130143, -0.00010745912857236311],
        [-0.0001781347789793579, -0.000107459128572364, 0.023959846111139427],
    ]
    b = [[-43.66569525818584], [13.282358287199125], [39.721969917591395]]
    config = {
        "level": {"acc": (0, 0, 0), "gyro": (0, 0, 0)},
        "hardSoftBias": {"Ainv": Ainv, "b": b, "error": 0.5208506356816079},
    }
    return config


def loadConfig(fileName):
    if not os.path.isfile(fileName):
        conf = initConf()
        with open(fileName, "w") as f:
            f.write(json.dumps(conf))
    with open(fileName, "r") as f:
        config = json.load(f)
    return config


def setupIcm(configFileName):
    config = loadConfig(configFileName)
    icm = Icm(config)
    return icm


class Icm:
    """
    Icm sensor data must be calibrated and adjusted before
    pitch roll and heading can be calculated.
    The calibration is in the config file
    and is used to adjust the sender data
    before any calculation of picth roll
    and heading. Only the mag and gyro have calibration.
    The acc have not maybe I should do that it is done in the
    same way as mag. After the calculation of pitch, roll and
    heading they can be adjusted for oritation and leveling if
    the sensor is not level or not facing north.
    """
    def __init__(self, config):
        x, y, z = config.get("level").get("acc")
        self.levelRoll = filters.calcRoll(x, y, z)
        self.levelPitch = filters.calcPitch(x, y, z)
        self.levelHeading = 0
        print("Level Acceleration: {},{},{}".format(x, y, z))
        print("Level Roll,Pitch: {},{}".format(self.levelRoll, self.levelPitch))
        self.calibrationGyro = config.get("level").get("gyro")
        hsb = config.get("hardSoftBias")
        self.Ainv = np.array(hsb.get("Ainv"))
        self.b = np.array(hsb.get("b"))

    def mag(self, sData):
        x, y, z = sData['mag']
        h = np.array([[x, y, z]]).T
        hat = np.matmul(self.Ainv, (h - self.b))
        return (hat[0, 0], hat[1, 0], hat[2, 0])

    def gyro(self, sData):
        x, y, z = sData['gyro']
        lx, ly, lz = self.calibrationGyro
        dx = x - lx
        dy = y - ly
        dz = z - lz
        # dx,dy,dz=0.0,0.0,0.0
        return (dx, dy, dz)

    def acc(self, sData):
        x, y, z = sData['acc']
        # x,y,z=0.0,0.0,9.8
        return x, y, z

    def level(self, roll, pitch, heading):
        updRoll = roll - self.levelRoll
        updPitch = pitch - self.levelPitch
        udpHeading = heading - self.levelHeading
        return updRoll, updPitch, udpHeading


def wmmCreateSkMsg():
    sourceId = "rho.esp32-1"
    variation = {"path": "navigation.magneticVariation", "value": 0.0}
    variationDate = {
        "path": "navigation.magneticVariationAgeOfService",
        "value": 546721,
    }

    updateDict = {
        "$source": sourceId,
        "values": [variation, variationDate],
    }

    msg = {"updates": [updateDict]}
    return msg


def wmmUpdateSkMsg(msgDict, skCache, wmm):
    height = skCache.getValue("navigation.gnss.antennaAltitude")
    posDict = skCache.getValue("navigation.position")
    lon = posDict["longitude"]
    lat = posDict["latitude"]
    wmm.magnetic_field(lat, lon, height)
    declinationDeg = wmm.magnetic_elements["D"]
    declinationRad = filters.degreesToRad(declinationDeg)
    msgDict.get("updates")[0].get("values")[0].update(value=declinationRad)
    msgDict.get("updates")[0].get("values")[1].update(value=time.time())
    txt = json.dumps(msgDict) + "\n"
    return (txt, declinationRad)


def bmeInitSkMsg():
    sourceId = "rho.esp32-1"
    hum = {
        "path": "environment.inside.relativeHumidity",
        "value": 0.0,
    }  # fraction 0.2 20%
    temp = {"path": "environment.inside.temperature", "value": 0.0}  # kalvin c+273.15
    press = {"path": "environment.outside.pressure", "value": 0.0}  # Pa

    updateDict = {
        "$source": sourceId,
        "values": [hum, temp, press],
    }

    msg = {"updates": [updateDict]}
    return msg


def bmeUpdSkMsg(msgDict, sData):
    hum = sData['hum'] / 100.0
    temp = sData['temp'] + 273.15
    press = sData['press'] * 100.0
    msgDict.get("updates")[0].get("values")[0].update(value=hum)
    msgDict.get("updates")[0].get("values")[1].update(value=temp)
    msgDict.get("updates")[0].get("values")[2].update(value=press)
    txt = json.dumps(msgDict) + "\n"
    return txt


def icmInitSkMsg():
    sourceId = "rho.esp32-1"
    att = {
        "path": "navigation.attitude",
        "value": {"roll": 0.0, "pitch": 0.0, "yaw": 0.0},
    }
    head = {"path": "navigation.headingMagnetic", "value": 0.0}
    headTrue = {"path": "navigation.headingTrue", "value": 0.0}

    updateDict = {
        "$source": sourceId,
        "values": [att, head, headTrue],
    }

    msg = {"updates": [updateDict]}
    return msg


def icmUpdSkMsg(msgDict, pitch, roll, heading, declination):
    msgDict.get("updates")[0].get("values")[0].get("value").update(
        roll=roll, pitch=pitch
    )
    heading = filters.removeNegRad(heading)
    msgDict.get("updates")[0].get("values")[1].update(value=heading)
    headingTrue = filters.removeNegRad(heading + declination)
    msgDict.get("updates")[0].get("values")[2].update(value=headingTrue)
    txt = json.dumps(msgDict) + "\n"
    return txt


def icmFilterPrint(icm, sFilter, heading, pitch, roll, a, g, m):
    sfRoll, sfPitch, sfHeading = sFilter.attitude()
    sfRoll, sfPitch, sfHeading = icm.level(sfRoll, sfPitch, sfHeading)
    print("Acceleration raw: X: %.2f, Y: %2f, Z: %2f m/s^2" % (a))
    print("Gyro raw X: %2f, Y: %2f, Z: %2f rads/s" % (g))
    print(
        "Magnometer raw X: {:.2f}, Y: {:.2f}, Z: {:.2f} ut".format(
            m[0], m[1], m[2]
        )
    )
    print()
    print(
        "Mag Acc heading: {:.2f} pitch: {:.2f} Roll: {:.2f}".format(
            filters.radToDegrees(filters.removeNegRad(sfHeading)),
            filters.radToDegrees(sfPitch),
            filters.radToDegrees(sfRoll),
        )
    )
    print(
        "MainFilter heading: {:.2f} pitch: {:.2f} Roll: {:.2f}".format(
            filters.radToDegrees(filters.removeNegRad(heading)),
            filters.radToDegrees(pitch),
            filters.radToDegrees(roll),
        )
    )
    print()


def validateSkData(height, lon, lat):
    errMsg = "NaN values in signalk data {}"
    errDataTxt = []
    if isNaN(height):
        errDataTxt.append("Height,")
    if isNaN(lon):
        errDataTxt.append("Longitude,")
    if isNaN(lat):
        errDataTxt.append("Latitude")
    if len(errDataTxt) > 0:
        raise ValueError(errMsg.format(errDataTxt))


def isNaN(n):
    return n != n


if __name__ == "__main__":
    stoplx = False
    des = "Get sensor data from arduino and send it to a signalk server"
    parser = argparse.ArgumentParser(
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
        description=des,
    )
    parser.add_argument(
        "-p", "--port", type=int, help="The signal k port", default="10129"
    )
    parser.add_argument(
        "-ip", "--ip", help="The signal k ip addresss", default="127.0.0.1"
    )
    parser.add_argument(
        "-ar", "--arduino", help="The arduino path", default="/dev/ttyACM0"
    )
    parser.add_argument("-v", "--verbuse", action="store_true")
    parser.add_argument(
        "-dt",
        "--dt",
        type=int,
        help="Time between sensor reads in milli seconds",
        default=200,
    )
    parser.add_argument(
        "-fi",
        "--ficm",
        type=int,
        help="Frequens for signal k updates for pitch,roll and heading",
        default=4,
    )
    parser.add_argument(
        "-fb",
        "--fbme",
        type=float,
        help="Frequens for signal k updates for temp, pressure, humidity",
        default=50,
    )
    parser.add_argument(
        "-cf",
        "--configfile",
        help="The configuration file",
        default="espServerConfig.json",
    )
    args = parser.parse_args()
    serve(args.arduino,
          (args.ip, args.port),
          args.dt,
          args.ficm,
          args.fbme,
          args.configfile,
          args.verbuse)
