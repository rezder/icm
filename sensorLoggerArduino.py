import argparse
import numpy as np
import time

import sensorLogger as logger
import sensors as sen


class arduinoSensor:
    def __init__(self, dots, con, sensor):
        n = con.readSensorData()[sensor]
        self.distance = np.linalg.norm(n)*5.65/dots
        self.con = con
        self.sensor = sensor

    def getData(self):
        return np.array(self.con.readSensorData()[self.sensor])

    def distFilter(self, data, buf, bufIx):
        add = True
        for i in range(bufIx):
            diff = data - buf[i]
            # print(np.linalg.norm(diff))

            if np.linalg.norm(diff) < self.distance:
                add = False
                break
        return add


def log(logFileName, sensor, dots, dt, bufSize, arPath):
    con = sen.SerialEspOnDemand(arPath, sen.esp32DataSize(),
                                sen.esp32HavsmolfData)
    time.sleep(1)
    ar = arduinoSensor(dots, con, sensor)
    logger.log(logFileName, ar.getData, ar.distFilter, dt, bufSize)
    con.stop()


if __name__ == "__main__":
    stop = False
    des = "Collect sensor data in csv file. Data can be either Magnetic, Gyro or Acceleration. The data is filtered with distance to the last collected readings only recording if they far enough away."
    parser = argparse.ArgumentParser(
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
        description=des,
    )
    parser.add_argument("-f", "--file", help="The csv data file", default="data.csv")
    parser.add_argument(
        "-s",
        "--sensor",
        help="The sensors mag: Magnetic, acc: Acceleration, gyro: Gyro ",
        default="mag",
        choices=["mag", "acc", "gyro"],
    )
    parser.add_argument(
        "-d",
        "--dots",
        help="The number of observation on a circle. The mag have a norm between 25 and 65. It is 45 at Petralia. The quarter arc is sqrt of 2 times norm. The old distance of 1 made room for about 4*sqrt(2)*45 about 280 dots on circle",
        default=360,
        type=float,
    )
    parser.add_argument(
        "-dt",
        "--sampletime",
        help="Minimum time in seconds between messurement",
        default=20,
        type=float,
    )
    parser.add_argument(
        "-b",
        "--buffer",
        help="The nummer records to hold in memory before save only records in memory is used to messure distance",
        default=200,
        type=int,
    )
    parser.add_argument(
        "-p",
        "--arduinopath",
        help="Arduino path",
        default="/dev/ttyACM0",
    )

    args = parser.parse_args()

    log(args.file,
        args.sensor,
        args.dots,
        args.sampletime,
        args.buffer,
        args.arduinopath)
