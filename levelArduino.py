import json
import argparse
import os
import numpy as np
import time

import sensors as sen


def level(loopno, con):
    rows = loopno
    cols = 6
    buf = np.zeros((rows, cols))
    for i in range(rows):
        data = con.readSensorData()
        row = np.array(data["acc"] + data["gyro"])
        buf[i] = row
        time.sleep(0.100)

    avgv = np.average(buf, axis=0)
    return (tuple(avgv[:3]), tuple(avgv[3:]))


def main(fileName, sampleSize, arPath):
    print("Collecting samles no: {}".format(sampleSize))
    field = "level"
    con = sen.SerialEspOnDemand(arPath, sen.esp32DataSize(),
                                sen.esp32HavsmolfData)
    time.sleep(1)

    acc, gyro = level(sampleSize, con)
    con.stop()

    dict = {"acc": acc, "gyro": gyro}
    levelDict = {field: dict}
    print(levelDict)

    if not os.path.isfile(fileName):
        print("New file created: {}".format(fileName))
        with open(fileName, "w") as f:
            f.write(json.dumps(levelDict))
    else:
        print("Existing file is updated: {}".format(fileName))
        with open(fileName, "r") as f:
            dict = json.load(f)
        dict.update(levelDict)
        with open(fileName, "w") as f:
            f.write(json.dumps(dict))


if __name__ == "__main__":
    des = "Record the average accelaration and gyro sensor data. Save the result in a json file. Update the file if it exist. The average of the gyro data can be used to calibrate the gyro sensor and the average of acc can be used to adjust the pitch and roll if the sensor is not placed level."
    parser = argparse.ArgumentParser(
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
        description=des,
    )
    parser.add_argument(
        "-s",
        "--samplesize",
        help="The samle size",
        type=int,
        default=500,
    )
    parser.add_argument(
        "-f",
        "--file",
        help="The data is save in this json file. If the file exist the dict keys acc and gyro is updated else a new dict is created with these to keys.",
        default="espServerConfig.json",
    )
    parser.add_argument(
        "-ar", "--arduino", help="The arduino path", default="/dev/ttyACM0"
    )
    args = parser.parse_args()
    main(args.file, args.samplesize, args.arduino)
