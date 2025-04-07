import time
import sys

import sensors as sen


def main(serial, wifi, pw):
    path = '/dev/ttyACM0'
    if serial:
        con = sen.SerialEspOnDemand(path,
                                    sen.esp32DataSize(),
                                    sen.esp32HavsmolfData)
        time.sleep(1)
        print("Serial on demand (esp server)")
        for i in range(4):
            data = con.readSensorData()
            print(data)
            time.sleep(0.1)

        con.stop()
        time.sleep(3)  # wait for restart
        print("Serial loop (esp client)")
        con = sen.SerialEsp(path,
                            sen.esp32DataSize(),
                            sen.esp32HavsmolfData,
                            sen.DEFAULT_BAUDRATE,
                            dataSpeed=100
                            )
        _ = con.readSensorData()  # wait for my setup
        for i in range(10):
            data = con.readSensorData()
            print(data)

        con.stop()

        time.sleep(3)  # wait for restart
    if wifi:
        print("UPD loop (esp client)")
        con = sen.UdpClientEsp(path,
                               sen.esp32DataSize(),
                               pw,
                               ssid="RHO_2.4GHz",
                               dataFunc=sen.esp32HavsmolfData,
                               dataSpeed=100,
                               interface='wlp2s0'
                               )

        _ = con.readSensorData()  # wait for my setup
        for i in range(10):
            data = con.readSensorData()
            print(data)
        con.stop()

        time.sleep(5)  # wait for restart

        con = sen.UdpServerEsp(path,
                               sen.esp32DataSize(),
                               pw,
                               ssid="RHO_2.4GHz",
                               dataFunc=sen.esp32HavsmolfData,
                               interface='wlp2s0'
                               )
        print("Udp on demand (esp server)")
        for i in range(4):
            data = con.readSensorData()
            print(data)
            time.sleep(0.12)
        con.stop()


if __name__ == "__main__":
    pw = sys.argv[1]
    main(True, False, pw)
