# Aruino source

The directory holds my python source code for the my boat that uses
arduino sensor.

## Files

### esp32Server.py 

The signal k udp server collects data from the arduino sensors
and send them to signal k server as UDP messages on port 10129.
The server uses a magnetic model of the world together with
location latitude, longitude and height from the signal k server.
The model remade every 5 years to match the worlds magnetic
field so the lib arhs needs to be updated.

###  espServerConfig.json

The servers configuration file it is data used for calibration and
adjustment of the sensors.

### levelArduino.py

Calculate avarage sensor data for acc and gyro. Gyro is used
for calibration of the gyro sensor and acc is used for adjustment of 
pitch and role if the sensor is not level.
I have not clibrated the acc sensor but it could be done in the same way
as the mag sensor.

### sensorLoggerArduino.py

Logs sensor data from the arduino at a interval and saves it in a file
this file use **sensorLogger.py** the independent part of sensor source.
default file is data.csv. Could be moved to pi and used there.

### sensors.py

Arduino connections to read sensor data.
Currently 4 types exist. Udp and serial and they comes in two variants
on demand or on a loop. On demand: you ask for data. On loop data
is send constanly from the arduino.

### testSensor.py

Test of Arduino connections.

There is also files from my pi python code used
directory /home/rho/Python/pi/havsmolf/

- utill.py used by esp32Server.py mostly a signal cache
- filters.py used by esp32Server.py metods to calculated pitch, role and headings
from icm data
- hardsoftmag.py calilbrate mag sensor data recorded by
the sensorLogger and used in the espServerConfig.json.


## Bootstrap server

To bootstrap the esp32Server:
Start the server to create the configuration file default is
espServerConfig.json. Then stop the server after a little time.

On a calm day run levelGyroAcc.py it updates espServerConfig.json with
level gyro and acc data if the file is in the executing directory else
it create a new file.

On a windy day run sensorLoggerArduino.py to collect
magnetic sensor data. Default is Magnetic data but change
the default time interval to 20 seconds (dt=20).
Then do some sailing. 20 seconds is my best guess.
Default is to save data in data.csv.

The hardsofmag.py updates icmserverconfig.json with the
magnetic bias calculated from the data.csv.
The default file is not the arduino server and must be changed.

Example where the data.csv and config file
is in the arduino directory.

```
python /home/rho/Python/pi/havsmolf/hardsoftmag.py -f=espServerConfig.json
```



