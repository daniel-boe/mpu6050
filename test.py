from mpu6050 import mpu6050
from time import sleep
import sys
import datetime as dt
import json

path=f'RawData/GryoTest_{dt.datetime.now().strftime("%m%d%y_%H%M")}.json'
sensor=mpu6050(0x68)

while True:
    try:
        xyz=sensor.get_accel_data()
        angle=sensor.calculate_angle(xyz)
        xyz.update({'theta':angle,'timestamp':dt.datetime.now().timestamp()})
        with open(path,'a') as f:
            f.write(json.dumps(xyz))
            f.write('\n')
        print(xyz)
        sleep(0.1)
    except KeyboardInterrupt:
        print('Interrupted')
        break

sys.exit(0)