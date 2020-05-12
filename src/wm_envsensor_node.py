#!/usr/bin/env python

"""
BH1750 measurement mode:
           |hex_input |measuring    |precision  |range                 |timeused  |
    Mode 0 |      0x20|     one-time|    4.0 lux|   0.0 to 54612.50 lux|     16 ms|
    Mode 1 |      0x21|     one-time|    1.0 lux|   0.0 to 54612.50 lux|    120 ms|
    Mode 2 |      0x23|     one-time|    0.5 lux|   0.0 to 27306.25 lux|    120 ms|
    Mode 3 |      0x13|   continuous|    4.0 lux|   0.0 to 54612.50 lux|     16 ms|
    Mode 4 |      0x10|   continuous|    1.0 lux|   0.0 to 54612.50 lux|    120 ms|
    Mode 5 |      0x11|   continuous|    0.5 lux|   0.0 to 27306.25 lux|    120 ms|
"""

import sys
import json
import time
import rospy
import std_msgs.msg
import Adafruit_DHT
import smbus

class sensor_DHT11():
    def __init__(self,gpio_pin):
        self.type     = 'DHT11'
        self.gpio_pin = gpio_pin
        self.update()
    def update(self):
        self.humidity,self.temperature = Adafruit_DHT.read_retry(Adafruit_DHT.DHT11,self.gpio_pin)

class sensor_BH1750():
    def __init__(self,mode=0,alt_addr=False):
        self.type = 'BH1750'
        self.bus  = smbus.SMBus(1)
        self.address = 0x5C if alt_addr else 0x23
        self.mode    = [0x20,0x21,0x23,0x13,0x10,0x11][mode]
        self.update()
    def update(self):
        rawdata = self.bus.read_i2c_block_data(self.address,self.mode)
        self.intensity = (rawdata[1]+(256*rawdata[0]))/1.2

if __name__ == '__main__':
    try:
        setupFileName = sys.argv[1] if len(sys.argv) > 1 else 'wm_setupFile.json'
        with open(setupFileName,'r') as setupFile:
            generalsetup,_,_,envsetup,_ = json.loads(setupFile.read())

        rospy.init_node(generalsetup['machineName']+'_envsensor_node', anonymous=True)

        sensorDict = {}
        for (senName, senType, senParam) in envsetup:
            if senType == 'DHT11':
                sensorDict[senName] = sensor_DHT11(*senParam)
            elif senType == 'BH1750':
                sensorDict[senName] = sensor_BH1750(*senParam)

        pub_temp = rospy.Publisher('/'+ generalsetup['machineName'] +'/envsensor/temperature', std_msgs.msg.Float64, queue_size=1)
        pub_humd = rospy.Publisher('/'+ generalsetup['machineName'] +'/envsensor/humidity', std_msgs.msg.Float64, queue_size=1)
        pub_inlx = rospy.Publisher('/'+ generalsetup['machineName'] +'/envsensor/ambientlight', std_msgs.msg.Float64, queue_size=1)
        
        while not rospy.is_shutdown():
            raw_temp = []
            raw_humd = []
            raw_inlx = []
            for eachSensor in sensorDict:
                eachSensor.update()
                if eachSensor.type == 'DHT11':
                    raw_humd.append(eachSensor.humidity)
                    raw_temp.append(eachSensor.temperature)
                elif eachSensor.type == 'BH1750':
                    raw_inlx.append(eachSensor.intensity)
            pub_inlx.publish(sum(raw_inlx)/len(raw_inlx))
            pub_temp.publish(sum(raw_temp)/len(raw_temp))
            pub_humd.publish(sum(raw_humd)/len(raw_humd))
            
            rospy.sleep(60)

    except rospy.ROSInterruptException:
        pass
