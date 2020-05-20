#!/usr/bin/env python

"""
BH1750 measurement mode:
           |hex_input |measuring    |precision  |range                 |timeused  |
    Mode 0 |      0x23|     one-time|    4.0 lux|   0.0 to 54612.50 lux|     16 ms|
    Mode 1 |      0x21|     one-time|    1.0 lux|   0.0 to 54612.50 lux|    120 ms|
    Mode 2 |      0x20|     one-time|    0.5 lux|   0.0 to 27306.25 lux|    120 ms|
    Mode 3 |      0x13|   continuous|    4.0 lux|   0.0 to 54612.50 lux|     16 ms|
    Mode 4 |      0x11|   continuous|    1.0 lux|   0.0 to 54612.50 lux|    120 ms|
    Mode 5 |      0x10|   continuous|    0.5 lux|   0.0 to 27306.25 lux|    120 ms|
"""

import sys
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
        return self.humidity,self.temperature
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
        return self.intensity

if __name__ == '__main__':
    try:
        rospy.init_node('wm_ros_envsensor_node', anonymous=True)

        dht11_1  = sensor_DHT11(20)
        dht11_2  = sensor_DHT11(21)
        bh1750_1 = sensor_BH1750(1,False)
        bh1750_2 = sensor_BH1750(1,True)
        
        pub_temp1 = rospy.Publisher('/wm_ros/envsensor/temp1', std_msgs.msg.Float64, queue_size=1)
        pub_temp2 = rospy.Publisher('/wm_ros/envsensor/temp2', std_msgs.msg.Float64, queue_size=1)
        pub_humd1 = rospy.Publisher('/wm_ros/envsensor/humd1', std_msgs.msg.Float64, queue_size=1)
        pub_humd2 = rospy.Publisher('/wm_ros/envsensor/humd2', std_msgs.msg.Float64, queue_size=1)
        pub_inlx1 = rospy.Publisher('/wm_ros/envsensor/inlx1', std_msgs.msg.Float64, queue_size=1)
        pub_inlx2 = rospy.Publisher('/wm_ros/envsensor/inlx2', std_msgs.msg.Float64, queue_size=1)
        
        while not rospy.is_shutdown():
            try:
                # update and publish dht11_1 sensor value
                humd1,temp1 = dht11_1.update()
                pub_temp1.publish(temp1)
                pub_humd1.publish(humd1)
                # update and publish bh1750_1 sensor value
                inlx1 = round(bh1750_1.update(),1)
                pub_inlx1.publish(inlx1)
                # update and publish dht11_2 sensor value
                humd2,temp2 = dht11_2.update()
                pub_temp2.publish(temp2)
                pub_humd2.publish(humd2)
                # update and publish bh1750_2 sensor value
                inlx2 = round(bh1750_2.update(),1)
                pub_inlx2.publish(inlx2)

                currentDateTime = time.strftime("%b %d %Y %H:%M:%S", time.localtime())
                print('{} | temp: {}, {} | humd1: {}, {} | inlx1: {}, {}'.format(currentDateTime,temp1,temp2,humd1,humd2,inlx1,inlx2))
            except:
                pass
            
            rospy.sleep(60)

    except rospy.ROSInterruptException:
        pass
 
