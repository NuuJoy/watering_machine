#!/usr/bin/env python

print('env_monitor_node.py started')

import sys
import json
import time
import rospy
import std_msgs.msg
import Adafruit_DHT
import smbus

print('all module inported')

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
    print('entered main program')
    try:
        print('node initializing')
        # init ros node
        rospy.init_node('env_monitor_node', anonymous=True)
        # init each sensor handle
        dht11_1  = sensor_DHT11(20)
        dht11_2  = sensor_DHT11(21)
        bh1750_1 = sensor_BH1750(2,False)
        bh1750_2 = sensor_BH1750(2,True)
        # init publisher
        pub_temp1 = rospy.Publisher('env_monitor_node/temp1', std_msgs.msg.Float64, queue_size=1)
        pub_temp2 = rospy.Publisher('env_monitor_node/temp2', std_msgs.msg.Float64, queue_size=1)
        pub_humd1 = rospy.Publisher('env_monitor_node/humd1', std_msgs.msg.Float64, queue_size=1)
        pub_humd2 = rospy.Publisher('env_monitor_node/humd2', std_msgs.msg.Float64, queue_size=1)
        pub_inlx1 = rospy.Publisher('env_monitor_node/inlx1', std_msgs.msg.Float64, queue_size=1)
        pub_inlx2 = rospy.Publisher('env_monitor_node/inlx2', std_msgs.msg.Float64, queue_size=1)
        
        print('entering while-loop')
        while not rospy.is_shutdown():
            try:
                humd1,temp1 = dht11_1.update()
                if humd1:
                    pub_humd1.publish(humd1)
                if temp1:
                    pub_temp1.publish(temp1)
            except:
                pass
            
            try:
                humd2,temp2 = dht11_2.update()
                if humd2:
                    pub_humd2.publish(humd2)
                if temp2:
                    pub_temp2.publish(temp2)
            except:
                pass
            
            try:
                inlx1 = bh1750_1.update()
                if inlx1:
                    pub_inlx1.publish(inlx1)
            except:
                pass
            
            try:
                inlx2 = bh1750_2.update()
                if inlx2:
                    pub_inlx2.publish(inlx2)
            except:
                pass
            
            try:
                print('temp1:{}, temp2:{}, humd1:{}, humd2:{}, inlx1:{}, inlx2:{}'.format(temp1,temp2,humd1,humd2,inlx1,inlx2))
            except:
                pass

            rospy.sleep(60)

    except rospy.ROSInterruptException:
        pass

