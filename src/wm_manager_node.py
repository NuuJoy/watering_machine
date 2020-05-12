#!/usr/bin/env python

import sys
import json
import rospy
import std_msgs.msg

class envDataStore():
    def __init__(self):
        self.curTemperature  = None
        self.curHumidity     = None
        self.curAmbientLight = None
    def temperature_update(self,msgs):
        self.curTemperature = msgs.data
    def humidity_update(self,msgs):
        self.curHumidity    = msgs.data
    def ambientlight_update(self,msgs):
        self.curLightLux    = msgs.data
    
if __name__ == '__main__':
    try:
        setupFileName = sys.argv[1] if len(sys.argv) > 1 else 'wm_setupFile.json'
        with open(setupFileName,'r') as setupFile:
            generalsetup,_,_,_,managersetup = json.loads(setupFile.read())

        env = envDataStore()
        rospy.Subscriber('/'+ generalsetup['machineName'] +'/envsensor/temperature', std_msgs.msg.Float64, env.temperature_update, queue_size=1)
        rospy.Subscriber('/'+ generalsetup['machineName'] +'/envsensor/humidity', std_msgs.msg.Float64, env.humidity_update, queue_size=1)
        rospy.Subscriber('/'+ generalsetup['machineName'] +'/envsensor/ambientlight', std_msgs.msg.Float64, env.ambientlight_update, queue_size=1)

        pub_cmd = rospy.Publisher('/'+ generalsetup['machineName'] +'/overseer/inputfile', std_msgs.msg.String, queue_size=100)
        
        while not rospy.is_shutdown():
            
            rospy.sleep(60)

    except rospy.ROSInterruptException:
        pass
