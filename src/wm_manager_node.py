#!/usr/bin/env python

import sys
import time
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
            generalsetup,_,_,_,managertask = json.loads(setupFile.read())

        env = envDataStore()
        rospy.Subscriber('/'+ generalsetup['machineName'] +'/envsensor/temperature', std_msgs.msg.Float64, env.temperature_update, queue_size=1)
        rospy.Subscriber('/'+ generalsetup['machineName'] +'/envsensor/humidity', std_msgs.msg.Float64, env.humidity_update, queue_size=1)
        rospy.Subscriber('/'+ generalsetup['machineName'] +'/envsensor/ambientlight', std_msgs.msg.Float64, env.ambientlight_update, queue_size=1)

        pub_cmd = rospy.Publisher('/'+ generalsetup['machineName'] +'/overseer/inputfile', std_msgs.msg.String, queue_size=100)
        
        curDay    = None
        todayTask = None
        while not rospy.is_shutdown():
            # check daily task
            dateTime   = time.localtime()
            curWeekday = ['Mon','Tue','Wed','Thu','Fri','Sat','Sun'][dateTime.tm_wday]
            curHour    = dateTime.tm_hour
            curMinute  = dateTime.tm_min

            if curDay != dateTime.tm_mday:
                # refresh today task
                todayTask = [task if task[0] == curWeekday else None for task in managertask]
                todayTask.remove(None)
                curDay = dateTime.tm_mday

            for task in todayTask:
                if 60*task[1]+task[2] >= 60*curHour+curMinute:
                    pub_cmd.publish(task[3])
                    rospy.sleep(task[4])
                    break
            
            # check on-demand task
            if env.curHumidity < 60.0:
                pub_cmd.publish('DoSomethingWithHumidity.json')
                rospy.sleep(150)
            elif env.curTemperature > 35.0:
                pub_cmd.publish('DoSomethingWithTemperature.json')
                rospy.sleep(10)
            elif env.curAmbientLight < 3000.0:
                pub_cmd.publish('DoSomethingWithAmbientLight.json')

            rospy.sleep(60)

    except rospy.ROSInterruptException:
        pass
