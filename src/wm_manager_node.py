#!/usr/bin/env python

import sys
import time
import json
import rospy
import std_msgs.msg

class envDataStore():
    def __init__(self):
        self.__temp = [None,None]
        self.__humd = [None,None]
        self.__inlx = [None,None]

    def data_update(self,msgs,argv):
        if isinstance(msgs.data, (float,int)):
            getattr(self,argv[0])[argv[1]] = msgs.data
        else:
            print('{} sensor {} not functional'.format(argv[0],argv[1]+1))

    @property
    def temp(self):
        if self.__temp.count(None) < len(self.__temp):
            temp = list(filter(None,self.__temp))
            return sum(temp)/len(temp)
    @property
    def humd(self):
        if self.__humd.count(None) < len(self.__humd):
            humd = list(filter(None,self.__humd))
            return sum(humd)/len(humd)
    @property
    def inlx(self):
        if self.__inlx.count(None) < len(self.__inlx):
            inlx = list(filter(None,self.__inlx))
            return sum(inlx)/len(inlx)

if __name__ == '__main__':
    try:
        rospy.init_node('wm_ros_manager_node', anonymous=True)

        env = envDataStore()
        rospy.Subscriber('/wm_ros/envsensor/temp1', std_msgs.msg.Float64, env.data_update, callback_args=('temp',0), queue_size=1)
        rospy.Subscriber('/wm_ros/envsensor/humd1', std_msgs.msg.Float64, env.data_update, callback_args=('humd',0), queue_size=1)
        rospy.Subscriber('/wm_ros/envsensor/inlx1', std_msgs.msg.Float64, env.data_update, callback_args=('inlx',0), queue_size=1)
        rospy.Subscriber('/wm_ros/envsensor/temp2', std_msgs.msg.Float64, env.data_update, callback_args=('temp',1), queue_size=1)
        rospy.Subscriber('/wm_ros/envsensor/humd2', std_msgs.msg.Float64, env.data_update, callback_args=('humd',1), queue_size=1)
        rospy.Subscriber('/wm_ros/envsensor/inlx2', std_msgs.msg.Float64, env.data_update, callback_args=('inlx',1), queue_size=1)

        pub_cmd = rospy.Publisher('/wm_ros/overseer/inputfile', std_msgs.msg.String, queue_size=100)
        
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
            if env.humd < 60.0:
                pub_cmd.publish('DoSomethingWithTooLowHumidity.json')
                rospy.sleep(150)
            elif env.humd > 80.0:
                pub_cmd.publish('DoSomethingWithTooHighHumidity.json')
                rospy.sleep(150)
            elif env.temp > 35.0:
                pub_cmd.publish('DoSomethingWithTemperature.json')
                rospy.sleep(10)
            elif env.inlx < 3000.0:
                pub_cmd.publish('DoSomethingWithTooDarkAmbientLight.json')
            elif env.inlx > 15000.0:
                pub_cmd.publish('DoSomethingWithTooBrightAmbientLight.json')

            rospy.sleep(60)

    except rospy.ROSInterruptException:
        pass
