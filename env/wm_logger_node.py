#!/usr/bin/env python

import sys
import time
import json
import rospy
import std_msgs.msg

class message_publisher():
    def __init__(self,msgName,pub_msg):
        self.msgName = msgName
        self.pub_msg = pub_msg
    def logData(self,data):
        concatStr = time.strftime("%a, %d %b %Y %H:%M:%S", time.localtime()) + ', ' + self.msgName + ', ' + str(data.data)
        pub_msg.publish(concatStr)

class message_logger():
    def __init__(self,outputfile):
        self.outputfile   = outputfile
        self.squeezing    = False
        self.squeezeCount = 0
    def getandprint(self,inputStr):
        with open(self.outputfile,'a') as logfile:
            if '/setdeg' in inputStr.data:
                if self.squeezing == False:
                    print(inputStr.data+'... setdeg: started')
                    logfile.write(inputStr.data+'... setdeg: started'+'\n')
                else:
                    self.squeezeCount += 1
                self.squeezing = inputStr.data
            else:
                if self.squeezing != False:
                    print(self.squeezing+'... setdeg: ' + str(self.squeezeCount) + ' squeezed')
                    logfile.write(self.squeezing+'... setdeg: ' + str(self.squeezeCount) + ' squeezed'+'\n')
                    self.squeezing    = False
                    self.squeezeCount = 0
                print(inputStr.data)
                logfile.write(inputStr.data+'\n')

if __name__ == '__main__':
    try:
        setupFileName = sys.argv[1] if len(sys.argv) > 1 else 'wm_setupFile.json'
        with open(setupFileName,'r') as setupFile:
            generalsetup,_,pwmsetup,_,_ = json.loads(setupFile.read())

        rospy.init_node(generalsetup['machineName']+'_logger_node', anonymous=True)

        pub_msg = rospy.Publisher('/wm_logger/feed', std_msgs.msg.String, queue_size=100)

        allTopicList = rospy.get_published_topics()
        print('-------- allTopicList --------------------------------------------------------')
        print(allTopicList)
        print('-------------------------------------------------------- allTopicList --------')

        loggerDict = {}
        for (topicName,topicType) in allTopicList:
            loggerDict[topicName] = message_publisher(topicName,pub_msg)
            rospy.Subscriber(topicName, std_msgs.msg.Bool, loggerDict[topicName].logData, queue_size=1)

        logger = message_logger('logFile.csv')
        rospy.Subscriber('/wm_logger/feed', std_msgs.msg.String, logger.getandprint, queue_size=1)

        rospy.spin()
        
    except rospy.ROSInterruptException:
        pass
