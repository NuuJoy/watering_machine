#!/usr/bin/env python

import sys
import json
import time
import numpy
import scipy.optimize
# --------------------------------------------------------------------------------------------------------------
class servoPoint():
    def __init__(self,nodeName,initPos,initAxis,initDeg,degMin,degMax,child=[]):
        self.nodeName = nodeName
        self.curPos   = initPos
        self.curAxs   = initAxis
        self.curDeg   = initDeg
        self.degMin   = degMin
        self.degMax   = degMax
        self.child    = child
    def action(self,degree):
        if degree < self.degMin:
            degree = self.degMin
        elif degree > self.degMax:
            degree = self.degMax
        theta     = (degree-self.curDeg)/180*numpy.pi
        rotateMat = numpy.array([[numpy.cos(theta)+(self.curAxs[0]**2)*(1-numpy.cos(theta)),
                                    self.curAxs[0]*self.curAxs[1]*(1-numpy.cos(theta))-self.curAxs[2]*numpy.sin(theta),
                                    self.curAxs[0]*self.curAxs[2]*(1-numpy.cos(theta))+self.curAxs[1]*numpy.sin(theta)],
                                    [self.curAxs[0]*self.curAxs[1]*(1-numpy.cos(theta))+self.curAxs[2]*numpy.sin(theta),
                                    numpy.cos(theta)+(self.curAxs[1]**2)*(1-numpy.cos(theta)),
                                    self.curAxs[1]*self.curAxs[2]*(1-numpy.cos(theta))-self.curAxs[0]*numpy.sin(theta)],
                                    [self.curAxs[0]*self.curAxs[2]*(1-numpy.cos(theta))-self.curAxs[1]*numpy.sin(theta),
                                    self.curAxs[1]*self.curAxs[2]*(1-numpy.cos(theta))+self.curAxs[0]*numpy.sin(theta),
                                    numpy.cos(theta)+(self.curAxs[2]**2)*(1-numpy.cos(theta))]])
        for eachChild in self.child:
            deOffsetPos      = [val1-val0 for val1,val0 in zip (eachChild.curPos,self.curPos)]
            newDeOffsetPos   = numpy.matmul(rotateMat,deOffsetPos)
            eachChild.curPos = [val1+val0 for val1,val0 in zip (newDeOffsetPos,self.curPos)]
            eachChild.curAxs = numpy.matmul(rotateMat,eachChild.curAxs)
        self.curDeg = degree
# --------------------------------------------------------------------------------------------------------------
class servoCalculator():
    def __init__(self,actuators,targetList,maxStepSize,yoffset):
        self.actuators  = actuators
        self.targetList = [[[None,None,None],[None,None,None],[None,None,None],[x,y,z+yoffset],[x,y,z]] for x,y,z in targetList]
        
        self.newTargetList = []
        for target0,target1 in zip(self.targetList[:-1],self.targetList[1:]):
            norm = numpy.linalg.norm([pos1-pos0 for pos0,pos1 in zip(target0[-1],target1[-1])])
            step = numpy.ceil(norm/maxStepSize)
            for x3,y3,z3,x4,y4,z4 in zip(numpy.ndarray.tolist(numpy.linspace(target0[-2][0],target1[-2][0],step)),
                                         numpy.ndarray.tolist(numpy.linspace(target0[-2][1],target1[-2][1],step)),
                                         numpy.ndarray.tolist(numpy.linspace(target0[-2][2],target1[-2][2],step)),
                                         numpy.ndarray.tolist(numpy.linspace(target0[-1][0],target1[-1][0],step)),
                                         numpy.ndarray.tolist(numpy.linspace(target0[-1][1],target1[-1][1],step)),
                                         numpy.ndarray.tolist(numpy.linspace(target0[-1][2],target1[-1][2],step))):
                self.newTargetList.append(target0[:-2]+[[x3,y3,z3]]+[[x4,y4,z4]])

        print('calculating actuator-state for all target-timeline ...')
        startTime = time.time()
        self.actStateList = [self.actuatorStateCalculation(target) for target in self.newTargetList]
        print('... calculation DONE ({:0.2f} sec)'.format(time.time()-startTime))

    def actuatorStateCalculation(self,target):
        # save original state
        orgnDeg = {}
        for act in self.actuators:
            orgnDeg[act.nodeName] = act.curDeg
        # define optimize-function
        def responseFunc(inputVal,actuators,target):
            # apply new state
            for act,val in zip(actuators,inputVal):
                act.action(val)
            # get overall position error
            posError = []
            for act,tar in  zip(actuators,target):
                for axsAct,axsTar in zip(act.curPos,tar):
                    if axsTar != None:
                        posError.append(abs(axsAct-axsTar))
            return numpy.linalg.norm(posError)
        def con(inputVal,actuators):
            excdDeg = 0.0
            for act,val in zip(actuators,inputVal):
                if val < act.degMin:
                    excdDeg += abs(val-act.degMin)
                elif val > act.degMax:
                    excdDeg += abs(val-act.degMax)
            return -excdDeg
        # Nelder-Mead, Powell ----------- unconstraints 
        # COBYLA, SLSQP, trust-constr --- constraints
        res = scipy.optimize.minimize(fun=responseFunc,
                                       x0=[act.curDeg for act in self.actuators][:-1], 
                                     args=(self.actuators,target), 
                                   method='trust-constr',
                              constraints={'type': 'ineq','fun': con, 'args': (self.actuators,)},
                                      tol=1e-8)
        # restore original state
        for act in self.actuators:
            act.action(orgnDeg[act.nodeName])
        return numpy.ndarray.tolist(res.x)

point4 = servoPoint('point4',[ 0 , 40, 50],[1,1,1],0,-90,90)
point3 = servoPoint('point3',[ 0 , 40, 60],[1,0,0],0,-90,90,child=[point4])
point2 = servoPoint('point2',[ 0 ,-10, 60],[1,0,0],0,-90,90,child=[point3,point4])
point1 = servoPoint('point1',[ 0 ,-10, 10],[1,0,0],0,-90,90,child=[point2,point3,point4])
point0 = servoPoint('point0',[ 0 ,-10,  0],[0,0,1],0,-90,90,child=[point1,point2,point3,point4])

targetList = [[  0,  0,  0],
              [-60,  0,  0],
              [-60, 60,  0],
              [-60, 60, 50],
              [-30, 60, 50],
              [-30, 60,  0],
              [  0,100,  0],
              [ 30, 30, 30],
              [ 60, 30, 60],
              [ 60,  0, 30],
              [ 30,  0, 30],
              [  0,  0,  0]]

myServo = servoCalculator((point0,point1,point2,point3,point4),targetList,1.0,10)

for row in myServo.actStateList:
    print(row)

with open('/home/nuujoy/Desktop/workingDirectory/wateringArm/wateringArmCmdList.json','w') as outputFile:
    outputFile.write(json.dumps(myServo.actStateList))
