import sys
import json
import time
import numpy
import scipy.optimize

from _servoPoint import servoPoint

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
                                   method='COBYLA',
                              constraints={'type': 'ineq','fun': con, 'args': (self.actuators,)},
                                      tol=1e-8)
        # restore original state
        for act in self.actuators:
            act.action(orgnDeg[act.nodeName])
        return numpy.ndarray.tolist(res.x)

# build-up geometry
initNode = [['point4',[ 0 , 40, 50],[1,1,1],0,-90,90,[]],
            ['point3',[ 0 , 40, 60],[1,0,0],0,-90,90,['point4']],
            ['point2',[ 0 ,-10, 60],[1,0,0],0,-90,90,['point3','point4']],
            ['point1',[ 0 ,-10, 10],[1,0,0],0,-90,90,['point2','point3','point4']],
            ['point0',[ 0 ,-10,  0],[0,0,1],0,-90,90,['point1','point2','point3','point4']]]
buildNode = {}
inputNode = []
for eachInitNode in initNode:
    buildNode[eachInitNode[0]] = servoPoint(eachInitNode[0],eachInitNode[1],eachInitNode[2],eachInitNode[3],eachInitNode[4],eachInitNode[5],[buildNode[eachName] for eachName in eachInitNode[6]])
    inputNode.append(buildNode[eachInitNode[0]])
inputNode.reverse()

targetList = [[  0, 40, 40],
              [  0,  0,  0],
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
              [  0, 40, 40]]

myServo = servoCalculator(inputNode,targetList,1.0,10)

with open('wateringArmCmdList.json','w') as outputFile:
    outputFile.write(json.dumps([initNode,myServo.actStateList]))
