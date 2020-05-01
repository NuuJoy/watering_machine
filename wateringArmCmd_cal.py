import sys
import json
import time
import numpy
import scipy.optimize

from _servoPoint import servoPoint

# --------------------------------------------------------------------------------------------------------------
class servoCalculator():
    def __init__(self,actuators,targetList,maxStepSize):
        self.actuators = actuators
        self.lastAct   = [act.curDeg for act in self.actuators][:-1]
        print('calculating actuator-state for all target-timeline ...')
        startTime = time.time()
        self.actStateList = [self.actuatorStateCalculation(target) for target in targetList]
        print('... calculation DONE ({:0.2f} sec, {:0.2f} pos/sec)'.format(time.time()-startTime,len(targetList)/(time.time()-startTime)))

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
                                       x0=self.lastAct, 
                                     args=(self.actuators,target), 
                                   method='COBYLA',
                              constraints={'type': 'ineq','fun': con, 'args': (self.actuators,)},
                                      tol=1e-8)
        # restore original state
        for act in self.actuators:
            act.action(orgnDeg[act.nodeName])
        
        self.lastAct = numpy.ndarray.tolist(res.x)
        return numpy.ndarray.tolist(res.x)

# build-up geometry
initNode = [['point4',[ 20,-20,  0],[1,1,1],  0,-90,90,[]],
            ['point3',[-30,-20,  0],[0,1,0],-90,-90,90,['point4']],
            ['point2',[150,-20,  0],[0,1,0],-90,-90,90,['point3','point4']],
            ['point1',[-50,-20,  0],[0,1,0], 90,-90,90,['point2','point3','point4']],
            ['point0',[-50,-20,-20],[0,0,1], 90,-90,90,['point1','point2','point3','point4']]]
buildNode = {}
inputNode = []
for eachInitNode in initNode:
    buildNode[eachInitNode[0]] = servoPoint(eachInitNode[0],eachInitNode[1],eachInitNode[2],eachInitNode[3],eachInitNode[4],eachInitNode[5],[buildNode[eachName] for eachName in eachInitNode[6]])
    inputNode.append(buildNode[eachInitNode[0]])
inputNode.reverse()

sleep2wake = [[[ -50, -20, -20],[ -50, -20,   0],[None,None,None],[ -30, -20,   0],[  20, -20,   0]], # sleep,
              [[ -50, -20, -20],[ -50, -20,   0],[None,None,None],[   0, -20, 100],[   0, -20,  50]],
              [[ -50, -20, -20],[ -50, -20,   0],[None,None,None],[ -50,   0,  50],[ -50,   0,  00]]] # ready

""" 
worktarget = [[[ -50, -20, -20],[ -50, -20,   0],[None,None,None],[ -50,   0,  50],[ -50,   0,   0]]]
for z in range(-60,221,20):
    for y in range(-20,221,20):
        for x in range(-220,221,20):
            worktarget.append([[-50,-20,-20],[-50,-20,0],[None,None,None],[x,y,z+50],[x,y,z]])
"""

worktarget = [[[ -50, -20, -20],[ -50, -20,   0],[None,None,None],[ -50,   0,  50],[ -50,   0,   0]],
              [[ -50, -20, -20],[ -50, -20,   0],[None,None,None],[ 200,   0,  50],[ 200,   0,   0]],
              [[ -50, -20, -20],[ -50, -20,   0],[None,None,None],[ 200, 100,  50],[ 200, 100,   0]],
              [[ -50, -20, -20],[ -50, -20,   0],[None,None,None],[-200, 100,  50],[-200, 100,   0]],
              [[ -50, -20, -20],[ -50, -20,   0],[None,None,None],[-200, 200,  50],[-200, 200,   0]],
              [[ -50, -20, -20],[ -50, -20,   0],[None,None,None],[ 200, 200,  50],[ 200, 200,   0]],
              [[ -50, -20, -20],[ -50, -20,   0],[None,None,None],[ 200, 200, 200],[ 200, 200, 150]],
              [[ -50, -20, -20],[ -50, -20,   0],[None,None,None],[-200, 100, 200],[-200, 100, 150]],
              [[ -50, -20, -20],[ -50, -20,   0],[None,None,None],[   0,   0, 100],[   0,   0,  50]],
              [[ -50, -20, -20],[ -50, -20,   0],[None,None,None],[   0,   0,  50],[   0,   0,   0]]]

wake2sleep = list(sleep2wake)
wake2sleep.reverse()

targetList = sleep2wake + worktarget + wake2sleep

# ------------------------------------------------------------------------------------------

def refineTargetStep(targetList):
    maxStepSize = 10.0
    newTargetList = []
    for target0,target1 in zip(targetList[:-1],targetList[1:]):
        norm = numpy.linalg.norm([pos1-pos0 for pos0,pos1 in zip(target0[-1],target1[-1])])
        step = numpy.ceil(norm/maxStepSize)
        for x3,y3,z3,x4,y4,z4 in zip(numpy.ndarray.tolist(numpy.linspace(target0[-2][0],target1[-2][0],step)),
                                        numpy.ndarray.tolist(numpy.linspace(target0[-2][1],target1[-2][1],step)),
                                        numpy.ndarray.tolist(numpy.linspace(target0[-2][2],target1[-2][2],step)),
                                        numpy.ndarray.tolist(numpy.linspace(target0[-1][0],target1[-1][0],step)),
                                        numpy.ndarray.tolist(numpy.linspace(target0[-1][1],target1[-1][1],step)),
                                        numpy.ndarray.tolist(numpy.linspace(target0[-1][2],target1[-1][2],step))):
            newTargetList.append(target0[:-2]+[[x3,y3,z3]]+[[x4,y4,z4]])
    return newTargetList

sleep2wake_act = servoCalculator(inputNode,refineTargetStep(sleep2wake),5).actStateList

wake2sleep_act = list(sleep2wake_act)
wake2sleep_act.reverse()

worktarget_act = servoCalculator(inputNode,refineTargetStep(worktarget),5).actStateList

actList = sleep2wake_act + worktarget_act + wake2sleep_act

# ------------------------------------------------------------------------------------------

maxdeg    = 5.0
lastState = None
rfnActStateList = []
for eachState in actList:
    if lastState != None:
        maxdiff = max([abs(val1-val0) for val0,val1 in zip(lastState,eachState)])
        stpneed = numpy.ceil(maxdiff/maxdeg)
        for eachStep in numpy.linspace(lastState,eachState,stpneed):
            rfnActStateList.append(numpy.ndarray.tolist(eachStep))
    lastState = eachState

with open('wateringArmCmdList.json','w') as outputFile:
    outputFile.write(json.dumps([initNode,rfnActStateList,targetList]))
