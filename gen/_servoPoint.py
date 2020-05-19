import numpy

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
