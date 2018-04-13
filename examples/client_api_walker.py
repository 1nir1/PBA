import sys
sys.path.append('../src')
import pba
import math
import time

def buildWalker():
    w = pba.Robot_Module()
    w.addBody("torso", "0 0 1.5")
    w.addGeom("torso_geom", "0 0 1.45 0 0 1.05", "0.12", "capsule", "torso")

    w.addBody("tie", "0 0.2 0", "torso")
    w.addJoint("tie_joint", "1", "0 -1 0", "-150 0", "200", "tie")
    w.addGeom("tie_geom", "0 0 1.05 0 0 0.6", "0.05", "capsule", "tie")

    w.addBody("leg", "0 0 0")
    w.addJoint("leg_joint", "1", "0 -1 0", "-150 0", "200", "leg")
    w.addGeom("leg_geom", "0 0 0.6 0 0 0.1", "0.04", "capsule", "leg")
    
    w.addBody("foot", "0 0 0", "leg")
    w.addJoint("foot_joint", "1", "0 -1 0", "-45 45", "200", "foot")
    w.addGeom("foot_geom", "-0.13 0 0.1 0.26 0 0.1", "0.06", "capsule", "foot")
    
    w.addBody("tie_left", "0 -0.2 0", "torso")
    w.addJoint("tie_joint_left", "1", "0 -1 0", "-150 0",  "200", "tie_left")
    w.addGeom("tie_geom_left", "0 0 1.05 0 0 0.6", "0.05", "capsule", "tie_left")

    w.addBody("leg_left", "0 0 0", "tie_left")
    w.addJoint("leg_joint_left", "1", "0 -1 0", "-150 0",  "200", "leg_left")
    w.addGeom("leg_geom_left", "0 0 0.6 0 0 0.1", "0.04", "capsule", "leg_left")
    
    w.addBody("foot_left", "0 0 0", "leg_left")
    w.addJoint("foot_joint_left", "1", "0 -1 0", "-45 45", "200", "foot_left")
    w.addGeom("foot_geom_left", "-0.13 0 0.1 0.26 0 0.1", "0.06", "capsule", "foot_left")

    return w
def walkOurWalker(w):
    w = buildWalker()
    w.startRun('our_hopper', visual=False)
    flag = True
    while (1):
        half = 40
        deg = 30
        if (w.getFrame() % half < half * 0.5):
            if flag:
                w.moveJoint("foot_joint", deg , 1)
                w.moveJoint("leg_joint", deg / 4, 1)
                w.moveJoint("foot_joint_left", 0, 1)
                w.moveJoint("leg_joint_left",0, 1)
                w.moveJoint("tie_joint_left",0, 1)
                w.moveJoint("tie_joint", deg, 1)
        else:
            flag = False
            w.moveJoint("foot_joint", deg * 3, 1)
            w.moveJoint("leg_joint", 0, 1)
            w.moveJoint("tie_joint", 0, 1)
            w.moveJoint("foot_joint_left",0, 1)
            w.moveJoint("leg_joint_left", 0, 1)
            w.moveJoint("tie_joint_left", 0, 1)
        #for _ in range(100):
        w.makeStep()
        pos, ori = w.getPositionAndOrientation()
        print (str(pos))

if __name__=="__main__":
    walkOurWalker(buildWalker())