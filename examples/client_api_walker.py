import sys
sys.path.append('../src')
import pba
import math
import time

'''
Example for building WALKER module using the Robot_Module API.
'''
def buildWalker():
    w = pba.Robot_Module()

    # Main body
    w.addBody("torso", "0 0 1.5")
    w.addGeom("torso_geom", "0 0 1.45 0 0 1.05", "0.12", "capsule", "torso")

    # Right leg
    w.addBody("tie", "0 0.2 0", "torso")
    w.addJoint("tie_joint", "1", "0 -1 0", "-150 0", "200")
    w.addGeom("tie_geom", "0 0 1.05 0 0 0.6", "0.05", "capsule")

    w.addBody("leg", "0 0 0")
    w.addJoint("leg_joint", "1", "0 -1 0", "-150 0", "200")
    w.addGeom("leg_geom", "0 0 0.6 0 0 0.1", "0.04", "capsule")
    
    w.addBody("foot", "0 0 0")
    w.addJoint("foot_joint", "1", "0 -1 0", "-45 45", "200")
    w.addGeom("foot_geom", "-0.13 0 0.1 0.26 0 0.1", "0.06", "capsule")
    
    # Left leg
    w.addBody("tie_left", "0 -0.2 0", "torso")
    w.addJoint("tie_joint_left", "1", "0 -1 0", "-150 0",  "200")
    w.addGeom("tie_geom_left", "0 0 1.05 0 0 0.6", "0.05", "capsule")

    w.addBody("leg_left", "0 0 0")
    w.addJoint("leg_joint_left", "1", "0 -1 0", "-150 0",  "200")
    w.addGeom("leg_geom_left", "0 0 0.6 0 0 0.1", "0.04", "capsule")
    
    w.addBody("foot_left", "0 0 0")
    w.addJoint("foot_joint_left", "1", "0 -1 0", "-45 45", "200")
    w.addGeom("foot_geom_left", "-0.13 0 0.1 0.26 0 0.1", "0.06", "capsule")

    return w
    
'''
Example of basic WALKER module movement given a built Robot_Module 'w'
'''
def walkOurWalker(w):
    w.startRun('our_walker', visual=False) # visual = False => only console.
    for _ in range(1000):
        half = 40
        deg = 30
        if (w.getFrame() % half < half * 0.5):
            # Move right foot
            w.moveJoint("tie_joint", deg, 1)
            w.moveJoint("foot_joint", deg , 1)
            w.moveJoint("leg_joint", deg / 4, 1)

            w.moveJoint("tie_joint_left",0, 1)
            w.moveJoint("foot_joint_left", 0, 1)
            w.moveJoint("leg_joint_left",0, 1)
        else:
            # Move left foot
            w.moveJoint("foot_joint", 0, 1)
            w.moveJoint("leg_joint", 0, 1)
            w.moveJoint("tie_joint", 0, 1)
            
            w.moveJoint("tie_joint_left", deg, 1)
            w.moveJoint("foot_joint_left",deg, 1)
            w.moveJoint("leg_joint_left", deg / 4, 1)

        # makeStep returns a list of joint that didn't finish the requested move yet - when the list is empty - finish the run.
        while len(w.makeStep()) != 0:
            continue

        pos, ori = w.getPositionAndOrientation()
        print (str(pos))

    w.stopRun()

if __name__=="__main__":
    walkOurWalker(buildWalker())