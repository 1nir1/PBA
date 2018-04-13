import sys
sys.path.append('../src')
import pba

if __name__=="__main__":
    print ('Start')
    # Create RobotModule object from text file
    w = pba.Utils.parseTxtToPBA('txtToParseFormat.txt')
    # Start run the pybullet env
    w.startRun('exampleParseFormat')
    # Show the initial state
    for _ in range(200):
        w.makeStep()
    
    # Move all ankles
    joints = [ 'ankle_1', 'ankle_2', 'ankle_3', 'ankle_4']
    for _ in range(100):
        for j in joints:
            w.moveJoint(j + "_joint", -50)
        w.makeStep() # Camera will be at a fixed position

    # Move all hips
    joints = ['hip_1', 'hip_2', 'hip_3', 'hip_4']
    for i in range(400):
        for j in joints:
            w.moveJoint(j + "_joint", 60)
        w.makeStep(5, i, -20) # Camera will rotate around the robot

    w.stopRun()
    print ("Finish")