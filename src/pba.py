import sys
import numpy as np
import argparse
import pybullet as p
import time
import math
import os

'''
Default units:
    - Size is cm
    - Joint position is radians (API is in degrees)
    - Velocity is cm for second
'''

'''
Main class for pybullet API - contains all the interface between client side and pybullet.
'''
class Robot_Module:
    # internal parameter for indentation lines in the final XML file
    _spaces = 0
    # internal list for the motor objects in the module
    _motor_names = []
    # defualt joint type for all the joints
    _default_joint_type = "hinge"

    def __init__(self):
        self.WB = _WorldBody("WB")
        # list that contains all the names of the bodies in the robot module
        self.names = [] 
        # object id in the pybullet world
        self.robot = ""
        # Dictionary that map between joint name to joint index in the pybullet world 
        self.jdict = {}
        # Current frame. In each makeStep call, frame value will be increased by 1
        self.frame = 0
        # The start time after makeWorld step
        self.startTime = None
        # Save the start position of the robot
        self.startPosition = None
        # Parameter that indicates if the build step is finished
        self.finishBuild = False
        # Holds the last body that was added
        self.lastBody = self.WB
        # Dictionary that holds all the joints names that didn't reach their target angle
        self.jointSteps = {}
        # Dictionary that holds the joints range
        self.jRanges = {}
        # Dictionary that holds the joints max velocity
        self.jMaxVelocity = {}
        # Build state - may continue while there are no errors
        self.hasError = False

    '''
    Iterates over all the created objects (bodies, geoms and joints) and returns the object
    with name equals the input if exists - otherwise returns None.
    '''
    def _findByName(self, objs, name):
        if objs is None:
            return None

        for obj in objs:
            if obj.name == name:
                return obj
            ret_val = self._findByName(obj.children,name)
            if ret_val is not None:
                return ret_val

        return None
    '''
    Returns the current frame (frame = how many steps were made in the simulation)
    '''
    def getFrame(self):
        return self.frame
    '''
    Adds object of type 'body' to the xml
    '''
    def addBody(self, name, pos, parent=-1):
        if not Utils.dimensionAmount(pos, 3):
            print ("Position dimension shall be 3")
            self.hasError = True
            return False
        if self._verifyName(name):
            tmp = _Body(name,pos,parent)
            self._addComponent(tmp,parent)
            self.lastBody = tmp
            return True
        return False
    '''
    Adds object of type 'joint' to the xml
    '''
    def addJoint(self, name, max_velocity, axis, joint_range, gear, parent=-1):
        if not Utils.dimensionAmount(max_velocity, 1):
            print (name + "- Max velocity dimension shall be 1")
            self.hasError = True
        if not Utils.dimensionAmount(axis, 3):
            print (name + "- Axis dimension shall be 3")
            self.hasError = True
        if not Utils.dimensionAmount(joint_range, 2):
            print (name + "- Joint range shall be 2 dimension")
            self.hasError = True
        if not Utils.dimensionAmount(gear, 1):
            print (name + "- Joint gear shall be 1 dimension")
            self.hasError = True
        if axis == "0 0 0":
            return True
        
        if self.hasError:
            return False

        if self._verifyName(name):
            tmp = _Joint(name, max_velocity, axis, joint_range, self._default_joint_type, gear, parent)
            self._addComponent(tmp,parent)
            self.jRanges[name] = joint_range
            self.jMaxVelocity[name] = max_velocity
            return True
        return False
    '''
    Adds object of type 'geom' to the xml
    '''
    def addGeom(self, name, fromto, size, geom_type, parent=-1):
        geom_type = str(geom_type).lower()
        if not geom_type in Utils.geomTypes():
            print (name + "- Geom type is not valid. Look at Utils.geomTypes()")
            self.hasError = True
        if not Utils.dimensionAmount(fromto, Utils.geomTypeFromToDimension(geom_type)):
            print (name + "- Geom type require different fromto dimension. Look at Utils.geomTypeFromToDimension()")
            self.hasError = True
        if not Utils.dimensionAmount(size, Utils.geomTypeSizeDimension(geom_type)):
            print (name + "- Geom type require different size dimension. Look at Utils.geomTypeSizeDimension()")
            self.hasError = True
        if self.hasError:
            return False
        
        if self._verifyName(name):
            tmp = _Geom(name,fromto,size,geom_type,parent)
            self._addComponent(tmp,parent)
            return True
        return False
    '''
    Starts the pybullet physics run - creates the XML and loads it.
    '''
    def startRun(self, filePath = None, sdfPath = None, gravity = (0,0,-9.8), visual = True):
        if self.hasError:
            print ("Error occured during build - can't start with the existing Robot_Module")
            return
        if filePath is None:
            print ("Filepath is required")
            return
        if sdfPath is None:
            sdfPath = Utils.getDefaultSDF()
        if len(gravity) != 3:
            print ("Gravity must be 3 dimension")
            return
        
        self.finishBuild = True
        self.WB.prettyPrint(filePath)

        cid = p.connect(p.SHARED_MEMORY)
        if (cid<0):
            if visual is True:
                cid = p.connect(p.GUI) #DIRECT is much faster, but GUI shows the running gait
            else:
                cid = p.connect(p.DIRECT)

        p.setGravity(gravity[0], gravity[1], gravity[2])
        p.setPhysicsEngineParameter(fixedTimeStep=1.0/60., numSolverIterations=5, numSubSteps=2)

        p.loadSDF(sdfPath)
        objs = p.loadMJCF(filePath + ".xml" , flags = p.URDF_USE_SELF_COLLISION_EXCLUDE_ALL_PARENTS)

        self.robot  = objs[0]
        ordered_joints = []
        ordered_joint_indices = []

        parser = argparse.ArgumentParser()
        parser.add_argument('--profile')
        self.jdict = {}
        for j in range( p.getNumJoints(self.robot) ):
            info = p.getJointInfo(self.robot, j)
            ordered_joint_indices.append(j)
            
            if info[2] != p.JOINT_REVOLUTE: continue
            jname = info[1].decode("ascii")
            self.jdict[jname] = j
            lower, upper = (info[8], info[9])
            ordered_joints.append( (j, lower, upper) )
            p.setJointMotorControl2(self.robot, j, controlMode=p.VELOCITY_CONTROL, force=0)

        self.frame = 0
        self.startTime = time.time()
        self.jointSteps = {}
        self.startPosition, (qx, qy, qz, qw) = p.getBasePositionAndOrientation(self.robot)
    '''
    Adds the action of: 
        Move joint named @name@ by the angle @target_angle@ with the velocity @angular_velocity@
    to the pybullet simulation action list - does not run the simulation until "makeStep" is called.
    '''
    def moveJoint(self, name, target_angle, angular_velocity = 1):
        if not name in self.jdict:
            print ("Name is not valid")
            return

        limits = self.jRanges[name].split()
        if target_angle < int(limits[0]):
            target_angle = int(limits[0])
        elif target_angle > int(limits[1]):
            target_angle = int(limits[1])

        maxVelocity = int(self.jMaxVelocity[name])
        if angular_velocity > maxVelocity:
            angular_velocity = maxVelocity

        self.jointSteps[name] = math.radians(target_angle)
        p.setJointMotorControl2(bodyUniqueId=self.robot,
                                jointIndex=self.jdict[name],
                                controlMode=p.POSITION_CONTROL,
                                targetPosition=math.radians(target_angle),
                                maxVelocity=angular_velocity
                                )
    '''
    Runs the simulation by a single step (preforms the actions added to the simulation
    action list. Important note: many times a single step isn't enough to generate the entire
    action requested)
    '''
    def makeStep(self, cameraDistance = -1, cameraYaw = 0, cameraPitch = -20):
        p.stepSimulation()
        time.sleep(0.01)
        self.frame += 1

        epsilon = 0.2
        remove_list = []
        for joint in self.jointSteps:
            jointState = p.getJointState(self.robot, self.jdict[joint])
            jointPos = jointState[0]
            targetPos = self.jointSteps[joint]

            if jointPos - epsilon <= targetPos and targetPos <= jointPos + epsilon:
                remove_list.append(joint)
        
        for r in remove_list:
            del self.jointSteps[r]

        if cameraDistance != -1:
            robotPos, robotOrn = p.getBasePositionAndOrientation(self.robot)
            p.resetDebugVisualizerCamera(cameraDistance ,cameraYaw , cameraPitch, robotPos)
        return self.jointSteps
    '''
    the function returns the position value of the given joint
    '''
    def getJoinState(self, name):
        if not name in self.jdict:
            print ("getJoinState - Name is not valid")
            return
        jointState = p.getJointState(self.robot, self.jdict[name])
        return jointState[0]
    '''
    getPositionAndOrientation returns the position list of 3 floats and orientation as list of 4 
    floats in [x,y,z,w] order. Use getEulerFromQuaternion to convert the quaternion to Euler if needed.
    '''
    def getPositionAndOrientation(self):
        return p.getBasePositionAndOrientation(self.robot)
    '''
    getEulerFromQuaternion returns alist of 3 floating point values, a vec3.
    '''
    def getEulerFromQuaternion(self, orientation):
        return p.getEulerFromQuaternion(orientation)
    '''
    Ends the simulation run - prints the number of frames occured and FPS
    '''
    def stopRun(self):
        t2 = time.time()
        print ("FPS = ", 1000/ (t2 - self.startTime))
        print ("Total time in seconds: " + str((t2 - self.startTime).total_seconds()))
        print ("Frame amount = " + str(self.frame))
    '''
    Prints the XML structure.
    '''
    def prettyPrint(self, filePath = None):
        self.WB.prettyPrint(filePath)

    # Private functions - for internal use only.
    '''
    the function check if the given name already in use
    '''
    def _verifyName(self, name):
        if name in self.names:
            print("Name " + name + " already exists - created only the first one requested")
            return False
        self.names.append(name)
        return True
    @staticmethod
    def _addSpace():
        Robot_Module._spaces = Robot_Module._spaces + 4
    @staticmethod
    def _removeSpace():
        Robot_Module._spaces = Robot_Module._spaces - 4
    '''
    the function add new component (body/joint/geom) the the given parent
    '''
    def _addComponent(self, obj, parent):
        if self.finishBuild:
            print ("Can't add new componenets after start")
            return
        if parent == -1:
            self.lastBody.children.append(obj)
        else:
            point = self._findByName(self.WB.children,parent)
            if point is None:
                print("No parent is found")
                self.hasError = True
                return
            point.children.append(obj)

'''
WorldBody class - main body of the robot. Handles the XML creation.
'''
class _WorldBody:
    def __init__(self, name, time_step = 0.01):
        # The main name of the module
        self.name = name
        # type of object
        self.type = "WB"
        # list of children
        self.children = []
        # time step
        self.timestep = time_step

    '''
    the function print the current object data to a given file 
    or in case of filePath=None to the screen
    '''
    def prettyPrint(self, filePath):
        if filePath is None:
            self.fileName = None
        else:
            self.fileName = open(filePath + '.xml', 'w')
        
        self._printConstFile(self.fileName)

        self.printToFile(self.fileName, '<worldbody>')
        Robot_Module._addSpace()
        for c in self.children:
            c.prettyPrint(self.fileName)
        Robot_Module._removeSpace()
        self.printToFile(self.fileName, '</worldbody>')
        self.printToFile(self.fileName, '<actuator>')
        Robot_Module._addSpace()
        self._printAct(self.fileName)
        Robot_Module._removeSpace()
        self.printToFile(self.fileName, '</actuator>')
        Robot_Module._removeSpace()
        self.printToFile(self.fileName, '</mujoco>')

        if filePath is not None:
            self.fileName.close()
    '''
    the actual write to the file / print
    '''
    def printToFile(self, fileName, msg):
        for _ in range(Robot_Module._spaces):
            msg = " " + msg
        
        if fileName is None:
            print (msg)
        else:
            fileName.write(msg + "\n")

    # Private functions - for internal use only.
    '''
    This function add the defualt data to the xml file
    '''
    def _printConstFile(self, xml):
        self.printToFile(xml, '<mujoco model="' + self.name + '">')
        Robot_Module._addSpace()
        self.printToFile(xml, '<compiler angle="degree" coordinate="local" inertiafromgeom="true"/>')
        
        self.printToFile(xml, '<option integrator="RK4" timestep="' + str(self.timestep) + '"/>') 
        
        self.printToFile(xml, '<default>')
        Robot_Module._addSpace()
        self.printToFile(xml, '<joint limited="true"/>')
        self.printToFile(xml, '<geom friction="1 0.5 0.5" />')
        Robot_Module._removeSpace()
        self.printToFile(xml, '</default>')
    '''
    This function write the actuator data to the xml file
    '''
    def _printAct(self,fileName):
        if self.children is None:
            return

        for c in self.children:
            if c.type == "Joint" and int(c.gear) != 0:
                self.printToFile(fileName,'<motor ctrllimited="false" crtlrange="-1.0 1.0" joint="'+c.name+'" gear="'+str(c.gear)+'" />')
                Robot_Module._motor_names.append(c.name)
            c._printAct(fileName)

'''
Body class - all objects named "Body" in the XML are created here.
'''
class _Body(_WorldBody):
    def __init__(self, name, pos, parent):
        _WorldBody.__init__(self ,name)
        # Default position of the body
        self.pos = pos
        # the parent of the new body
        self.parent = parent
        # the type of the object
        self.type = "Body"
    
    '''
    Same sa in WorldBody
    '''
    def prettyPrint(self, fileName):
        self.printToFile(fileName, '<body ' + Utils.fix_attr("name",self.name) + Utils.fix_attr("pos",self.pos) + '>')
        Robot_Module._addSpace()
        for c in self.children:
            c.prettyPrint(fileName)
        Robot_Module._removeSpace()
        self.printToFile(fileName, '</body>')

'''
Joint class - all objects named "Joint" in the XML are created here.
'''
class _Joint(_WorldBody):
    def __init__(self,name,max_velocity,axis,joint_range,joint_type,gear,parent):
        _WorldBody.__init__(self,name)
        # Joint max velocity
        self.max_velocity = max_velocity
        # Joint parent
        self.parent = parent
        # Joint axis - e.g. "1 1 0"
        self.axis = axis
        # Joint range
        self.joint_range = joint_range
        # Joint type - alawys will be 'hinge'
        self.joint_type = joint_type
        # Joint gear
        self.gear = gear
        # Object type
        self.type = "Joint"
        # Joint position
        self.pos = "0 0 0"

    '''
    Look at WorldBody documentation
    '''
    def prettyPrint(self, fileName):
        self.printToFile(fileName, '<joint ' + Utils.fix_attr("name",self.name)
             + Utils.fix_attr("axis",self.axis) + Utils.fix_attr("pos",self.pos) + Utils.fix_attr("range",self.joint_range)
             + Utils.fix_attr("type",self.joint_type) + '>')
        Robot_Module._addSpace()
        for c in self.children:
            c.prettyPrint(fileName)
        Robot_Module._removeSpace()
        self.printToFile(fileName, '</joint>')

'''
Geom class - all objects named "Geom" in the XML are created here.
'''
class _Geom(_WorldBody):
    def __init__(self,name,fromto,size,geom_type,parent):
        _WorldBody.__init__(self,name)
        # Geom fromTo data - can be 3 or 6 coordinates depends on the geom_type
        self.fromto = fromto
        # Geom size
        self.size = size
        # Geom type
        self.geom_type = geom_type
        # Geom parent name
        self.parent = parent
        # Type of object
        self.type = "Geom"
    
    '''
    Look at WorldBody
    '''
    def prettyPrint(self, fileName):
        self.printToFile(fileName, '<geom ' + Utils.fix_attr("name", self.name) + Utils.fix_attr("fromto",self.fromto)
                + Utils.fix_attr("type",self.geom_type) + Utils.fix_attr("size",self.size) + '>')
        Robot_Module._addSpace()
        for c in self.children:
            c.prettyPrint(fileName)
        Robot_Module._removeSpace()
        self.printToFile(fileName, '</geom>')

class Utils:
    # All the supported geom types and thier fromto dimension
    geom_type_fromto_dic = { "capsule":6, "box":0, "sphere": 0, "cylinder": 6 }
    # All the supported geom types and thier size dimension
    geom_type_size_dic = { "capsule":1, "box":3, "sphere": 1, "cylinder": 1 }

    '''
    The function return True if the given text is empty or Null, otherwise it returns False
    '''
    @staticmethod
    def stringIsNullOrEmpty(text):
        return text is None or text == ""
    '''
    The function return True if the split array with blankspace length equals to amount
    '''
    @staticmethod
    def dimensionAmount(text, amount):
        if Utils.stringIsNullOrEmpty(text):
            return False
        return (len(text.split()) == (amount))
    '''
    The function return a list of supported geom types
    '''
    @staticmethod
    def geomTypes():
        geom_list = []
        for key in Utils.geom_type_fromto_dic:
            geom_list.append(key)
        return geom_list
    '''
    The function returns the dimesnion of fromto for the given geom_type
    '''
    @staticmethod
    def geomTypeFromToDimension(geom_type):
        geom_type = str(geom_type).lower()
        if geom_type in Utils.geom_type_fromto_dic:
            return Utils.geom_type_fromto_dic[geom_type]
        return -1
    '''
    The function returns the dimesnion of size for the given geom_type
    '''
    @staticmethod
    def geomTypeSizeDimension(geom_type):
        geom_type = str(geom_type).lower()
        if geom_type in Utils.geom_type_size_dic:
            return Utils.geom_type_size_dic[geom_type]
        return -1
    '''
    The function gets a file path to and creates a WorldBody objects for this file
    '''
    @staticmethod
    def parseTxtToPBA(file_name):
        index = 0
        tab = 4
        norm_len = 9
        ident_bodies = {}
        ident_bodies[0] = -1
        last_line = None
        curr_line = None

        try:
            input_file = open(file_name)
        except:
            print("Couldn't open the file")
            return

        w = Robot_Module()

        for idx, line in enumerate(input_file):
            tmp_line = line.lstrip()
            if len(tmp_line) == 0 or tmp_line[0] == '#':
                continue

            last_line = curr_line
            curr_line = line.lstrip()
            line_ident = int(len(line) - len(curr_line))

            curr_line = curr_line.replace('\n', '')
            curr_line = curr_line.replace(', ', ',')
            curr_line = curr_line.split(',')

            if len(curr_line) != norm_len:
                print('Line number ' + str(idx+1) + ' size is invalid')
                exit(0)
            
            if int(line_ident/tab) > index:
                index = line_ident/tab
                ident_bodies[index] = last_line[0]+'_body'
            elif (line_ident/tab) < index:
                index = line_ident/tab

            father = ident_bodies[index]
            ans = [False,False,False]
            ans[0] = w.addBody(curr_line[0]+'_body',curr_line[1],father)
            ans[1] = w.addGeom(curr_line[0]+'_geom',curr_line[2],curr_line[3],curr_line[4])
            ans[2] = w.addJoint(curr_line[0]+'_joint',curr_line[5],curr_line[6],curr_line[7],curr_line[8])

            if False in ans:
                print('Error in ' + curr_line[0] + ' line number: ' + str(idx+1))
                exit(0)

        return w
    '''
    merge two strings with a equal char ('=') between them
    '''
    @staticmethod
    def fix_attr(name,value):
        return str(name) + '="' + str(value) + '" '
    '''
    return the path to the stadium.sdf  file for simulation environment
    '''
    @staticmethod
    def getDefaultSDF():
        import sys
        return sys.prefix + "\\Lib\\site-packages\\pybullet_data\\stadium.sdf"