from openravepy import *
from numpy import *
import numpy as np

class GraspTransform:
    def __init__(self,env,target):
        self.env = env
        self.robot = env.GetRobots()[0]
        self.target= target

    def drawTransform(self,T,length=0.1):
        """draws a set of arrows around a coordinate system
        """
        return [self.env.drawarrow(p1=T[0:3,3],p2=T[0:3,3]+length*T[0:3,0],linewidth=0.01,color=[1.0,0.0,0.0]),
                self.env.drawarrow(p1=T[0:3,3],p2=T[0:3,3]+length*T[0:3,1],linewidth=0.01,color=[0.0,1.0,0.0]),
                self.env.drawarrow(p1=T[0:3,3],p2=T[0:3,3]+length*T[0:3,2],linewidth=0.01,color=[0.0,0.0,1.0])]
            
    def showGrasp(self,Tgrasp):
        """visualizes the robot configuration when robot.GetActiveManipulator().GetTransform()==Tgrasp
        
        :param Tgrasp: a row-major 4x4 matrix in numpy.array format
        """
        O_T_R = self.robot.GetTransform() # robot transform R in global frame O 
        O_T_G = self.robot.GetActiveManipulator().GetTransform() # grasping frame G in global frame O
        G_T_O = np.linalg.inv(O_T_G) # global frame O in grasping frame G
        G_T_R = np.dot(G_T_O, O_T_R) # robot frame R in grasping frame G
        O_T_G_goal = Tgrasp # final grasping frame G_goal in global frame O 
        O_T_R_goal = np.dot(O_T_G_goal,G_T_R) # final robot transform R_goal in global frame O                
        self.robot.SetTransform(O_T_R_goal)

env = Environment() # create openrave environment 
env.SetViewer('qtcoin')
"Main example code."
robot = env.ReadRobotXMLFile('robots/pr2-beta-static.zae')
env.AddRobot(robot)

body = RaveCreateKinBody(env,'')
name = 'objects1'
body.SetName(name) 
xMin = random.uniform(0, 1)
yMin = random.uniform(0, 1)
zMin = random.uniform(0, 1)
body.InitFromBoxes(np.array([[0,0,0,xMin,yMin,zMin]]), True)

target = body

# target = env.ReadKinBodyXMLFile('data/mug1.kinbody.xml')
env.AddKinBody(target)

# init target pose
O_T_Target = array([[1,0,0,1],
                    [0,1,0,1],
                    [0,0,1,1],
                    [0,0,0,1]])
target.SetTransform(O_T_Target)

robot.SetActiveManipulator('leftarm')
# init robot pose: l_shoulder_pan, r_shoulder_pan, torso, l_gripper_l_finger_joint
names = ['l_shoulder_pan_joint', 'r_shoulder_pan_joint', 'torso_lift_joint', 'l_gripper_l_finger_joint']
dofs = [robot.GetJoint(name).GetDOFIndex() for name in names]
robot.SetDOFValues([pi/2,-pi/2,0.31,0.54],dofs)
gt = GraspTransform(env,target)
handles = []
raw_input('This demo shows how to find the transform that moves the hand to the target.\npress ENTER to continue...')
print 'showing robot transform in global frame O_T_R'
handles = gt.drawTransform(gt.robot.GetTransform())
raw_input('press ENTER to continue...')
print 'showing target transform in global frame O_T_Target'
handles = gt.drawTransform(gt.target.GetTransform())
raw_input('press ENTER to continue...')
print 'showing grasping frame in global frame O_T_G'
handles = gt.drawTransform(gt.robot.GetActiveManipulator().GetTransform())
raw_input('press ENTER to continue...')
raw_input('Guess what the robot will look like when the hand is on the target?\npress ENTER to continue...')
gt.showGrasp(target.GetTransform())
raw_input('press ENTER to exit')