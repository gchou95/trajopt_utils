from itertools import izip
import openravepy
if not __openravepy_build_doc__:
    from openravepy import *
    from numpy import *

class FastGrasping:
    class GraspingException(Exception):
        def __init__(self,args):
            self.args=args

    def __init__(self,robot,target):
        self.robot = robot
        self.ikmodel = databases.inversekinematics.InverseKinematicsModel(robot=robot,iktype=IkParameterization.Type.Transform6D)
        if not self.ikmodel.load():
            self.ikmodel.autogenerate()
        self.gmodel = databases.grasping.GraspingModel(robot,target)
        self.gmodel.init(friction=0.4,avoidlinks=[])

    def checkgraspfn(self, contacts,finalconfig,grasp,info):
        # check if grasp can be reached by robot
        Tglobalgrasp = self.gmodel.getGlobalGraspTransform(grasp,collisionfree=True)
        # have to set the preshape since the current robot is at the final grasp!
        self.gmodel.setPreshape(grasp)
        sol = self.gmodel.manip.FindIKSolution(Tglobalgrasp,True)
        if sol is not None:
            jointvalues = array(finalconfig[0])
            jointvalues[self.gmodel.manip.GetArmIndices()] = sol
            raise self.GraspingException([grasp,jointvalues])
        return True

    def computeGrasp(self):
        approachrays = self.gmodel.computeBoxApproachRays(delta=0.02,normalanglerange=0.5) # rays to approach object
        standoffs = [0]
        # roll discretization
        rolls = arange(0,2*pi,0.5*pi)
        # initial preshape for robot is the released fingers
        with self.gmodel.target:
            self.gmodel.target.Enable(False)
            taskmanip = interfaces.TaskManipulation(self.robot)
            final,traj = taskmanip.ReleaseFingers(execute=False,outputfinal=True)
            preshapes = array([final])
        try:
            self.gmodel.disableallbodies=False
            self.gmodel.generate(preshapes=preshapes,standoffs=standoffs,rolls=rolls,approachrays=approachrays,checkgraspfn=self.checkgraspfn,graspingnoise=0.01)
            return None,None # did not find anything
        except self.GraspingException, e:
            return e.args

env = Environment() # create openrave environment 
env.SetViewer('qtcoin')
"Main example code."
# robot = env.ReadRobotXMLFile('robots/pr2-beta-static.zae')
# env.AddRobot(robot)
module = openravepy.RaveCreateModule(env, 'urdf')
name = module.SendCommand('load /home/viki/catkin_ws/src/kinova-ros/kinova_description/urdf/jaco.urdf /home/viki/catkin_ws/src/kinova-ros/kinova_description/urdf/jaco.srdf')
robot = env.GetRobot(name)
target = env.ReadKinBodyXMLFile('data/mug1.kinbody.xml')
env.AddKinBody(target)

"Main example code."
# env.Load(options.scene)
# robot = env.GetRobots()[0]
# if options.manipname is not None:
robot.SetActiveManipulator('j2s7s300')
# find an appropriate target
bodies = [b for b in env.GetBodies() if not b.IsRobot() and linalg.norm(b.ComputeAABB().extents()) < 0.2]
for body in bodies:
    self = FastGrasping(robot,target=body)
    grasp,jointvalues = self.computeGrasp()
    if grasp is not None:
        print 'grasp is found!'
        self.gmodel.showgrasp(grasp)
        self.robot.SetDOFValues(jointvalues)
        raw_input('press any key')