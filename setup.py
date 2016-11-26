from object_sampling import *
from trajopt_testing import *

env = Environment() # create openrave environment 
env.SetViewer('qtcoin') # attach viewer (optional) 
env.GetPhysicsEngine().SetGravity([0,0,-10]) 

env.Load("robots/pr2-beta-static.zae")
robot = env.GetRobot('pr2')
joint_start = [-1.832, -0.332, -1.011, -1.437, -1.1  , -2.106,  3.074]
robot.SetDOFValues(joint_start, robot.GetManipulator('rightarm').GetArmIndices())
joint_target = [0.062, 1.287, 0.1, -1.554, -3.011, -0.268, 2.988]

# module = RaveCreateModule(env, 'urdf')
# name = module.SendCommand('load /home/viki/catkin_ws/src/kinova-ros/kinova_description/urdf/jaco.urdf /home/viki/catkin_ws/src/kinova-ros/kinova_description/urdf/jaco.srdf')
# robot = env.GetRobot(name)
# joint_start = [0,3,0,2.5,0,4,0,0,0,0]
# robot.SetDOFValues(joint_start)

# Create floor
floor = RaveCreateKinBody(env, '')
floorExtents_x = [0,7]
floorExtents_y = [0,7]
floorExtents_z = [0,0.01]
floor.InitFromBoxes(np.array([[x for t in zip(floorExtents_x, floorExtents_y, floorExtents_z) \
	for x in t]]), True)
floor.SetName('floor')
env.Add(floor,True)

filenames = listdir('/home/viki/trajopt_utils/furniture')
minExtents, maxExtents = getExtents('/home/viki/trajopt_utils/furniture')
furniture = ['/home/viki/trajopt_utils/furniture/' + name for name in filenames]
names = getNames('/home/viki/trajopt_utils/furniture')

alpha = 1
beta = 0
num_f, num_o = objectDensity(alpha, len(furniture))
angles = objectClutter(beta)
# num_f = 5
# num_o = 5*num_f
limsFloor = env.GetKinBody('floor').ComputeAABB()
transforms = sampleFurniture(num_f, env, limsFloor.pos() - limsFloor.extents(), \
	limsFloor.pos() + limsFloor.extents(), furniture, names, angles, minExtents, maxExtents)
# relRot = np.array([[ -4.82484691e-02,   9.98835364e-01,   1.19070411e-07,   2.45494914e+00],
# 	[ -9.98835364e-01,  -4.82484691e-02,  -1.24960937e-07,  -8.58813822e-02],
# 	[ -1.19070438e-07,  -1.24960912e-07,   1.00000000e+00,   2.34236717e-01],
# 	[  0.00000000e+00,   0.00000000e+00,   0.00000000e+00,   1.00000000e+00]])
relRot = np.array([[  1.00000000e+00,  -2.22044605e-16,   1.05282905e-23,  -1.05362874e-06],
 [  2.22044605e-16,   1.00000000e+00,   1.05282905e-23,   2.38317718e-01],
 [ -1.05282905e-23,  -1.05282905e-23,   1.00000000e+00,   2.89634588e-01],
 [  0.00000000e+00,   0.00000000e+00,   0.00000000e+00,   1.00000000e+00]])


pdb.set_trace()

robot.SetTransform(np.dot(relRot, transforms[0]))
preObjs = len(env.GetBodies())
# Sample objects
sampleObjects(num_f, num_o, env, minExtents, maxExtents, transforms, angles)
pdb.set_trace()
dragToGround(env, env.GetBodies()[preObjs:])

manip = robot.GetManipulator("rightarm")
ikmodel= databases.inversekinematics.InverseKinematicsModel(robot,\
  iktype=IkParameterizationType.Transform6D) 
if not ikmodel.load(): 
  ikmodel.autogenerate()
TPlanner = or_trajopt.TrajoptPlanner()
pdb.set_trace()
errors, envTransforms, objTransforms = planToGrasp(env, TPlanner, preObjs)

pdb.set_trace() # here, manually set a goal

errors = planToJointPos(env, TPlanner, goal)

# pdb.set_trace()
# # env.SetPhysicsEngine(RaveCreatePhysicsEngine(env, 'bullet'))
# env.GetPhysicsEngine().SetGravity([0,0,-5])
# time.sleep(0.1)
# for body in env.GetBodies():
# 	temp = body.GetLinks()
# 	for idx in range(len(temp)):
# 		temp[idx].SetStatic(True)