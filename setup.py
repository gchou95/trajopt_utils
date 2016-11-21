from object_sampling import *

env = Environment() # create openrave environment 
env.SetViewer('qtcoin') # attach viewer (optional) 
env.GetPhysicsEngine().SetGravity([0,0,-10]) 

module = RaveCreateModule(env, 'urdf')
name = module.SendCommand('load /home/viki/catkin_ws/src/kinova-ros/kinova_description/urdf/jaco.urdf /home/viki/catkin_ws/src/kinova-ros/kinova_description/urdf/jaco.srdf')
robot = env.GetRobot(name)
joint_start = [0,3,0,2.5,0,4,0,0,0,0]
robot.SetDOFValues(joint_start)

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
preObjs = len(env.GetBodies())
# Sample objects
sampleObjects(num_f, num_o, env, minExtents, maxExtents, transforms, angles)
pdb.set_trace()
dragToGround(env, env.GetBodies()[preObjs:])