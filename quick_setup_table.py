from object_sampling import *
from trajopt_testing import *

env = Environment() # create openrave environment 
env.SetViewer('qtcoin') # attach viewer (optional) 
env.GetPhysicsEngine().SetGravity([0,0,-10]) 

# env.Load('/home/viki/trajopt_utils/bookshelf_easier_pulledoutextra.zae')
env.Load('/home/viki/trajopt_utils/full_table_harder.zae')
robot = env.GetRobot('pr2')
preObjs = len(env.GetBodies())
manip = robot.GetManipulator("rightarm")
robot.SetActiveDOFs(manip.GetArmIndices())
TPlanner = or_trajopt.TrajoptPlanner()
minLims = robot.GetActiveDOFLimits()[0]
maxLims = robot.GetActiveDOFLimits()[1]
startDOF = robot.GetActiveDOFValues()
ct = 0
goals = []
for _ in range(5):
	while True:
		g = []
		ct += 1
		for i in range(robot.GetActiveDOF()):
			# if i == 0:
			# 	randJoint = random.uniform(-0.3, 0.3)
			# elif i == 3:
			# 	randJoint = random.uniform(-0.8, -0.2)
			if i == 4 or i == 6:
				randJoint = random.uniform(-2*np.pi, 2*np.pi)
			else:
				randJoint = random.uniform(minLims[i]+0.01, maxLims[i]-0.01)
			g.append(randJoint)
		robot.SetActiveDOFValues(g)
		if not env.CheckCollision(robot) and not robot.CheckSelfCollision():
			goal = g
			ct = 0
			break
	goals.append(goal)
	# elif ct > 20:
	# 	for i in env.GetBodies()[preObjs:]:
	# 		if env.CheckCollision(robot, i):
	# 			i.SetTransform(np.eye(4))

# specialized goal states
# goal = [ -1.94010912e-01,   6.89854694e-01,  -9.77339159e-01,
#         -2.36721933e-01,  -9.79539662e+03,  -1.60082215e+00,
#          6.04607519e+03] # for the bookshelf problem
goals = [[-.6,0,-1.5,-1.5,0,-.6,0]] #-> for the tabletop planning problem
# goal = [-0.8, -0.3,  0.1, -0.19999999,  0., -0.20000001,  0.10000001] -> for the cluttered init problem
robot.SetActiveDOFValues(startDOF)
errors, trajs, starts, initializations = planToJointPos(env, TPlanner, goals)