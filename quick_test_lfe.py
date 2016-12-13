from object_sampling import *
from trajopt_testing import *
from lfe_testing import *

env = Environment() # create openrave environment 
env.SetViewer('qtcoin') # attach viewer (optional) 
env.GetPhysicsEngine().SetGravity([0,0,-10]) 

env.Load('/home/viki/trajopt_utils/bookshelf_easier_pulledoutextra.zae')
# env.Load('/home/viki/trajopt_utils/init_clutter.zae')
robot = env.GetRobot('pr2')
preObjs = len(env.GetBodies())
manip = robot.GetManipulator("rightarm")
robot.SetActiveDOFs(manip.GetArmIndices())
TPlanner = or_trajopt.TrajoptPlanner()
minLims = robot.GetActiveDOFLimits()[0]
maxLims = robot.GetActiveDOFLimits()[1]
startDOF = robot.GetActiveDOFValues()
ct = 0
errors = np.load('/home/viki/trajopt_utils/constrained/finalerror1bookshelf.txt.npy')
trajs = np.load('/home/viki/trajopt_utils/constrained/finaltraj1bookshelf.txt.npy')
traj = trajs[58][0][20]
start = traj[0]
goal = traj[-1]
manip = robot.GetManipulator("leftarm")
robot.SetActiveDOFs(manip.GetArmIndices())
test = robot.GetActiveDOFLimits()[1]
test[0] = 1
robot.SetActiveDOFValues(test)
manip = robot.GetManipulator("rightarm")
robot.SetActiveDOFs(manip.GetArmIndices())
robot.SetActiveDOFValues(start)
newstart = np.copy(start)
newstart[0] = newstart[0] - 0.2
newstart[1] = newstart[1] - 0.1
newstart[2] = newstart[2] - 0.05
# trajCheck = repairTrajectory(robot, 30, newstart, goal, traj)
trajCheck = np.copy(traj)
trajCheck[0] = newstart
robot.SetActiveDOFValues(newstart)
pdb.set_trace()
out = TPlanner.PlanToConfiguration(robot, goal, trajCheck)
newtraj = out.GetAllWaypoints2D()

for i in range(len(traj)):
  robot.SetActiveDOFValues(newtraj[i])
  if env.CheckCollision(robot) or robot.CheckSelfCollision():
  	print 'collided'
  time.sleep(0.07)