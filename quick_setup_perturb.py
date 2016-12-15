from plotting_utils import *
from trajopt_testing import *
from object_sampling import *

env = Environment() # create openrave environment 
# env.SetViewer('qtcoin') # attach viewer (optional) 
# env.GetPhysicsEngine().SetGravity([0,0,-10]) 

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
robot.SetActiveDOFValues(startDOF)

errors = np.load('/home/viki/trajopt_utils/perturbedErrorBookshelf.txt.npy')
trajs = np.load('/home/viki/trajopt_utils/perturbedTrajBookshelf.txt.npy')
initializations = np.load('/home/viki/trajopt_utils/perturbedInitBookshelf.txt.npy')
starts, goals = getStartsAndGoals(trajs)
# goals = [goals[0]]
goals = [np.array([-0.11860579, -0.27078926, -2.53044859, -0.43059192,  4.75675355,
       -0.47717729, -2.1130558 ])]
startKeep = [np.array([-0.10694262,  0.54494402, -1.21343519, -0.53959419,  4.39120272,
       -1.16924255,  0.52026279])]
# startKeep = [np.array([ 0.24519869,  0.3484765 , -2.91996497, -0.29538065,  6.25284509,  -0.90046152, -0.84571456])] # (successRatio[13])
# startKeep = [np.array([-0.01366614, -0.09987283, -2.84752081, -0.40774119,  1.83361928,  -1.40117315, -5.02086834])] (successRatio[0])
limCoeff = 0.05
allStarts = perturbDOFOne(env, startKeep, limCoeff)
errors, trajs, initializations = testPerturb(env, TPlanner, allStarts, goals)