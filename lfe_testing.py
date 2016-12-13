
from trajopt_testing import *
import operator

global trajLib
global startLib
global goalLib
global initLib

def repairTrajectory(robot, n_steps, start_joints, end_joints, old_traj):
	waypoint_step = (n_steps - 1)// 2
	joint_waypoints = []
	print 'using random initializations'
	# joint_waypoints.extend(old_traj)
	joint_waypoints.extend(old_traj)
	pdb.set_trace()
	# inittraj[0:something] = mu.linspace2d(start_joints, old_traj[0])
	trajs = []
	for i, waypoint in enumerate(joint_waypoints):
		if i == 2493204:
			inittraj = mu.linspace2d(start_joints, end_joints, n_steps)
		else:
			inittraj = np.empty((n_steps, robot.GetActiveDOF()))
			inittraj[:waypoint_step+1] = mu.linspace2d(start_joints, waypoint, waypoint_step+1)
			inittraj[waypoint_step:] = mu.linspace2d(waypoint, end_joints, n_steps - waypoint_step)
		trajs.append(inittraj)
	return trajs

# def repairTrajectory(traj):
# 	return fix_init_trajs()

def globalSetup():
	trajLib = []
	startLib = []
	goalLib = []
	initLib = {}

def addToLibrary(obj, tag):
	if tag == 'traj':
		# list of trajs
		trajLib.append(obj)
	elif tag == 'start':
		# list of starts
		startLib.append(obj)
	elif tag == 'goal':
		# list of goals
		goalLib.append(obj)
	elif tag == 'init':
		# keys: (trajIdx, initIndex), value: collision fraction
		initLib[obj[0]] = obj[1]
	return 

def getBestTraj(trajLib, startLib, goalLib, initLib, start, goal):

	# calculate distances between existing start goal pairs and queried pair
	dist = np.array([sum(np.abs(startLib[i] - start)**2)**(1./2) for i in range(len(startLib))] + \
		[sum(np.abs(goalLib[i] - goal)**2)**(1./2) for i in range(len(goalLib))])

	# keep the indices of the top 10 distance wise ones
	keepDist = dist.argsort()[:10]

	# find the collision costs of each initialization in those 10 trajectories
	toConsider = {}
	for i in keepDist:
		bestCost = 1
		bestIndex = 0
		for j in range(len(trajLib[i])):
			cost = initLib[(i, j)]
			if cost < bestCost:
				bestCost = cost
				bestIndex = j
		toConsider[keepDist] = (dist[keepDist[i]], bestCost, bestIndex)

	# choose an initialization

	weightedCosts = {}
	for i in toConsider.keys():
		(d, c, idx) = toConsider[i]
		weightedCosts[(i, idx)] = d + 10*(c)
	sortedDict = sorted(weightedCosts.items(), key=operator.itemgetter(1))
	(trajIdx, initIdx) = sortedDict[0]

	# repair that initialization

	# maybe start with an initialization rather than an old trajectory?
	traj = repairTrajectory(robot, n_steps, start, goal, initLib[trajIdx][initIdx])
	# for i in range(len(trajStarts)):
	# 	dist.append()

	# DO (TRAJECTORY, START, GOAL, SUCCESS RATIO)

def testLFE(env, TPlanner):

	# Initialization of global variables
	globalSetup()

	# Initialization
	robot = env.GetRobots()[0]
	if str(robot.GetName()) == 'pr2':
		manip = robot.GetManipulator('rightarm')
	else:
		manip = robot.GetManipulators()[0] 
	robot.SetActiveDOFs(manip.GetArmIndices())

	# set goals
	goals = [np.array([-0.01001492,  0.73836064, -3.15755519, -0.41885679,  0.00771066,
       -0.53017778,  1.24829427])]

	# Generate 10 random start configurations
	starts = []
	minLims = robot.GetActiveDOFLimits()[0]
	maxLims = robot.GetActiveDOFLimits()[1]

	ct = 0
	while ct < 10:
		start = []
		olddof = robot.GetActiveDOFValues()
		for j in range(robot.GetActiveDOF()):
			if j == 4 or j == 6:
				randJoint = random.uniform(-2*np.pi, 2*np.pi)
			else:
				randJoint = random.uniform(minLims[j]+0.01, maxLims[j]-0.01)
			start.append(randJoint)
		robot.SetActiveDOFValues(start)
		if not env.CheckCollision(robot) and not robot.CheckSelfCollision():
			starts.append(start)
			ct += 1
		else:
			robot.SetActiveDOFValues(olddof)

	# Running TrajOpt on each start configuration, with multiple initializations
	errors = []
	trajs = []
	collisions = []

	for goal in goals:
		startIterCounter = 0
		for start in starts:
			er = []
			tr = []
			co = []
			robot.SetActiveDOFValues(start)
			print 'goal'
			if env.CheckCollision(robot):
				continue
			else:
				initializations = gen_init_trajs(robot, 15, start, goal)
				initializations.append(None)
				err = []
				tra = []
				col = []
				itercount = 0
				for init in initializations:
					e = []
					t = []
					traj = []
					collFrac = 1
					try:
						collCount = 0
						out = TPlanner.PlanToConfiguration(robot, goal, init)
						traj = out.GetAllWaypoints2D()
						olddof = robot.GetActiveDOFValues()
						for i in range(len(traj)):
							robot.SetActiveDOFValues(traj[i])
							if env.CheckCollision(robot):
								print 'collided'
								collCount += 1
								e = 'error: collision with environment'
								# break
						robot.SetActiveDOFValues(olddof)
						collFrac = collCount / len(traj)
					except Exception, e:
						pdb.set_trace()
						print e
					col.append(collFrac)
					tra.append(traj)
					err.append(str(e))
					if itercount == 0:
						idxLib = addToLibrary(start, 'start')
						idxLib = addToLibrary(goal, 'goal')
					addToLibrary([(startIterCounter, itercount), collFrac], 'init')
					itercount += 1
				co.append(col)
				er.append(err)
				tr.append(tra)
			collisions.append(co)
			errors.append(er)
			trajs.append(tr)
			startIterCounter += 1

	return errors, trajs, starts, initializations