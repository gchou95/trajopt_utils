from object_sampling import *
import or_trajopt
import trajoptpy.math_utils as mu
import pdb

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

def planToGrasp(env, TPlanner, num_objects):
	# for each object, get object's transform
	robot = env.GetRobots()[0]
	envBodies = env.GetBodies()[0:num_objects]
	objList = env.GetBodies()[num_objects:]

	envTransforms = []
	objTransforms = []
	for obj in envBodies:
		envTransforms.append(obj.GetTransform())
	for obj in objList:
		objTransforms.append(obj.GetTransform())

	errors = []
	for transform in objTransforms:
		e = []
		try:
			TPlanner.PlanToEndEffectorPose(robot, transform)
		except Exception, e:
			print e
		errors.append(str(e))

	return errors, envTransforms, objTransforms

	# send to trajopt as a destination
	# solve
	# catch failure
	# save the environment? or save list of all kinbodies, transforms of all kinbodies

def calc_world_bounds(env, ignore_names):
	inf = float('inf')
	lo, hi = np.array([inf, inf, inf]), -np.array([inf, inf, inf])
	for body in env.GetBodies():
		if body.GetName() in ignore_names: continue
		for link in body.GetLinks():
			for geom in link.GetGeometries():
				trans = body.GetTransform()#.dot(link.GetTransform().dot(geom.GetTransform()))
				aabb = geom.ComputeAABB(trans)
				lo = np.minimum(lo, aabb.pos() - aabb.extents())
				hi = np.maximum(hi, aabb.pos() + aabb.extents())
	return lo, hi

def sample_base_positions(robot, num=5):
	joints = np.asarray(robot.GetActiveDOFValues())
	env = robot.GetEnv()
	min_xyt, max_xyt = calc_world_bounds(env, ignore_names=[robot.GetName()])
	min_xyt[2], max_xyt[2] = 0, np.pi
	print 'world bounds', min_xyt, max_xyt
	out = []
	with robot:
		while len(out) < num:
			x, y, theta = np.random.rand(3)*(max_xyt-min_xyt) + min_xyt
			joints[-3:] = [x, y, theta]
			robot.SetActiveDOFValues(joints)
			if not env.CheckCollision(robot) and not robot.CheckSelfCollision():
				out.append(joints.copy())
	return np.asarray(out)

def gen_init_trajs(robot, n_steps, start_joints, end_joints):
	waypoint_step = (n_steps - 1)// 2
	joint_waypoints = [(np.asarray(start_joints) + np.asarray(end_joints))/2]
	print 'using random initializations'
	joint_waypoints.extend(sample_base_positions(robot, num=10))
	trajs = []
	for i, waypoint in enumerate(joint_waypoints):
		if i == 0:
			inittraj = mu.linspace2d(start_joints, end_joints, n_steps)
		else:
			inittraj = np.empty((n_steps, robot.GetActiveDOF()))
			inittraj[:waypoint_step+1] = mu.linspace2d(start_joints, waypoint, waypoint_step+1)
			inittraj[waypoint_step:] = mu.linspace2d(waypoint, end_joints, n_steps - waypoint_step)
		trajs.append(inittraj)
	return trajs

def planToJointPos(env, TPlanner, goal):
	# pdb.set_trace()
	# Initialization
	robot = env.GetRobots()[0]
	if str(robot.GetName()) == 'pr2':
		manip = robot.GetManipulator('rightarm')
	else:
		manip = robot.GetManipulators()[0] 
	robot.SetActiveDOFs(manip.GetArmIndices())

	# Generate 10 random start configurations
	starts = []
	minLims = robot.GetActiveDOFLimits()[0]
	maxLims = robot.GetActiveDOFLimits()[1]

	ct = 0
	while ct < 20:
		start = []
		olddof = robot.GetActiveDOFValues()
		for j in range(robot.GetActiveDOF()):
			if j == 0:
				randJoint = random.uniform(-0.3, 0.3)
			elif j == 3:
				randJoint = random.uniform(-0.8, -0.2)
			# else:
			elif j == 4 or j == 6:
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

	# pdb.set_trace()
	# Running TrajOpt on each start configuration, with multiple initializations
	errors = []
	trajs = []
	# starts = [np.array([ 0.1       ,  0.1       ,  0.1       , -0.19999999,  0.        ,
 #       -0.20000001,  0.10000001])] # for the cluttered start problem
	# starts = [np.array([ -9.96991008e-02,   9.93469212e-01,   4.05354266e-01,
 #         -5.11823993e-01,   4.34700273e+00,  -1.83598577e+00,
 #         -5.08743365e+00])]
	for start in starts:
		er = []
		tr = []
		robot.SetActiveDOFValues(start)
		print 'goal'
		print goal
		print robot.GetActiveDOFValues()
		if env.CheckCollision(robot):
			continue
		else:
			initializations = gen_init_trajs(robot, 30, start, goal)
			initializations.append(None)
			# pdb.set_trace()
			err = []
			tra = []
			for init in initializations:
				e = []
				t = []
				traj = []
				try:
					out = TPlanner.PlanToConfiguration(robot, goal, init)
					# pdb.set_trace()
					traj = out.GetAllWaypoints2D()
					olddof = robot.GetActiveDOFValues()
					for i in range(len(traj)):
						robot.SetActiveDOFValues(traj[i])
						if env.CheckCollision(robot):
							print 'collided'
							e = 'error: collision with environment'
							break
					robot.SetActiveDOFValues(olddof)
				except Exception, e:
					pdb.set_trace()
					print e
				tra.append(traj)
				err.append(str(e))
				# if e == []:
					# break
			er.append(err)
			tr.append(tra)
		errors.append(er)
		trajs.append(tr)

	# for idx in range(robot.GetActiveDOF()):
	# 	toCheck = np.linspace(minLims[idx], maxLims[idx], 3)
	# 	for gridpoint in toCheck:
	# 		# old = robot.GetActiveDOFValues()
	# 		robot.SetActiveDOFValues(values)
	# 		if env.CheckCollision(robot):
	# 			continue

	return errors, trajs, starts



# def compareBIRRT(env, RRTPlanner, num_objects):
# 	