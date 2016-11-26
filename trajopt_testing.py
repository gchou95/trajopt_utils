from object_sampling import *
import or_trajopt
import trajoptpy.math_utils as mu

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
	if args.multi_init:
		print 'using random initializations'
		joint_waypoints.extend(sample_base_positions(robot, num=5))
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

	# Initialization
	robot = env.GetRobots()[0]
	if str(robot.GetName()) == 'pr2':
		manip = robot.GetManipulator('rightarm')
	else:
		manip = robot.GetManipulators()[0] 
	robot.SetActiveDOFs(manip.GetArmIndices())

	# Generate 20 random start configurations
	starts = []
	minLims = robot.GetActiveDOFLimits()[0]
	maxLims = robot.GetActiveDOFLimits()[1]

	for i in range(20):
		start = []
		for j in range(robot.GetActiveDOF()):
			randJoint = random.uniform(minLims[j], maxLims[j])
			start.append(randJoint)
		starts.append(start)

	# Running TrajOpt on each start configuration, with multiple initializations
	errors = []
	for start in starts:
		er = []
		robot.SetActiveDOFvalues(start)
		if env.CheckCollision(robot):
			continue
		else:
			initializations = gen_init_trajs(robot, 10, start, goal)
			for init in initializations:
				err = []
				try:
					TPlanner.PlanToConfiguration(robot, goal)
				except Exception, e:
					print e
				err.append(str(e))
				if err == []:
					break
			er.append(err)
		errors.append(er)

	# for idx in range(robot.GetActiveDOF()):
	# 	toCheck = np.linspace(minLims[idx], maxLims[idx], 3)
	# 	for gridpoint in toCheck:
	# 		# old = robot.GetActiveDOFValues()
	# 		robot.SetActiveDOFValues(values)
	# 		if env.CheckCollision(robot):
	# 			continue

	return errors



# def compareBIRRT(env, RRTPlanner, num_objects):
# 	