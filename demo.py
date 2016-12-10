import time
import openravepy
import trajoptpy
import numpy as np
import or_trajopt
import json
import time
import pdb

env = openravepy.Environment()
env.SetViewer('qtcoin')
module = openravepy.RaveCreateModule(env, 'urdf')
name = module.SendCommand('load /home/viki/catkin_ws/src/kinova-ros/kinova_description/urdf/jaco.urdf /home/viki/catkin_ws/src/kinova-ros/kinova_description/urdf/jaco.srdf')
robot = env.GetRobot(name)
env.Load("/home/viki/my_trajopt/data/table.xml")

robot = env.GetRobots()[0]
env.Load("/home/viki/trajopt_utils/objects/mug1.kinbody.xml")

raw_input("Press Enter to continue...")

mugTrans = np.array([[  1.00000000e+00,   0.00000000e+00,   0.00000000e+00,
          5.51362038e-01],
       [  0.00000000e+00,   0.00000000e+00,  -1.00000000e+00,
         -3.04697005e-06],
       [  0.00000000e+00,   1.00000000e+00,   0.00000000e+00,
          8.41973305e-01],
       [  0.00000000e+00,   0.00000000e+00,   0.00000000e+00,
          1.00000000e+00]])

mug = env.GetKinBody('mug')
mugTrans = mug.SetTransform(mugTrans)

joint_start = [-0.5,3,0,2.5,0,4,0,0,0,0]
robot.SetDOFValues(joint_start)

# goal = mugTrans
joint_target = np.array([  0.00000000e+00,   3.00000000e+00,   0.00000000e+00,
         2.50000000e+00,   0.00000000e+00,   4.00000000e+00,
         0.00000000e+00,   2.22044605e-16,   0.00000000e+00,
        -1.11022302e-16]).tolist()

request = {
  "basic_info" : {
    "n_steps" : 10,
    "manip" : "j2s7s300+j2s7s300_joint_finger_1+j2s7s300_joint_finger_2+j2s7s300_joint_finger_3", # see below for valid values
    # "manip" : "j2s7s300+j2s7s300_joint_finger_tip_1+j2s7s300_joint_finger_tip_2+j2s7s300_joint_finger_tip_3", # see below for valid values
    "start_fixed" : True # i.e., DOF values at first timestep are fixed based on current robot state
  },
  "costs" : [
  {
    "type" : "joint_vel", # joint-space velocity cost
    "params": {"coeffs" : [1]} # a list of length one is automatically expanded to a list of length n_dofs
    # also valid: [1.9, 2, 3, 4, 5, 5, 4, 3, 2, 1]
  },
  {
    "type" : "collision",
    "params" : {
      "coeffs" : [20], # penalty coefficients. list of length one is automatically expanded to a list of length n_timesteps
      "dist_pen" : [0.025] # robot-obstacle distance that penalty kicks in. expands to length n_timesteps
    },    
  }
  ],
  "constraints" : [
  {
    "type" : "joint", # joint-space target
    "params" : {"vals" : joint_target } # length of vals = # dofs of manip
  }
  ],
  "init_info" : {
      "type" : "straight_line", # straight line in joint space.
      "endpoint" : joint_target
  }
}
s = json.dumps(request) # convert dictionary into json-formatted string
prob = trajoptpy.ConstructProblem(s, env) # create object that stores optimization problem
t_start = time.time()
result = trajoptpy.OptimizeProblem(prob) # do optimization
t_elapsed = time.time() - t_start
print result
print "optimization took %.3f seconds"%t_elapsed
raw_input("Press Enter to continue...")

from trajoptpy.check_traj import traj_is_safe
prob.SetRobotActiveDOFs() # set robot DOFs to DOFs in optimization problem
assert traj_is_safe(result.GetTraj(), robot) # Check that trajectory is collision free

traj = result.GetTraj()
for i in range(len(traj)):
  robot.SetDOFValues(traj[i], prob.GetDOFIndices())
  time.sleep(0.5)

# BEGIN ik
# manip = robot.GetManipulator("j2s7s300")
# ikmodel= openravepy.databases.inversekinematics.InverseKinematicsModel(robot,\
#   iktype=openravepy.IkParameterizationType.Transform6D) 
# if not ikmodel.load(): 
#   ikmodel.autogenerate()
# TPlanner = or_trajopt.TrajoptPlanner()
# out = TPlanner.PlanToEndEffectorPose(robot, goal)
# traj = out.GetAllWaypoints2D()

# raw_input("Press Enter to continue...")

# for i in range(len(traj)):
#   robot.SetDOFValues(traj[i], np.array(range(len(traj[0]))))
#   time.sleep(0.5)

def closeFingers(env, robot):
	dofs = robot.GetDOFValues()
	# pdb.set_trace()
	while True:
		dofsold = dofs
		dofs[-3:] = dofs[-3:] + np.array([0.01, 0.01, 0.01])
		robot.SetDOFValues(dofs)
		if env.CheckCollision(robot):
			robot.SetDOFValues(dofs)
			break

closeFingers(env, robot)


