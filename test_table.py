import argparse
parser = argparse.ArgumentParser()
parser.add_argument("--interactive", action="store_true")
args = parser.parse_args()

import openravepy
import trajoptpy
import json
import time

env = openravepy.Environment()
env.SetViewer('qtcoin')
# env.StopSimulation()
# env.Load("robots/pr2-beta-static.zae")

module = openravepy.RaveCreateModule(env, 'urdf')
name = module.SendCommand('load /home/viki/catkin_ws/src/kinova-ros/kinova_description/urdf/jaco.urdf /home/viki/catkin_ws/src/kinova-ros/kinova_description/urdf/jaco.srdf')
# name = module.SendCommand('load /home/viki/catkin_ws/src/kinova-ros/kinova_description/urdf/jaco_test.urdf /home/viki/catkin_ws/src/kinova-ros/kinova_description/urdf/jaco_test.srdf' '/home/viki/catkin_ws/src/kinova-ros/kinova_description/urdf/jaco_test.xml')
# name = module.SendCommand('load /home/viki/catkin_ws/src/kinova-ros/kinova_description/urdf/jaco.urdf /home/viki/catkin_ws/src/kinova-ros/kinova_description/urdf/jaco.srdf')
robot = env.GetRobot(name)
print robot
env.Load("/home/viki/my_trajopt/data/table.xml")
# robot.SetDOFValues([0,3,0,2,0,4,0,0,0,0])
# robot.SetActiveManipulator("j2s7s300")

trajoptpy.SetInteractive(args.interactive) # pause every iteration, until you press 'p'. Press escape to disable further plotting
# robot = env.GetRobots()[0]

# joint_start = [-1.832, -0.332, -1.011, -1.437, -1.1  , -2.106,  3.074]
joint_start = [0,3,0,2.5,0,4,0,0,0,0]
robot.SetDOFValues(joint_start)
# robot.SetDOFValues(joint_start, robot.GetManipulator('rightarm').GetArmIndices())

# joint_target = [0.062, 1.287, 0.1, -1.554, -3.011, -0.268, 2.988]
# joint_target = [1,2,1,3,1,3,1,1,1,1]
joint_target = [1,2,1,3,1,3,1,1,1,1]


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