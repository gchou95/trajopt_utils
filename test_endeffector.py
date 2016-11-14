import argparse
parser = argparse.ArgumentParser()
parser.add_argument("--interactive", action="store_true")
parser.add_argument("--position_only", action="store_true")
args = parser.parse_args()
import pdb

import openravepy
from openravepy import *
import trajoptpy
import json
import numpy as np
import trajoptpy.kin_utils as ku

env = openravepy.Environment()
env.SetViewer('qtcoin')
module = openravepy.RaveCreateModule(env, 'urdf')
name = module.SendCommand('load /home/viki/catkin_ws/src/kinova-ros/kinova_description/urdf/jaco.urdf /home/viki/catkin_ws/src/kinova-ros/kinova_description/urdf/jaco.srdf')
# name = module.SendCommand('load /home/viki/catkin_ws/src/kinova-ros/kinova_description/urdf/jaco_test.urdf /home/viki/catkin_ws/src/kinova-ros/kinova_description/urdf/jaco_test.srdf' '/home/viki/catkin_ws/src/kinova-ros/kinova_description/urdf/jaco_test.xml')
# name = module.SendCommand('load /home/viki/catkin_ws/src/kinova-ros/kinova_description/urdf/jaco.urdf /home/viki/catkin_ws/src/kinova-ros/kinova_description/urdf/jaco.srdf')
robot = env.GetRobot(name)
print robot
env.Load("/home/viki/my_trajopt/data/table.xml")

trajoptpy.SetInteractive(args.interactive) # pause every iteration, until you press 'p'. Press escape to disable further plotting

robot = env.GetRobots()[0]
joint_start = [0,3,0,2.5,0,4,0,0,0,0]
robot.SetDOFValues(joint_start)

# quat_target = [1,0,0,0] # wxyz
# xyz_target = [6.51073449e-01,  -1.87673551e-01, 4.91061915e-01]
# hmat_target = openravepy.matrixFromPose( np.r_[quat_target, xyz_target] )

# BEGIN ik
manip = robot.GetManipulator("j2s7s300")
# joints = robot.GetJoints()
# jointnames = [j.GetName() for j in joints]
# robot.SetActiveDOFs([robot.GetJoint(name).GetDOFIndex() for name in jointnames])
# manip.SetIkSolver(openravepy.RaveCreateIkSolver(env, 'ikfast_pr2_rightarm'))
# init_joint_target = ku.ik_for_link(hmat_target, manip, "j2s7s300_link_7",
#     filter_options = openravepy.IkFilterOptions.CheckEnvCollisions)
ikmodel=databases.inversekinematics.InverseKinematicsModel(robot,iktype=IkParameterizationType.Transform6D) 
if not ikmodel.load(): 
  ikmodel.autogenerate()
quat_target = [1,0,0,0] # wxyz
xyz_target = [6.51073449e-01,  -1.87673551e-01, 4.91061915e-01]
pose = openravepy.matrixFromPose( np.r_[quat_target, xyz_target] )
# pose = manip.GetEndEffectorTransform()
pose = np.array([[ -.707372017e-02,   1.46850361e-12,  -.997494987e-01,
          3.80807626e-01],
       [ -2.66223425e-12,  -1.00000000e+00,  -1.28339954e-12,
          1.30000000e-02],
       [ -9.97494987e-01,   2.56478123e-12,   7.07372017e-02,
          8.94489672e-01],
       [  0.00000000e+00,   0.00000000e+00,   0.00000000e+00,
          1.00000000e+00]])

# pdb.set_trace()
init_joint_target = ku.ik_for_link(pose, manip, "j2s7s300_link_7",
    filter_options = openravepy.IkFilterOptions.CheckEnvCollisions)
# if ranker is None:
from prpy.ik_ranking import NominalConfiguration
ranker = NominalConfiguration(manip.GetArmDOFValues())

# Find initial collision-free IK solution.
ik_param = IkParameterization(pose, IkParameterizationType.Transform6D)
ik_solutions = manip.FindIKSolutions(ik_param, IkFilterOptions.CheckEnvCollisions)
# if not len(ik_solutions):
#   # Identify collision and raise error.
#   self._raiseCollisionErrorForPose(robot, robot_checker, pose)

# Sort the IK solutions in ascending order by the costs returned by the
# ranker. Lower cost solutions are better and infinite cost solutions
# are assumed to be infeasible.
scores = ranker(robot, ik_solutions)
best_idx = numpy.argmin(scores)
init_joint_config = ik_solutions[best_idx]

# Convert IK endpoint transformation to pose. OpenRAVE operates on
# GetEndEffectorTransform(), which is equivalent to:
#
#   GetEndEffector().GetTransform() * GetLocalToolTransform()
#
link_pose = numpy.dot(pose, numpy.linalg.inv(manip.GetLocalToolTransform()))
goal_position = link_pose[0:3, 3].tolist()
goal_rotation = openravepy.quatFromRotationMatrix(link_pose).tolist()

# END ik


request = {
  "basic_info" : {
    "n_steps" : 10,
    "manip" : 'active',
    # "start_fixed" : True
    # "manip" : "j2s7s300+j2s7s300_joint_finger_1+j2s7s300_joint_finger_2+j2s7s300_joint_finger_3", # see below for valid values
    "start_fixed" : True # i.e., DOF values at first timestep are fixed based on current robot state
  },
  "costs" : [
  {
    "type" : "joint_vel", # joint-space velocity cost
    "params": {"coeffs" : [1]} # a list of length one is automatically expanded to a list of length n_dofs
  },
  {
    "type" : "collision",
    "name" :"cont_coll", # shorten name so printed table will be prettier
    "params" : {
      # "continuous" : True,
      "coeffs" : [20], # penalty coefficients. list of length one is automatically expanded to a list of length n_timesteps
      "dist_pen" : [0.025] # robot-obstacle distance that penalty kicks in. expands to length n_timesteps
    }
  }
  ],
  "constraints" : [
  # BEGIN pose_constraint
  {
    "type" : "pose", 
    "params" : {"xyz" : goal_position, 
                "wxyz" : goal_rotation, 
                "link": "j2s7s300_link_7",
                "timestep" : 9
                }
                 
  }
  # END pose_constraint
  ],
  # BEGIN init
  "init_info" : {
      "type" : "straight_line", # straight line in joint space.
      "endpoint" : init_joint_config.tolist() # need to convert numpy array to list
  }
  # END init
}

if args.position_only: request["constraints"][0]["params"]["rot_coeffs"] = [0,0,0]

s = json.dumps(request) # convert dictionary into json-formatted string
pdb.set_trace()

prob = trajoptpy.ConstructProblem(s, env) # create object that stores optimization problem
result = trajoptpy.OptimizeProblem(prob) # do optimization
print result

from trajoptpy.check_traj import traj_is_safe
prob.SetRobotActiveDOFs() # set robot DOFs to DOFs in optimization problem
assert traj_is_safe(result.GetTraj(), robot) # Check that trajectory is collision free

# Now we'll check to see that the final constraint was satisfied
robot.SetActiveDOFValues(result.GetTraj()[-1])
posevec = openravepy.poseFromMatrix(robot.GetLink("j2s7s300_link_7").GetTransform())
quat, xyz = posevec[0:4], posevec[4:7]

quat *= np.sign(quat.dot(quat_target))
if args.position_only:
    assert (quat - quat_target).max() > 1e-3
else:
    assert (quat - quat_target).max() < 1e-3

