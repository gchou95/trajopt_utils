import time
import openravepy
import trajoptpy
import numpy as np
import or_trajopt

env = openravepy.Environment()
env.SetViewer('qtcoin')
module = openravepy.RaveCreateModule(env, 'urdf')
name = module.SendCommand('load /home/viki/catkin_ws/src/kinova-ros/kinova_description/urdf/jaco.urdf /home/viki/catkin_ws/src/kinova-ros/kinova_description/urdf/jaco.srdf')
robot = env.GetRobot(name)
env.Load("/home/viki/my_trajopt/data/table.xml")

robot = env.GetRobots()[0]
joint_start = [0,3,0,2.5,0,4,0,0,0,0]
robot.SetDOFValues(joint_start)

# BEGIN ik
manip = robot.GetManipulator("j2s7s300")
ikmodel= openravepy.databases.inversekinematics.InverseKinematicsModel(robot,\
  iktype=openravepy.IkParameterizationType.Transform6D) 
if not ikmodel.load(): 
  ikmodel.autogenerate()
TPlanner = or_trajopt.TrajoptPlanner()
out = TPlanner.PlanToEndEffectorPose(robot, manip.GetEndEffectorTransform())
traj = out.GetAllWaypoints2D()

raw_input("Press Enter to continue...")

for i in range(len(traj)):
  robot.SetDOFValues(traj[i], np.array(range(len(traj[0]))))
  time.sleep(0.5)