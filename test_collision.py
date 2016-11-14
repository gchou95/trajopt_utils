# def main(env,options):

from openravepy import *
import trajoptpy
import json
import time
import random
from numpy import *
import pdb

pdb.set_trace()
env = Environment()
env.SetViewer('qtcoin')

"""Main example code.
"""
env.Load('robots/barrettwam.robot.xml')
# register an optional collision callback
# handle = env.RegisterCollisionCallback(collisioncallback)
robot1 = env.GetRobots()[0]

try:
    # when doing fast ray collision checking, can specify multiple rays where each column is one ray
    ray1 = array((0,0,-10,0,0,100)) # specify dir*range, pos
    ray2 = array((0,0,10,0,0,-100)) # specify dir*range, pos
    inliers,hitpoints = env.CheckCollisionRays(r_[[ray1],[ray2]],robot1)
    print 'rays hit:',inliers,'hit points:',hitpoints

    # can get more fine grained information
    report1 = CollisionReport()
    inlier1 = env.CheckCollision(Ray(ray1[0:3],ray1[3:6]))
    inlier1 = env.CheckCollision(Ray(ray1[0:3],ray1[3:6]),report1)
    print 'numcontacts: ',len(report1.contacts),' pos: ', report1.contacts[0].pos,' norm: ',report1.contacts[0].norm
except openrave_exception,e:
    print e

robot2 = env.ReadRobotXMLFile('robots/mitsubishi-pa10.zae')
env.Add(robot2)
body1 = env.ReadKinBodyXMLFile('data/mug1.kinbody.xml')
env.Add(body1)

raw_input("Press Enter to continue...")

env.CheckCollision(robot1,robot2)
env.CheckCollision(robot1,body1)
env.CheckCollision(robot2,body1)

print 'checking self collision'
robot1.SetDOFValues([2.98],[3])
if not robot1.CheckSelfCollision():
    print 'no collision detected (bad)!!, ',env.CheckCollision(robot1.GetLinks()[1],robot1.GetLinks()[13])

# handle.close()
# test distance queries
T = eye(4)
T[0,3] = 0.5
robot1.SetTransform(T)
T = eye(4)
T[1,3] = 0.5
body1.SetTransform(T)

raw_input("Press Enter to continue...")

print 'move the robots to update the closest distance'
if not env.GetCollisionChecker().SetCollisionOptions(CollisionOptions.Distance|CollisionOptions.Contacts):
    print 'current checker does not support distance, switching to pqp...'
    collisionChecker = RaveCreateCollisionChecker(env,'pqp')
    collisionChecker.SetCollisionOptions(CollisionOptions.Distance|CollisionOptions.Contacts)
    env.SetCollisionChecker(collisionChecker)
report = CollisionReport()
while True:
    contacts = []
    with env:
        check = env.CheckCollision(robot1, report)
        print 'mindist: ',report.minDistance
        contacts += report.contacts
        check = env.CheckCollision(robot2, report)
        print 'mindist: ',report.minDistance
        contacts += report.contacts
        check = env.CheckCollision(body1, report)
        print 'mindist: ',report.minDistance
        contacts += report.contacts
    handles = [env.drawlinestrip(points=array((c.pos,c.pos-c.depth*c.norm)), linewidth=3.0, colors=array((1,0,0,1))) for c in contacts]
    time.sleep(0.1)