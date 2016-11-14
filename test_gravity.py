import openravepy as rave
import trajoptpy
import json
import time
import random
import numpy as np

env = rave.Environment()
env.SetViewer('qtcoin')
env.Load("data/hanoi.env.xml")
# env.Load(options.scene)
# if options._physics is None:
    # no physics engine set, so set one
physics = rave.RaveCreatePhysicsEngine(env,'ode')
env.SetPhysicsEngine(physics)
with env:
    env.GetPhysicsEngine().SetGravity([0,0,-9.8])
    env.StopSimulation()
    # env.StartSimulation(timestep=options.timestep)
    starttime = time.time()
while True:
    bodynames = ['data/lego2.kinbody.xml', 'data/lego4.kinbody.xml', 'data/mug1.kinbody.xml']
    numbodies = 0
    if numbodies < 25:
        with env:
            # body = env.ReadKinBodyXMLFile(bodynames[np.ranodm.randint(len(bodynames))])
            body = rave.RaveCreateKinBody(env,'')
            body.SetName('body%d'%numbodies)
            body.InitFromBoxes(np.array([[1,1,1,0.1,0.2,0.3]]),True) 
            body.GetLinks()[0].SetMass(1) 
            body.GetLinks()[0].SetPrincipalMomentsOfInertia([1,2,3]) 
            body.GetLinks()[0].SetLocalMassFrame([1,0,0,0,1,1,1])
            env.AddKinBody(body,True)
            

            numbodies += 1
            
            T = np.eye(4)
            T[0:3,3] = np.array((-0.5,-0.5,2))+0.4*np.random.rand(3)
            body.SetTransform(T)
    body.GetLinks()[0].SetStatic(False)  
    env.GetPhysicsEngine().SetGravity([0,0,-9.8])
    
    time.sleep(0.4)
    simtime = env.GetSimulationTime()*1e-6
    realtime = time.time()-starttime
    print 'sim time: %fs, real time: %fs, diff = %fs'%(simtime,realtime,simtime-realtime)