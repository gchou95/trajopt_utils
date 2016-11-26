from openravepy import * 
import numpy, time 
env = Environment() # create openrave environment 
env.SetViewer('qtcoin') # attach viewer (optional) 
env.Load('data/hanoi.env.xml') # load a simple scene 

xml_str = """<environment>
  <!-- ... other definitions ... -->
  <physicsengine type="ode">
    <odeproperties>
      <friction>100000</friction>
      <gravity>0 0 -150</gravity>
      <selfcollision>1</selfcollision>
    </odeproperties>
  </physicsengine>
</environment>"""
fname = "/home/viki/my_trajopt/tmp/temp.xml"
with open(fname,"w") as fh:
    fh.write(xml_str)

env.Load(fname)
# env.GetPhysicsEngine().SetGravity([0,0,0]) 
# env.GetPhysicsEngine().SetFriction(0.5)
raw_input("Press Enter to continue...")
for i in range(0,5):
	with env: 
	    body = RaveCreateKinBody(env,'') 
	    body.SetName('body%d'%i) 
	    body.InitFromBoxes(numpy.array([[-3+i,1,1,0.1,0.2,0.3]]),True) 
	    body.GetLinks()[0].SetMass(1) 
	    body.GetLinks()[0].SetPrincipalMomentsOfInertia([1,2,3]) 
	    body.GetLinks()[0].SetLocalMassFrame([1,0,0,0,1,1,1])

	    env.Add(body,True) 
	body.GetLinks()[0].SetStatic(False)
	# raw_input("Press Enter to continue...")
time.sleep(0.5) # sleep 2 seconds (want to set this to be minimum st no collisions)

# env.GetPhysicsEngine().SetGravity([0,0,-1]) 
time.sleep(1) # sleep 2 seconds 
# env.GetPhysicsEngine().SetGravity([0,0,-9.8]) 
# time.sleep(1)
for body in env.GetBodies():
	body.GetLinks()[0].SetStatic(True)
