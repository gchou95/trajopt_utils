from openravepy import * 
import numpy as np, time 
import random
from os import listdir
import pdb
import traceback, sys, code

def rotationMatrix(theta, axis):
	if axis == 'x':
		return np.array([[1,0,0],[0,np.cos(theta),-np.sin(theta)],[0,np.sin(theta),np.cos(theta)]])
	elif axis == 'y':
		return np.array([[np.cos(theta),0,np.sin(theta)],[0,1,0],[-np.sin(theta),0,np.cos(theta)]])
	elif axis == 'z':
		return np.array([[np.cos(theta),-np.sin(theta),0],[np.sin(theta),np.cos(theta),0],[0,0,1]])
	else:
		return
def translationVector(x,y,z):
	return np.array([[x],[y],[z]])

def sampleFurniture(num, env, minExtents, maxExtents, furniture, names):
	# Deal with self-collisions (check)
	# Load upright, with rotations 0, 90, 180, 270 deg (check)
	# minExtents, maxExtents are wrt the floor
	deleted = []
	transforms = []
	for i in range(min(num, len(furniture))):
		env.Load(furniture[i])
		ct = 0
		while ct <= 5:
			ct = ct+1
			flag = False
			theta = random.sample([0,np.pi/2,np.pi,3*np.pi/2],1)
			rot = rotationMatrix(theta, 'z')
			x_rand = random.uniform(minExtents[0], maxExtents[0])
			y_rand = random.uniform(minExtents[1], maxExtents[1])
			z_rand = random.uniform(minExtents[2], maxExtents[2])
			trans = translationVector(x_rand, y_rand, z_rand)
			transform = np.vstack([np.hstack([rot, trans]), np.array([0,0,0,1])])
			# pdb.set_trace()
			for k in names[i]:
				# get the names of all kinbodies in an xml
				obj = env.GetKinBody(k)
				# pdb.set_trace()
				obj.SetTransform(transform)
			tempIdx = range(i)
			for idx in range(i):
				if idx in deleted:
					tempIdx.remove(idx)
			# pdb.set_trace()
			for j in tempIdx:
				for ii in range(len(names[i])):
					for jj in range(len(names[j])):
						# pdb.set_trace()
						for k in names[i]:
							# get the names of all kinbodies in an xml
							obj = env.GetKinBody(k)
							obj.SetTransform(transform)
						if env.CheckCollision(env.GetKinBody(names[i][ii]), env.GetKinBody(names[j][jj])):
							flag = True
							break
					if flag:
						break
				if flag:
					break
			if not flag:
				transforms.append(transform)
				break
			if ct == 5:
				# for kk in range(len(names[i])):
				# 	env.Remove(env.GetKinBody(names[i][kk]))
				deleted.append(i)
			# pdb.set_trace()
	for kk in deleted:
		for kkk in range(len(names[kk])):
			env.Remove(env.GetKinBody(names[kk][kkk]))
	return transforms

def sampleObjects(num, env, minExtentsArr, maxExtentsArr, transforms):
	# Deal with self-collisions
	# Deal with collisions with furniture
	# for i in range(int(floor(num/len(furniture)))):
	try:
		del_ct = 0
		orig_num = len(env.GetBodies())
		for ii in range(len(minExtentsArr)):
			for i in range(int(np.floor(num/2))):
				body = RaveCreateKinBody(env,'')
				name = 'objects%d'%(ii*int(np.floor(num/2)) + i)
				body.SetName(name) 
				# Create random object
				if random.uniform(0,1) > 0.5:
					# pdb.set_trace()
					xMin = random.uniform(0, np.abs(minExtentsArr[ii][0] - maxExtentsArr[ii][0])/5)
					yMin = random.uniform(0, np.abs(minExtentsArr[ii][1] - maxExtentsArr[ii][1])/5)
					zMin = random.uniform(0, np.abs(minExtentsArr[ii][2] - maxExtentsArr[ii][2])/5)
					body.InitFromBoxes(np.array([[0,0,0,xMin,yMin,zMin]]), True)
				else:
					# pdb.set_trace()
					body.InitFromSpheres(np.array([[0,0,0,random.uniform(0, \
						min(maxExtentsArr[ii] - minExtentsArr[ii]))]]),True) 
				# pdb.set_trace()
				body.GetLinks()[0].SetMass(1) 
				body.GetLinks()[0].SetPrincipalMomentsOfInertia([1,2,3]) 
				body.GetLinks()[0].SetLocalMassFrame([1,0,0,0,1,1,1])
				env.Add(body, True)
				theta0 = np.arccos((np.trace(transforms[ii][0:3,0:3]) - 1)/2)
				theta = random.sample([0,np.pi/2,np.pi,3*np.pi/2],1)
				rot = rotationMatrix(theta, 'z')
				print('theta0 = ')
				print theta0
				xyz0 = transforms[ii][0:3, 3]
				# pdb.set_trace()
				print('x0 = %d'%xyz0[0])
				print('y0 = %d'%xyz0[1])
				print('z0 = %d'%xyz0[2])
				x_rand = random.uniform(minExtentsArr[ii][0], maxExtentsArr[ii][0])
				y_rand = random.uniform(minExtentsArr[ii][1], maxExtentsArr[ii][1])
				z_rand = random.uniform(minExtentsArr[ii][2], maxExtentsArr[ii][2])
				trans = translationVector(x_rand, y_rand, z_rand)
				transform = np.vstack([np.hstack([rot, trans]), np.array([0,0,0,1])])
				body.SetTransform(np.dot(transforms[ii], transform))
				ct = 0
				while ct <= 5:
					flag = False
					ct = ct + 1
					for jj in range(orig_num, len(env.GetBodies())):
						for j in range(len(env.GetBodies())):
							if env.CheckCollision(env.GetBodies()[jj], env.GetBodies()[j]):
								# pdb.set_trace()
								flag = True
								break
						if flag:
							break
					if not flag:
						break
					if ct == 5:
						env.Remove(env.GetKinBody(name))
						del_ct = del_ct + 1
						break
					theta = random.sample([0,np.pi/2,np.pi,3*np.pi/2],1)
					rot = rotationMatrix(theta, 'z')
					x_rand = random.uniform(minExtentsArr[ii][0], maxExtentsArr[ii][0])
					y_rand = random.uniform(minExtentsArr[ii][1], maxExtentsArr[ii][1])
					z_rand = random.uniform(minExtentsArr[ii][2], maxExtentsArr[ii][2])
					trans = translationVector(x_rand, y_rand, z_rand)
					transform = np.vstack([np.hstack([rot, trans]), np.array([0,0,0,1])])
					body.SetTransform(np.dot(transforms[ii], transform))
						
					# Collision check with all objects in environment
	except:
		type, value, tb = sys.exc_info()
		traceback.print_exc()
		last_frame = lambda tb=tb: last_frame(tb.tb_next) if tb.tb_next else tb
		frame = last_frame().tb_frame
		ns = dict(frame.f_globals)
		ns.update(frame.f_locals)
		code.interact(local=ns)

def getExtent(folderpath, filepath):
	# Return extent for one kinbody
	tempEnv = Environment()
	tempEnv.Reset()
	tempEnv.Load(folderpath + '/' + filepath)
	minExtent = []
	maxExtent = []
	for i in range(len(tempEnv.GetBodies())):
		obj = tempEnv.GetBodies()[i]
		# ComputeAABB not correct if translated, zero out translation.
		tempTransform = obj.GetTransform()
		tempTransform[0:3, 3] = np.zeros(3)
		obj.SetTransform(tempTransform)
		bounds = obj.ComputeAABB()
		limits = np.vstack([bounds.pos(), bounds.extents()])
		minExtent.append(np.amin(limits, 0))
		maxExtent.append(np.amax(limits, 0))
	return np.amin(np.dstack(minExtent), 2), np.amax(np.dstack(maxExtent), 2)

def getExtents(folderpath):
	# Return extent for all kinbodies in path
	filenames = listdir(folderpath)
	minExtents = []
	maxExtents = []
	for i in range(len(filenames)):
		minExtent, maxExtent = getExtent(folderpath, filenames[i])
		minExtents.append(minExtent)
		maxExtents.append(maxExtent)
	minExtents = [j.flatten() for j in minExtents]
	maxExtents = [j.flatten() for j in maxExtents]
	return minExtents, maxExtents

def getName(folderpath, filepath):
	# Return names for each kinbody
	tempEnv = Environment()
	tempEnv.Reset()
	tempEnv.Load(folderpath + '/' + filepath)
	names = []
	for i in range(len(tempEnv.GetBodies())):
		name = tempEnv.GetBodies()[i].GetName()
		names.append(name)
	return names

def getNames(folderpath):
	# Return extent for all kinbodies in path
	filenames = listdir(folderpath)
	namesFolder = []
	for i in range(len(filenames)):
		name = getName(folderpath, filenames[i])
		namesFolder.append(name)
	return namesFolder

env = Environment() # create openrave environment 
env.SetViewer('qtcoin') # attach viewer (optional) 
# env.Load('data/hanoi.env.xml') # load a simple scene 
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
fname = "/home/viki/trajopt_utils/tmp/temp.xml"
with open(fname,"w") as fh:
    fh.write(xml_str)

env.Load(fname)
# env.GetPhysicsEngine().SetGravity([0,0,0]) 
# env.GetPhysicsEngine().SetFriction(0.5)
# raw_input("Press Enter to continue...")

# Create floor
floor = RaveCreateKinBody(env, '')
floorExtents_x = [0,10]
floorExtents_y = [0,10]
floorExtents_z = [0,0.01]
floor.InitFromBoxes(np.array([[x for t in zip(floorExtents_x, floorExtents_y, floorExtents_z) \
	for x in t]]), True)
floor.SetName('floor')
env.Add(floor,True)

num_f = 5
num_o = 5*num_f

# Sample furniture
# furn = []
# pdb.set_trace()
filenames = listdir('/home/viki/trajopt_utils/furniture')
minExtents, maxExtents = getExtents('/home/viki/trajopt_utils/furniture')
furniture = ['/home/viki/trajopt_utils/furniture/' + name for name in filenames]
names = getNames('/home/viki/trajopt_utils/furniture')
# pdb.set_trace()
transforms = sampleFurniture(num_f, env, min(zip(floorExtents_x, floorExtents_y, floorExtents_z)), \
	max(zip(floorExtents_x, floorExtents_y, floorExtents_z)), furniture, names)
# pdb.set_trace()
# Sample objects
sampleObjects(num_o, env, minExtents, maxExtents, transforms)

# for i in range(0,5):
# 	with env: 
# 	    body = RaveCreateKinBody(env,'') 
# 	    body.SetName('body%d'%i) 
# 	    body.InitFromBoxes(np.array([[-3+i,1,1,0.1,0.2,0.3]]),True) 
# 	    body.GetLinks()[0].SetMass(1) 
# 	    body.GetLinks()[0].SetPrincipalMomentsOfInertia([1,2,3]) 
# 	    body.GetLinks()[0].SetLocalMassFrame([1,0,0,0,1,1,1])

# 	    env.Add(body,True) 
# 	body.GetLinks()[0].SetStatic(False)
# 	# raw_input("Press Enter to continue...")
# time.sleep(0.5) # sleep 2 seconds (want to set this to be minimum st no collisions)

# # env.GetPhysicsEngine().SetGravity([0,0,-1]) 
# time.sleep(1) # sleep 2 seconds 
# env.GetPhysicsEngine().SetGravity([0,0,-9.8]) 
# time.sleep(1)