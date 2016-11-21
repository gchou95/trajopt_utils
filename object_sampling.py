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

def sampleFurniture(num, env, minExtents, maxExtents, furniture, names, angles, minExtentsArr, maxExtentsArr):
	# Deal with self-collisions (check)
	# Load upright, with rotations 0, 90, 180, 270 deg (check)
	# minExtents, maxExtents are wrt the floor
	# pdb.set_trace()
	deleted = []
	transforms = []
	for i in range(min(num, len(furniture))):
		env.Load(furniture[i])
		ct = 0
		while ct <= 5:
			ct = ct+1
			flag = False
			theta = random.sample(angles,1)
			rot = rotationMatrix(theta, 'z')
			# x_rand = random.uniform(0.6*minExtents[0], 0.6*maxExtents[0])
			# y_rand = random.uniform(0.6*minExtents[1], 0.6*maxExtents[1])
			z_rand = random.uniform(0.6*minExtents[2], 0.6*maxExtents[2])
			if i > 0:
				xy_temp = selectTranslation(minExtentsArr[i-1:i+1], maxExtentsArr[i-1:i+1])
				xy = xy_temp + transform[0:2, 3]
				print xy
			else:
				xy = [random.uniform(0.6*minExtents[0], 0.6*maxExtents[0]), \
					random.uniform(0.6*minExtents[1], 0.6*maxExtents[1])]
			trans = translationVector(xy[0], xy[1], z_rand)
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

def sampleObjects(numF, num, env, minExtentsArr, maxExtentsArr, transforms, angles):
	# Deal with self-collisions
	# Deal with collisions with furniture
	# for i in range(int(floor(num/len(furniture)))):
	try:
		del_ct = 0
		orig_num = len(env.GetBodies())
		for ii in range(numF):
			for i in range(int(np.floor(num/2))):
				# Create random object
				testnum = random.uniform(0,1)
				if testnum < 0.25:
					# pdb.set_trace()
					body = RaveCreateKinBody(env,'')
					name = 'objects%d'%(ii*int(np.floor(num/2)) + i)
					body.SetName(name) 
					xMin = random.uniform(0, np.abs(minExtentsArr[ii][0] - maxExtentsArr[ii][0])/5)
					yMin = random.uniform(0, np.abs(minExtentsArr[ii][1] - maxExtentsArr[ii][1])/5)
					zMin = random.uniform(0, np.abs(minExtentsArr[ii][2] - maxExtentsArr[ii][2])/5)
					body.InitFromBoxes(np.array([[0,0,0,xMin,yMin,zMin]]), True)
				elif testnum < 0.5:
					# pdb.set_trace()
					body = RaveCreateKinBody(env,'')
					name = 'objects%d'%(ii*int(np.floor(num/2)) + i)
					body.SetName(name) 
					radius = random.uniform(0, min(maxExtentsArr[ii] - minExtentsArr[ii]))/5
					body.InitFromSpheres(np.array([[0,0,0,radius]]),True) 
				elif testnum < 0.75:
					env.Load('/home/viki/trajopt_utils/objects/mug1.kinbody.xml')
					body = env.GetKinBody('mug')
					name = 'mug%d-1'%(ii*int(np.floor(num/2)) + i)
					body.SetName(name)
				else:
					env.Load('/home/viki/trajopt_utils/objects/mug2.kinbody.xml')
					body = env.GetKinBody('mug2')
					name = 'mug%d-2'%(ii*int(np.floor(num/2)) + i)
					body.SetName(name)

				# pdb.set_trace()
				body.GetLinks()[0].SetMass(100) 
				body.GetLinks()[0].SetPrincipalMomentsOfInertia([1,2,3]) 
				body.GetLinks()[0].SetLocalMassFrame([1,0,0,0,1,1,1])
				if testnum < 0.5:
					env.Add(body, True)
				# theta0 = np.arccos((np.trace(transforms[ii][0:3,0:3]) - 1)/2)
				theta = random.sample(angles,1)
				rot = rotationMatrix(theta, 'z')
				# print('theta0 = ')
				# print theta0
				# xyz0 = transforms[ii][0:3, 3]
				# # pdb.set_trace()
				# print('x0 = %d'%xyz0[0])
				# print('y0 = %d'%xyz0[1])
				# print('z0 = %d'%xyz0[2])
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
						# dims.append()
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

def dragToGround(env, objects):
	for ii in range(2):
		for i in range(len(objects)):
			while True:
				oldTransform = objects[i].GetTransform()
				newTransform = oldTransform
				newTransform[2, 3] = oldTransform[2, 3] - 0.001
				objects[i].SetTransform(newTransform)
				print(newTransform)
				if env.CheckCollision(objects[i]):
					objects[i].SetTransform(oldTransform)
					break

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
		limits = np.vstack([bounds.pos()-bounds.extents(), bounds.pos()+bounds.extents()])
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

def objectDensity(alpha, numF):
	# alpha in (0,1)
	numSamplesFurniture = int(np.floor(alpha*numF))
	numSamplesObjects = numSamplesFurniture*5
	return numSamplesFurniture, numSamplesObjects

def objectClutter(beta):
	# beta in (0,1)
	MAX = 10
	temp = np.asarray([0, np.pi/2, np.pi, 3*np.pi/2])
	ct = 0
	for i in range(int(np.floor(beta*MAX))):
		ct = ct + 1
		temp_shifted = temp + 2*np.pi/(temp.shape[0])
		temp = np.sort(np.hstack([temp, (temp + temp_shifted)/2]))
		if temp.shape[0] > beta*2048:
			break
	return temp

def selectTranslation(maxExtents, minExtents):
	# element 1 is the first, 2 is the one to be placed
	origMaxExtents = maxExtents[0]
	origMinExtents = minExtents[0]
	newMaxExtents = maxExtents[1]
	newMinExtents = minExtents[1]
	newSize = newMaxExtents - newMinExtents
	newRadius = 1.05*max(newSize)
	corners = [np.array([origMaxExtents[0],origMaxExtents[1]]) + np.array([newRadius, newRadius]), \
		np.array([origMaxExtents[0],origMinExtents[1]]) + np.array([newRadius, -newRadius]), \
		np.array([origMinExtents[0],origMaxExtents[1]]) + np.array([-newRadius, newRadius]), \
		np.array([origMinExtents[0],origMinExtents[1]]) + np.array([-newRadius, -newRadius])]
	print(corners)
	idx = random.randint(0,3)
	center = corners[idx]
	return center



# def chooseGravityObjects():