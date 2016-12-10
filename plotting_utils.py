from __future__ import division
import matplotlib.pyplot as plt
from openravepy import *
import numpy as np
import pdb

# for pr2, indsArms = 

def getStartsAndGoals(trajs):
	starts = []
	goals = []
	for i in range(len(trajs)):
		starts.append(trajs[i][0][0][0])
		goals.append(trajs[i][0][0][-1])
	return starts, goals

def getCSpaceDistances(starts, goals, indsArms, indsWrist):
	dists = []
	distsArms = []
	distsWrist = []
	for idx in range(len(starts)):
		dists.append(np.linalg.norm(goals[idx] - starts[idx]))
		distsArms.append(np.linalg.norm(np.asarray([goals[idx][i] for i in indsArms]) - np.asarray([starts[idx][i] for i in indsArms])))
		distsWrist.append(np.linalg.norm(np.asarray([goals[idx][i] for i in indsWrist]) - np.asarray([starts[idx][i] for i in indsWrist])))
	return dists, distsArms, distsWrist

def getEndEffectorDistances(starts, goals):
	distsEE = []
	tempenv = Environment()
	tempenv.Load("robots/pr2-beta-static.zae")
	robot = tempenv.GetRobot('pr2')
	manip = robot.GetManipulator('rightarm')
	robot.SetActiveDOFs(manip.GetArmIndices())
	for idx in range(len(starts)):
		robot.SetActiveDOFValues(starts[idx])
		startEE = manip.GetEndEffectorTransform()[0:3,3]
		robot.SetActiveDOFValues(goals[idx])
		goalEE = manip.GetEndEffectorTransform()[0:3,3]
		distsEE.append(np.linalg.norm(goalEE - startEE))
	return distsEE

def getSuccessRatios(errors):
	logicals = []
	for idx in errors:
		logicals.append([i != 'error: collision with environment' for i in idx[0]]) # for multi-goal format
		# logicals.append([i != ['error: collision with environment'] for i in idx]) # for single-goal format
	sums = [np.sum(i) for i in logicals]
	if type(errors) == list:
		idx = range(len(errors))
	else:
		idx = range(len(errors.tolist()))
	keep = [i for s, i in zip(sums,idx) if s > 0]
	successRatios = [i/float(len(logicals[0])) for i in sums] # for multi-goal format
	# successRatios = [i/float(len(logicals[0][0].tolist())) for i in sums] # for single goal format
	# [successes[i] for i in keep]
	return keep, successRatios

def plotDistancesVsSuccess(dists, distsArms, distsWrist, successRatios, keep):
	dists = [dists[i] for i in keep]
	distsArms = [distsArms[i] for i in keep]
	distsWrist = [distsWrist[i] for i in keep]
	successRatios = [successRatios[i] for i in keep]
	fig = plt.figure()
	plt.hold()
	# plt.scatter(dists, successRatios, hold=True,color='red')
	arm = plt.scatter(distsArms, successRatios, hold=True,color='green', label='Arm distance')
	wrist = plt.scatter(distsWrist, successRatios, hold=True,color='blue', label='Wrist distance')
	ax = fig.add_subplot(111)
	handles, labels = ax.get_legend_handles_labels()
	ax.set_title('C-Space Distances vs Success Ratio')
	ax.set_xlabel('C-Space Distances')
	ax.set_ylabel('Success Ratio')
	ax.legend()
	# plt.show()

def compareCSpaceWorldSpace(dists, distsEE, successRatios, keep):
	dists = [dists[i] for i in keep]
	distsEE = [distsEE[i] for i in keep]
	successRatios = [successRatios[i] for i in keep]
	fig = plt.figure()
	plt.hold()
	cspace = plt.scatter(dists, successRatios, hold=True, color='green', label='C-Space distance')
	# ee = plt.scatter(distsEE, successRatios, hold=True, color='blue', label='World distance')
	# plt.show()
	ax = fig.add_subplot(111)
	handles, labels = ax.get_legend_handles_labels()
	ax.set_title('Distances vs Success Ratio')
	ax.set_xlabel('Distances')
	ax.set_ylabel('Success Ratio')
	ax.legend()
	# plt.show()


# def plotOverallSuccessRatios(success_start, success_goal, success_all):