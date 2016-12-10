from os import listdir
import numpy as np

def getNames(folderpath, exptype):
	# Return extent for all kinbodies in path
	filenames = listdir(folderpath)
	namesFolder = []
	for i in range(len(filenames)):
		if exptype in filenames[i]:
			name = folderpath + '/' + filenames[i]
			namesFolder.append(name)
	return namesFolder

constrainedError = getNames('/home/viki/trajopt_utils/constrained', 'finalerror')
constrainedTraj = getNames('/home/viki/trajopt_utils/constrained', 'finaltraj')

startError = getNames('/home/viki/trajopt_utils/start', 'finalerror')
startTraj = getNames('/home/viki/trajopt_utils/start', 'finaltraj')

goalError = getNames('/home/viki/trajopt_utils/goal', 'finalerror')
goalTraj = getNames('/home/viki/trajopt_utils/goal', 'finaltraj')

constrainedError = []
constrainedTraj = []
for i in range(len(constrainedError)):
	error = np.load(constrainedError[i])
	traj = np.load(constrainedTraj[i])
	constrainedError.extend(error)
	constrainedTraj.extend(traj)

startError = []
startTraj = []
for i in range(len(startError)):
	error = np.load(startError[i])
	traj = np.load(startTraj[i])
	startError.extend(error)
	startTraj.extend(traj)

goalError = []
goalTraj = []
for i in range(len(goalError)):
	error = np.load(goalError[i])
	traj = np.load(goalTraj[i])
	goalError.extend(error)
	goalTraj.extend(traj)


starts, goals = getStartsAndGoals(trajs)
indsArms = range(0, 5)
indsWrist = range(5, 7)
dists, distsArms, distsWrist = getCSpaceDistances(starts, goals, indsArms, indsWrist)
distsEE = getEndEffectorDistances(starts, goals)
keep, successRatios = getSuccessRatios(errors)
plotDistancesVsSuccess(dists, distsArms, distsWrist, successRatios, keep)
compareCSpaceWorldSpace(dists, distsEE, successRatios, keep)