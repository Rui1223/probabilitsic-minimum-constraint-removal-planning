## This python script will visualize the statistics showing the performance of the algorithms
## (#obs, #posesPerObs, ObsDistrVariance)

from __future__ import division
import IPython
import sys
import matplotlib.pyplot as plt
plt.switch_backend('TKagg')

if __name__ == '__main__':
	
	file1_nObs_g = "./statistics_ExecutionReplanning/nObstacles/nObstacles_performance.txt";
	file2_nPosesPerObs_g = "./statistics_ExecutionReplanning/nPosesPerObs/nPosesPerObs_performance.txt";
	file3_distrVar_g = "./statistics_ExecutionReplanning/distrVar/distrVar_performance.txt";

	## write in my first file
	############################################################################
	f = open(file1_nObs_g)
	nObs = []
	time_g = []
	nReplan_g = []
	pathLength_g = []

	_counter = 0;

	for line in f:
		_counter +=1
		print "****" + str(_counter) + "****"
		line = line.split()
		nObs.append(line[0])
		time_g.append(line[1])
		nReplan_g.append(line[2])
		pathLength_g.append(line[3])

	## Let's plot the figures
	plt.figure(1)
	plt.plot(nObs, time_g, 'bs--', label='Greedy Search')
	plt.legend(loc='upper left')
	plt.xlabel("#obstacles")
	plt.xlim((3, 55))
	plt.ylabel("Computation time(s)")
	plt.ylim((0, 0.03))

	plt.figure(2)
	plt.plot(nObs, nReplan_g, 'bs--', label='Greedy Search') 
	plt.legend(loc='upper left')
	plt.xlabel("#obstacles")
	plt.xlim((3, 55))
	plt.ylabel("#replans")
	plt.ylim((0, 2))

	plt.figure(3)
	plt.plot(nObs, pathLength_g, 'bs--', label='Greedy Search') 
	plt.legend(loc='upper left')
	plt.xlabel("#obstacles")
	plt.xlim((3, 55))
	plt.ylabel("path length")
	plt.ylim((40,70))
	##############################################################################


	## write in my second file
	############################################################################
	f = open(file2_nPosesPerObs_g)
	nPosesPerObs = []
	time_g = []
	nReplan_g = []
	pathLength_g = []

	_counter = 0;

	for line in f:
		_counter +=1
		print "****" + str(_counter) + "****"
		line = line.split()
		nPosesPerObs.append(line[0])
		time_g.append(line[1])
		nReplan_g.append(line[2])
		pathLength_g.append(line[3])

	## Let's plot the figures
	plt.figure(4)
	plt.plot(nPosesPerObs, time_g, 'bs--', label='Greedy Search')
	plt.legend(loc='upper left')
	plt.xlabel("#Poses per obstacle")
	plt.xlim((1, 10))
	plt.ylabel("Computation time(s)")
	plt.ylim((0, 0.1))

	plt.figure(5)
	plt.plot(nPosesPerObs, nReplan_g, 'bs--', label='Greedy Search') 
	plt.legend(loc='upper left')
	plt.xlabel("#Poses per obstacle")
	plt.xlim((1, 10))
	plt.ylabel("#replans")
	plt.ylim((0, 3))

	plt.figure(6)
	plt.plot(nPosesPerObs, pathLength_g, 'bs--', label='Greedy Search') 
	plt.legend(loc='upper left')
	plt.xlabel("#Poses per obstacle")
	plt.xlim((1, 10))
	plt.ylabel("path length")
	plt.ylim((40,70))
	##############################################################################

	## write in my third file
	############################################################################
	f = open(file3_distrVar_g)
	distrVar = []
	time_g = []
	nReplan_g = []
	pathLength_g = []

	_counter = 0;

	for line in f:
		_counter +=1
		print "****" + str(_counter) + "****"
		line = line.split()
		distrVar.append(line[0])
		time_g.append(line[1])
		nReplan_g.append(line[2])
		pathLength_g.append(line[3])

	## Let's plot the figures
	plt.figure(7)
	plt.plot(distrVar, time_g, 'bs--', label='Greedy Search')
	plt.legend(loc='upper left')
	plt.xlabel("#obstacle distribution variance")
	plt.xlim((0, 6))
	plt.ylabel("Computation time(s)")
	plt.ylim((0, 0.1))

	plt.figure(8)
	plt.plot(distrVar, nReplan_g, 'bs--', label='Greedy Search') 
	plt.legend(loc='upper left')
	plt.xlabel("#obstacle distribution variance")
	plt.xlim((0, 6))
	plt.ylabel("#replans")
	plt.ylim((0, 3))

	plt.figure(9)
	plt.plot(distrVar, pathLength_g, 'bs--', label='Greedy Search') 
	plt.legend(loc='upper left')
	plt.xlabel("#obstacle distribution variance")
	plt.xlim((0, 6))
	plt.ylabel("path length")
	plt.ylim((40,70))
	##############################################################################



	plt.show()


