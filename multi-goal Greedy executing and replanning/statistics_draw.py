## This python script will visualize the statistics showing the performance of the algorithms
## (#obs, #posesPerObs, ObsDistrVariance)

from __future__ import division
import IPython
import sys
import matplotlib.pyplot as plt
plt.switch_backend('TKagg')

if __name__ == '__main__':
	
	file1_nObs = "./statistics_ExecutionReplanning/nObstacles/nObstacles_performance.txt";
	file2_nPosesPerObs = "./statistics_ExecutionReplanning/nPosesPerObs/nPosesPerObs_performance.txt";
	file3_distrVar = "./statistics_ExecutionReplanning/distrVar/distrVar_performance.txt";

	## write in my first file
	############################################################################
	f = open(file1_nObs)
	nObs = []
	time_g = []
	nReplan_g = []
	pathLength_g = []
	pathUtilityRate_g = []
	time_a = []
	nReplan_a = []
	pathLength_a = []
	pathUtilityRate_a = []
	time_m = []
	nReplan_m = []
	pathLength_m = []
	pathUtilityRate_m = []	

	_counter = 0;

	for line in f:
		_counter +=1
		print "****" + str(_counter) + "****"
		line = line.split()
		nObs.append(line[0])
		time_g.append(line[1])
		nReplan_g.append(line[2])
		pathLength_g.append(line[3])
		pathUtilityRate_g.append(line[4])
		time_a.append(line[5])
		nReplan_a.append(line[6])
		pathLength_a.append(line[7])
		pathUtilityRate_a.append(line[8])
		time_m.append(line[9])
		nReplan_m.append(line[10])
		pathLength_m.append(line[11])
		pathUtilityRate_m.append(line[12])

	# IPython.embed()

	## Let's plot the figures
	plt.figure(1)
	plt.plot(nObs, time_g, 'rs--', label='MaxSucess Greedy')
	plt.plot(nObs, time_a, 'y^--', label="A*")
	plt.plot(nObs, time_m, 'bo--', label="MaxSurvival")
	plt.legend(loc='upper left')
	plt.xlabel("#obstacles")
	plt.xlim((8, 65))
	plt.ylabel("computation time(s)")
	plt.ylim((0, 1))

	plt.figure(2)
	plt.plot(nObs, nReplan_g, 'rs--', label='MaxSuccess Greedy')
	plt.plot(nObs, nReplan_a, 'y^--', label="A*")
	plt.plot(nObs, nReplan_m, 'bo--', label="MaxSurvival") 
	plt.legend(loc='upper left')
	plt.xlabel("#obstacles")
	plt.xlim((8, 65))
	plt.ylabel("#replans")
	plt.ylim((0, 5))

	plt.figure(3)
	plt.plot(nObs, pathLength_g, 'rs--', label='MaxSuccess Greedy')
	plt.plot(nObs, pathLength_a, 'y^--', label="A*")
	plt.plot(nObs, pathLength_m, 'bo--', label="MaxSurvival")  
	plt.legend(loc='upper left')
	plt.xlabel("#obstacles")
	plt.xlim((8, 65))
	plt.ylabel("path length")
	plt.ylim((0,80))

	plt.figure(4)
	plt.plot(nObs, pathUtilityRate_g, 'rs--', label='MaxSuccess Greedy')
	plt.plot(nObs, pathUtilityRate_a, 'y^--', label="A*")
	plt.plot(nObs, pathUtilityRate_m, 'bo--', label="MaxSurvival")  
	plt.legend(loc='lower right')
	plt.xlabel("#obstacles")
	plt.xlim((8, 65))
	plt.ylabel("path utility rate")
	plt.ylim((0.5, 1.2))
	##############################################################################


	## write in my second file
	############################################################################
	f = open(file2_nPosesPerObs)
	nPosesPerObs = []
	time_g = []
	nReplan_g = []
	pathLength_g = []
	pathUtilityRate_g = []
	time_a = []
	nReplan_a = []
	pathLength_a = []
	pathUtilityRate_a = []
	time_m = []
	nReplan_m = []
	pathLength_m = []
	pathUtilityRate_m = []

	_counter = 0;

	for line in f:
		_counter +=1
		print "****" + str(_counter) + "****"
		line = line.split()
		nPosesPerObs.append(line[0])
		time_g.append(line[1])
		nReplan_g.append(line[2])
		pathLength_g.append(line[3])
		pathUtilityRate_g.append(line[4])
		time_a.append(line[5])
		nReplan_a.append(line[6])
		pathLength_a.append(line[7])
		pathUtilityRate_a.append(line[8])
		time_m.append(line[9])
		nReplan_m.append(line[10])
		pathLength_m.append(line[11])
		pathUtilityRate_m.append(line[12])

	## Let's plot the figures
	plt.figure(5)
	plt.plot(nPosesPerObs, time_g, 'rs--', label='MaxSucess Greedy')
	plt.plot(nPosesPerObs, time_a, 'y^--', label="A*")
	plt.plot(nPosesPerObs, time_m, 'bo--', label="MaxSurvival")
	plt.legend(loc='upper left')
	plt.xlabel("#poses per obstacle")
	plt.xlim((1, 10))
	plt.ylabel("computation time(s)")
	plt.ylim((0, 1))

	plt.figure(6)
	plt.plot(nPosesPerObs, nReplan_g, 'rs--', label='MaxSuccess Greedy')
	plt.plot(nPosesPerObs, nReplan_a, 'y^--', label="A*")
	plt.plot(nPosesPerObs, nReplan_m, 'bo--', label="MaxSurvival") 
	plt.legend(loc='upper left')
	plt.xlabel("#poses per obstacle")
	plt.xlim((1, 10))
	plt.ylabel("#replans")
	plt.ylim((0, 5))

	plt.figure(7)
	plt.plot(nPosesPerObs, pathLength_g, 'rs--', label='MaxSuccess Greedy')
	plt.plot(nPosesPerObs, pathLength_a, 'y^--', label="A*")
	plt.plot(nPosesPerObs, pathLength_m, 'bo--', label="MaxSurvival")  
	plt.legend(loc='upper left')
	plt.xlabel("#poses per obstacle")
	plt.xlim((1, 10))
	plt.ylabel("path length")
	plt.ylim((0,80))

	plt.figure(8)
	plt.plot(nPosesPerObs, pathUtilityRate_g, 'rs--', label='MaxSuccess Greedy')
	plt.plot(nPosesPerObs, pathUtilityRate_a, 'y^--', label="A*")
	plt.plot(nPosesPerObs, pathUtilityRate_m, 'bo--', label="MaxSurvival")  
	plt.legend(loc='lower right')
	plt.xlabel("#poses per obstacle")
	plt.xlim((1, 10))
	plt.ylabel("path utility rate")
	plt.ylim((0.5, 1.2))
	##############################################################################

	## write in my third file
	############################################################################
	f = open(file3_distrVar)
	distrVar = []
	time_g = []
	nReplan_g = []
	pathLength_g = []
	pathUtilityRate_g = []
	time_a = []
	nReplan_a = []
	pathLength_a = []
	pathUtilityRate_a = []
	time_m = []
	nReplan_m = []
	pathLength_m = []
	pathUtilityRate_m = []

	_counter = 0;

	for line in f:
		_counter +=1
		print "****" + str(_counter) + "****"
		line = line.split()
		distrVar.append(line[0])
		time_g.append(line[1])
		nReplan_g.append(line[2])
		pathLength_g.append(line[3])
		pathUtilityRate_g.append(line[4])
		time_a.append(line[5])
		nReplan_a.append(line[6])
		pathLength_a.append(line[7])
		pathUtilityRate_a.append(line[8])
		time_m.append(line[9])
		nReplan_m.append(line[10])
		pathLength_m.append(line[11])
		pathUtilityRate_m.append(line[12])

	## Let's plot the figures
	plt.figure(9)
	plt.plot(distrVar, time_g, 'rs--', label='MaxSucess Greedy')
	plt.plot(distrVar, time_a, 'y^--', label="A*")
	plt.plot(distrVar, time_m, 'bo--', label="MaxSurvival")
	plt.legend(loc='upper left')
	plt.xlabel("Variance of obstacle distribution")
	plt.xlim((0.0, 5.0))
	plt.ylabel("computation time(s)")
	plt.ylim((0, 1))

	plt.figure(10)
	plt.plot(distrVar, nReplan_g, 'rs--', label='MaxSuccess Greedy')
	plt.plot(distrVar, nReplan_a, 'y^--', label="A*")
	plt.plot(distrVar, nReplan_m, 'bo--', label="MaxSurvival") 
	plt.legend(loc='upper left')
	plt.xlabel("Variance of obstacle distribution")
	plt.xlim((0.0, 5.0))
	plt.ylabel("#replans")
	plt.ylim((0, 5))

	plt.figure(11)
	plt.plot(distrVar, pathLength_g, 'rs--', label='MaxSuccess Greedy')
	plt.plot(distrVar, pathLength_a, 'y^--', label="A*")
	plt.plot(distrVar, pathLength_m, 'bo--', label="MaxSurvival")  
	plt.legend(loc='upper left')
	plt.xlabel("Variance of obstacle distribution")
	plt.xlim((0.0, 6.0))
	plt.ylabel("path length")
	plt.ylim((0,80))

	plt.figure(12)
	plt.plot(distrVar, pathUtilityRate_g, 'rs--', label='MaxSuccess Greedy')
	plt.plot(distrVar, pathUtilityRate_a, 'y^--', label="A*")
	plt.plot(distrVar, pathUtilityRate_m, 'bo--', label="MaxSurvival")  
	plt.legend(loc='lower right')
	plt.xlabel("Variance of obstacle distribution")
	plt.xlim((0.0, 5.0))
	plt.ylabel("path utility rate")
	plt.ylim((0.5, 1.2))
	##############################################################################



	plt.show()


