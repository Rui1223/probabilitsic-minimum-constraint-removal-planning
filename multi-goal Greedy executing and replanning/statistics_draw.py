## This python script will visualize the statistics showing the performance of the algorithms
## (#obs, #posesPerObs, entropy)

from __future__ import division
import IPython
import sys
import matplotlib.pyplot as plt
plt.switch_backend('TKagg')

FIGSIZE = (3.45, 1.85)
# FIGSIZE = (2, 1.8)
FONTSIZE = 7
# LABELSIZE = 6
# MARKERSIZE = 4
LINEWIDTH = 1
plt.rcParams["legend.labelspacing"] = 0.2
plt.rcParams["legend.handlelength"] = 1.75
plt.rcParams["legend.handletextpad"] = 0.5
plt.rcParams["legend.columnspacing"] = 0.75

plt.rcParams.update({'figure.autolayout': True})

plt.rcParams['ps.useafm'] = True
plt.rcParams['pdf.use14corefonts'] = True
plt.rcParams['text.usetex'] = True
plt.rcParams['font.family'] = "serif"
plt.rcParams['font.serif'] = "Times"

if __name__ == '__main__':
	
	file1_nObs = "./statistics_ExecutionReplanning/nObstacles/nObstacles_performance.txt";
	file2_nPosesPerObs = "./statistics_ExecutionReplanning/nPosesPerObs/nPosesPerObs_performance.txt";
	file3_entropy = "./statistics_ExecutionReplanning/distrVar/distrVar_performance.txt";

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
	time_ml = []
	nReplan_ml = []
	pathLength_ml = []
	pathUtilityRate_ml = []	

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
		time_ml.append(line[13])
		nReplan_ml.append(line[14])
		pathLength_ml.append(line[15])
		pathUtilityRate_ml.append(line[16])

	# IPython.embed()

	## Let's plot the figures
	plt.figure(1, figsize = FIGSIZE)
	plt.plot(nObs, time_g, 'ro-', label='RPDOG greedy', linewidth = LINEWIDTH, markersize=3)
	plt.plot(nObs, time_m, 'bo-', label="RPDO greedy", linewidth = LINEWIDTH, markersize=3)
	plt.plot(nObs, time_ml, 'go-', label="RPMA", linewidth = LINEWIDTH, markersize=3)
	plt.plot(nObs, time_a, 'yo-', label="Shortest path", linewidth = LINEWIDTH, markersize=3)
	plt.xlim((8, 65))
	plt.ylim((0, 1))
	plt.legend(ncol = 2, loc='upper left', fontsize = FONTSIZE)
	plt.xlabel("Number of obstacles", fontsize = FONTSIZE, labelpad = 0)
	plt.ylabel("Computation time (s)", fontsize = FONTSIZE)
	plt.tick_params(labelsize = FONTSIZE)
	plt.savefig("/home/rui/Documents/19_socs/figures/result-obstacle-time.eps", bbox_inches="tight", pad_inches=0.05)
	plt.cla()

	plt.figure(2, figsize = FIGSIZE)
	plt.plot(nObs, nReplan_g, 'ro-', label='RPDOG greedy', linewidth = LINEWIDTH, markersize=3)
	plt.plot(nObs, nReplan_m, 'bo-', label="RPDO greedy", linewidth = LINEWIDTH, markersize=3)
	plt.plot(nObs, nReplan_ml, 'go-', label="RPMA", linewidth = LINEWIDTH, markersize=3) 
	plt.plot(nObs, nReplan_a, 'yo-', label="Shortest path", linewidth = LINEWIDTH, markersize=3)
	plt.xlim((8, 65))
	plt.ylim((0, 5))
	plt.legend(ncol = 2, loc='upper left', fontsize = FONTSIZE)
	plt.xlabel("Number of obstacles", fontsize = FONTSIZE, labelpad = 0)
	plt.ylabel("Number of replans", fontsize = FONTSIZE)
	plt.tick_params(labelsize = FONTSIZE)
	plt.savefig("/home/rui/Documents/19_socs/figures/result-obstacle-replan.eps", bbox_inches="tight", pad_inches=0.05)
	plt.cla()

	plt.figure(3, figsize = FIGSIZE)
	plt.plot(nObs, pathLength_g, 'ro-', label='RPDOG greedy', linewidth = LINEWIDTH, markersize=3)
	plt.plot(nObs, pathLength_m, 'bo-', label="RPDO greedy", linewidth = LINEWIDTH, markersize=3)
	plt.plot(nObs, pathLength_ml, 'go-', label="RPMA", linewidth = LINEWIDTH, markersize=3)
	plt.plot(nObs, pathLength_a, 'yo-', label="Shortest path", linewidth = LINEWIDTH, markersize=3)
	plt.xlim((8, 65))
	plt.ylim((30,70))
	plt.legend(ncol = 2, loc='upper left', fontsize = FONTSIZE)
	plt.xlabel("Number of obstacles", fontsize = FONTSIZE, labelpad = 0)
	plt.ylabel("Path length", fontsize = FONTSIZE)
	plt.tick_params(labelsize = FONTSIZE)
	plt.savefig("/home/rui/Documents/19_socs/figures/result-obstacle-length.eps", bbox_inches="tight", pad_inches=0.05)
	plt.cla()

	plt.figure(4, figsize = FIGSIZE)
	plt.plot(nObs, pathUtilityRate_g, 'ro-', label='RPDOG greedy', linewidth = LINEWIDTH, markersize=3)
	plt.plot(nObs, pathUtilityRate_m, 'bo-', label="RPDO greedy", linewidth = LINEWIDTH, markersize=3)
	plt.plot(nObs, pathUtilityRate_ml, 'go-', label="RPMA", linewidth = LINEWIDTH, markersize=3)
	plt.plot(nObs, pathUtilityRate_a, 'yo-', label="Shortest path", linewidth = LINEWIDTH, markersize=3)
	plt.xlim((8, 65))
	plt.ylim((0.4, 1.1))
	plt.legend(ncol = 2, loc='lower left', fontsize = FONTSIZE)
	plt.xlabel("Number of obstacles", fontsize = FONTSIZE, labelpad = 0)
	plt.ylabel("Path utility rate", fontsize = FONTSIZE)
	plt.tick_params(labelsize = FONTSIZE)
	plt.savefig("/home/rui/Documents/19_socs/figures/result-obstacle-utility.eps", bbox_inches="tight", pad_inches=0.05)
	plt.cla()

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
	time_ml = []
	nReplan_ml = []
	pathLength_ml = []
	pathUtilityRate_ml = []

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
		time_ml.append(line[13])
		nReplan_ml.append(line[14])
		pathLength_ml.append(line[15])
		pathUtilityRate_ml.append(line[16])

	## Let's plot the figures
	plt.figure(5, figsize = FIGSIZE)
	plt.plot(nPosesPerObs, time_g, 'ro-', label='RPDOG greedy', linewidth = LINEWIDTH, markersize=3)
	plt.plot(nPosesPerObs, time_m, 'bo-', label="RPDO greedy", linewidth = LINEWIDTH, markersize=3)
	plt.plot(nPosesPerObs, time_ml, 'go-', label="RPMA", linewidth = LINEWIDTH, markersize=3)
	plt.plot(nPosesPerObs, time_a, 'yo-', label="Shortest path", linewidth = LINEWIDTH, markersize=3)
	plt.xlim((1, 10))
	plt.ylim((0, 1))
	plt.legend(ncol = 2, loc='upper left', fontsize = FONTSIZE)
	plt.xlabel("Number of poses per obstacle", fontsize = FONTSIZE, labelpad = 0)
	plt.ylabel("Computation time (s)", fontsize = FONTSIZE)
	plt.tick_params(labelsize = FONTSIZE)
	plt.savefig("/home/rui/Documents/19_socs/figures/result-pose-time.eps", bbox_inches="tight", pad_inches=0.05)
	plt.cla()

	plt.figure(6, figsize = FIGSIZE)
	plt.plot(nPosesPerObs, nReplan_g, 'ro-', label='RPDOG greedy', linewidth = LINEWIDTH, markersize=3)
	plt.plot(nPosesPerObs, nReplan_m, 'bo-', label="RPDO greedy", linewidth = LINEWIDTH, markersize=3) 
	plt.plot(nPosesPerObs, nReplan_ml, 'go-', label="RPMA", linewidth = LINEWIDTH, markersize=3)
	plt.plot(nPosesPerObs, nReplan_a, 'yo-', label="Shortest path", linewidth = LINEWIDTH, markersize=3)
	plt.xlim((1, 10))
	plt.ylim((0, 5))
	plt.legend(ncol = 2, loc='upper left', fontsize = FONTSIZE)
	plt.xlabel("Number of poses per obstacle", fontsize = FONTSIZE, labelpad = 0)
	plt.ylabel("Number of replans", fontsize = FONTSIZE)
	plt.tick_params(labelsize = FONTSIZE)
	plt.savefig("/home/rui/Documents/19_socs/figures/result-pose-replan.eps", bbox_inches="tight", pad_inches=0.05)
	plt.cla()


	plt.figure(7, figsize = FIGSIZE)
	plt.plot(nPosesPerObs, pathLength_g, 'ro-', label='RPDOG greedy', linewidth = LINEWIDTH, markersize=3)
	plt.plot(nPosesPerObs, pathLength_m, 'bo-', label="RPDO greedy", linewidth = LINEWIDTH, markersize=3)
	plt.plot(nPosesPerObs, pathLength_ml, 'go-', label="RPMA", linewidth = LINEWIDTH, markersize=3)
	plt.plot(nPosesPerObs, pathLength_a, 'yo-', label="Shortest path", linewidth = LINEWIDTH, markersize=3)
	plt.xlim((1, 10))
	plt.ylim((30,70))
	plt.legend(ncol = 2, loc='upper left', fontsize = FONTSIZE)
	plt.xlabel("Number of poses per obstacle", fontsize = FONTSIZE, labelpad = 0)
	plt.ylabel("Path length", fontsize = FONTSIZE)
	plt.tick_params(labelsize = FONTSIZE)
	plt.savefig("/home/rui/Documents/19_socs/figures/result-pose-length.eps", bbox_inches="tight", pad_inches=0.05)
	plt.cla()


	plt.figure(8, figsize = FIGSIZE)
	plt.plot(nPosesPerObs, pathUtilityRate_g, 'ro-', label='RPDOG greedy', linewidth = LINEWIDTH, markersize=3)
	plt.plot(nPosesPerObs, pathUtilityRate_m, 'bo-', label="RPDO greedy", linewidth = LINEWIDTH, markersize=3)
	plt.plot(nPosesPerObs, pathUtilityRate_ml, 'go-', label="RPMA", linewidth = LINEWIDTH, markersize=3)
	plt.plot(nPosesPerObs, pathUtilityRate_a, 'yo-', label="Shortest path", linewidth = LINEWIDTH, markersize=3)
	plt.xlim((1, 10))
	plt.ylim((0.4, 1.1))
	plt.legend(ncol = 2, loc='lower right', fontsize = FONTSIZE)
	plt.xlabel("Number of poses per obstacle", fontsize = FONTSIZE, labelpad = 0)
	plt.ylabel("Path utility rate", fontsize = FONTSIZE)
	plt.tick_params(labelsize = FONTSIZE)
	plt.savefig("/home/rui/Documents/19_socs/figures/result-pose-utility.eps", bbox_inches="tight", pad_inches=0.05)
	plt.cla()
	##############################################################################

	## write in my third file
	############################################################################
	f = open(file3_entropy)
	entropy = []
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
	time_ml = []
	nReplan_ml = []
	pathLength_ml = []
	pathUtilityRate_ml = []	

	_counter = 0;

	for line in f:
		_counter +=1
		print "****" + str(_counter) + "****"
		line = line.split()
		entropy.append(line[0])
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
		time_ml.append(line[13])
		nReplan_ml.append(line[14])
		pathLength_ml.append(line[15])
		pathUtilityRate_ml.append(line[16])

	## Let's plot the figures
	plt.figure(9, figsize = FIGSIZE)
	plt.plot(entropy, time_g, 'ro-', label='RPDOG greedy', linewidth = LINEWIDTH, markersize=3)
	plt.plot(entropy, time_m, 'bo-', label="RPDO greedy", linewidth = LINEWIDTH, markersize=3)
	plt.plot(entropy, time_ml, 'go-', label="RPMA", linewidth = LINEWIDTH, markersize=3)
	plt.plot(entropy, time_a, 'yo-', label="Shortest path", linewidth = LINEWIDTH, markersize=3)
	plt.xlim((1.62, 1.38))
	plt.ylim((0, 1))
	plt.legend(ncol = 2, loc='upper left', fontsize = FONTSIZE)
	plt.xlabel("Entropy", fontsize = FONTSIZE, labelpad = 0)
	plt.ylabel("Computation time (s)", fontsize = FONTSIZE)
	plt.tick_params(labelsize = FONTSIZE)
	plt.savefig("/home/rui/Documents/19_socs/figures/result-entropy-time.eps", bbox_inches="tight", pad_inches=0.05)
	plt.cla()


	plt.figure(10, figsize = FIGSIZE)
	plt.plot(entropy, nReplan_g, 'ro-', label='RPDOG greedy', linewidth = LINEWIDTH, markersize=3)
	plt.plot(entropy, nReplan_m, 'bo-', label="RPDO greedy", linewidth = LINEWIDTH, markersize=3)
	plt.plot(entropy, nReplan_ml, 'go-', label="RPMA", linewidth = LINEWIDTH, markersize=3)
	plt.plot(entropy, nReplan_a, 'yo-', label="Shortest path", linewidth = LINEWIDTH, markersize=3)
	plt.xlim((1.62, 1.38))
	plt.ylim((0, 5))
	plt.legend(ncol = 2, loc='upper left', fontsize = FONTSIZE)
	plt.xlabel("Entropy", fontsize = FONTSIZE, labelpad = 0)
	plt.ylabel("Number of replans", fontsize = FONTSIZE)
	plt.tick_params(labelsize = FONTSIZE)
	plt.savefig("/home/rui/Documents/19_socs/figures/result-entropy-replan.eps", bbox_inches="tight", pad_inches=0.05)
	plt.cla()


	plt.figure(11, figsize = FIGSIZE)
	plt.plot(entropy, pathLength_g, 'ro-', label='RPDOG greedy', linewidth = LINEWIDTH, markersize=3)
	plt.plot(entropy, pathLength_m, 'bo-', label="RPDO greedy", linewidth = LINEWIDTH, markersize=3) 
	plt.plot(entropy, pathLength_ml, 'go-', label="RPMA", linewidth = LINEWIDTH, markersize=3)
	plt.plot(entropy, pathLength_a, 'yo-', label="Shortest path", linewidth = LINEWIDTH, markersize=3)
	plt.xlim((1.62, 1.38))
	plt.ylim((30,70))
	plt.legend(ncol = 2, loc='upper right', fontsize = FONTSIZE)
	plt.xlabel("Entropy", fontsize = FONTSIZE, labelpad = 0)
	plt.ylabel("Path length", fontsize = FONTSIZE)
	plt.tick_params(labelsize = FONTSIZE)
	plt.savefig("/home/rui/Documents/19_socs/figures/result-entropy-length.eps", bbox_inches="tight", pad_inches=0.05)
	plt.cla()


	plt.figure(12, figsize = FIGSIZE)
	plt.plot(entropy, pathUtilityRate_g, 'ro-', label='RPDOG greedy', linewidth = LINEWIDTH, markersize=3)
	plt.plot(entropy, pathUtilityRate_m, 'bo-', label="RPDO greedy", linewidth = LINEWIDTH, markersize=3)
	plt.plot(entropy, pathUtilityRate_ml, 'go-', label="RPMA", linewidth = LINEWIDTH, markersize=3)
	plt.plot(entropy, pathUtilityRate_a, 'yo-', label="Shortest path", linewidth = LINEWIDTH, markersize=3)
	plt.xlim((1.62, 1.38))
	plt.ylim((0.4, 1.1))
	plt.legend(ncol = 2, loc='lower left', fontsize = FONTSIZE)
	plt.xlabel("Entropy", fontsize = FONTSIZE, labelpad = 0)
	plt.ylabel("Path utility rate", fontsize = FONTSIZE)
	plt.tick_params(labelsize = FONTSIZE)
	plt.savefig("/home/rui/Documents/19_socs/figures/result-entropy-utility.eps", bbox_inches="tight", pad_inches=0.05)
	plt.cla()

	##############################################################################


