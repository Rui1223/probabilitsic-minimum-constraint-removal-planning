## This python script will visualize the statistics showing the performance of the algorithms
## Greedy, FixedLabel, and connectedComponent search

from __future__ import division
import IPython
import sys
import matplotlib.pyplot as plt
plt.switch_backend('TKagg')

if __name__ == '__main__':
	
	file_dir1 = "./" + sys.argv[1] + "/labelCoverage_performance.txt";
	file_dir2 = "./" + sys.argv[1] + "/gridSize_performance.txt";
	file_dir3 = "./" + sys.argv[1] + "/nLabels_performance.txt";

	## write in my first file
	############################################################################
	f = open(file_dir1)
	labelCoverage = []
	time_G = []
	solution_G = []
	time_F = []
	solution_F = []

	_counter = 0;

	for line in f:
		_counter +=1
		print "****" + str(_counter) + "****"
		line = line.split()
		labelCoverage.append(line[0])
		time_G.append(line[1])
		solution_G.append(line[2])
		time_F.append(line[3])
		solution_F.append(line[4])

	## Let's plot the figures
	plt.figure(1)
	plt.plot(labelCoverage, time_G, 'bs', label='Greedy Search') 
	plt.plot(labelCoverage, time_F, 'g^', label='FixedLabel Search')
	plt.legend(loc='upper left')
	plt.xlabel("labelCoverage(%)")
	plt.xlim((10, 60))
	plt.ylabel("Computation time(s)")
	plt.ylim((0, 0.01))

	plt.figure(2)
	plt.plot(labelCoverage, solution_G, 'bs', label='Greedy Search') 
	plt.plot(labelCoverage, solution_F, 'g^', label='FixedLabel Search')
	plt.legend(loc='upper left')
	plt.xlabel("labelCoverage(%)")
	plt.xlim((10, 60))
	plt.ylabel("survivability(0-1)")
	plt.ylim((0, 1.2))
	##############################################################################	


	## write in my second file
	##############################################################################
	f = open(file_dir2)
	gridSize = []
	time_G = []
	solution_G = []
	time_F = []
	solution_F = []
	for line in f:
		line = line.split()
		gridSize.append(line[0])
		time_G.append(line[1])
		solution_G.append(line[2])
		time_F.append(line[3])
		solution_F.append(line[4])

	## Let's plot the figures
	plt.figure(3)
	plt.plot(gridSize, time_G, 'bs', label='Greedy Search') 
	plt.plot(gridSize, time_F, 'g^', label='FixedLabel Search')
	plt.legend(loc='upper left')
	plt.xlabel("gridSize(n)")
	plt.xlim((10, 40))
	plt.ylabel("Computation time(s)")
	plt.ylim((0, 0.01))

	plt.figure(4)
	plt.plot(gridSize, solution_G, 'bs', label='Greedy Search') 
	plt.plot(gridSize, solution_F, 'g^', label='FixedLabel Search')
	plt.legend(loc='upper left')
	plt.xlabel("gridSize(n)")
	plt.xlim((10, 40))
	plt.ylabel("survivability(0-1)")
	plt.ylim((0, 1.2))	
	#############################################################################

	## write in my third file
	#############################################################################
	f = open(file_dir3)
	nLabels = []
	time_G = []
	solution_G = []
	time_F = []
	solution_F = []
	for line in f:
		line = line.split()
		nLabels.append(line[0])
		time_G.append(line[1])
		solution_G.append(line[2])
		time_F.append(line[3])
		solution_F.append(line[4])

	## Let's plot the figures
	plt.figure(5)
	plt.plot(nLabels, time_G, 'bs', label='Greedy Search') 
	plt.plot(nLabels, time_F, 'g^', label='FixedLabel Search')
	plt.legend(loc='upper left')
	plt.xlabel("nLabels")
	plt.xlim((0, 10))
	plt.ylabel("Computation time(s)")
	plt.ylim((0, 0.01))
	plt.figure(6)
	plt.plot(nLabels, solution_G, 'bs', label='Greedy Search') 
	plt.plot(nLabels, solution_F, 'g^', label='FixedLabel Search')
	plt.legend(loc='upper left')
	plt.xlabel("nLabels")
	plt.xlim((0, 10))
	plt.ylabel("survivability(0-1)")
	plt.ylim((0, 1.2))
	###########################################################################	


	plt.show()


