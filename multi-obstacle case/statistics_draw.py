## This python script will visualize the statistics showing the performance of the algorithms
## Greedy vs Exact

from __future__ import division
import IPython
import sys
import matplotlib.pyplot as plt
plt.switch_backend('TKagg')

if __name__ == '__main__':
	
	file_dir3 = "./statistics/exact_greedy_nPoses_performance.txt";


	## write in my third file
	#############################################################################
	f = open(file_dir3)
	nPoses = []
	time_G = []
	solution_G = []
	time_E = []
	solution_E = []

	for line in f:
		line = line.split()
		nPoses.append(line[0])
		time_G.append(line[1])
		solution_G.append(line[2])
		time_E.append(line[3])
		solution_E.append(line[4])

	## Let's plot the figures
	plt.figure(1)
	plt.plot(nPoses, time_G, 'bs-', label='Greedy Search') 
	plt.plot(nPoses, time_E, 'g^-', label='Exact Search')
	plt.legend(loc='upper left')
	plt.xlabel("nPoses")
	plt.xlim((80, 1600))
	plt.ylabel("Computation time(s)")
	plt.ylim((0, 10))

	plt.figure(2)
	plt.plot(nPoses, solution_G, 'bo-', label='Greedy Search') 
	plt.plot(nPoses, solution_E, 'g*-', label='Exact Search')
	plt.legend(loc='upper left')
	plt.xlabel("nPoses")
	plt.xlim((80, 1600))
	plt.ylabel("survivability(0-1)")
	plt.ylim((0, 1.2))
	###########################################################################	


	plt.show()



	# ## write in my first file
	# ############################################################################
	# f = open(file_dir1)
	# labelCoverage = []
	# time_G = []
	# solution_G = []
	# time_F = []
	# solution_F = []
	# # time_Gr = []
	# # solution_Gr = []

	# _counter = 0;

	# for line in f:
	# 	_counter +=1
	# 	print "****" + str(_counter) + "****"
	# 	line = line.split()
	# 	labelCoverage.append(line[0])
	# 	time_G.append(line[1])
	# 	solution_G.append(line[2])
	# 	time_F.append(line[3])
	# 	solution_F.append(line[4])
	# 	# time_Gr.append(line[5])
	# 	# solution_Gr.append(line[6])

	# ## Let's plot the figures
	# plt.figure(1)
	# plt.plot(labelCoverage, time_G, 'bs', label='Greedy Search') 
	# plt.plot(labelCoverage, time_F, 'g^', label='FixedLabel Search')
	# ##plt.plot(labelCoverage, time_Gr, 'ro', label='GrowingTree Search')
	# plt.legend(loc='upper left')
	# plt.xlabel("labelCoverage(%)")
	# plt.xlim((30, 70))
	# plt.ylabel("Computation time(s)")
	# plt.ylim((0, 1))

	# plt.figure(2)
	# plt.plot(labelCoverage, solution_G, 'bs', label='Greedy Search') 
	# plt.plot(labelCoverage, solution_F, 'g^', label='FixedLabel Search')
	# ##plt.plot(labelCoverage, solution_Gr, 'ro', label='GrowingTree Search')
	# plt.legend(loc='upper left')
	# plt.xlabel("labelCoverage(%)")
	# plt.xlim((30, 70))
	# plt.ylabel("survivability(0-1)")
	# plt.ylim((0, 1.2))
	# ##############################################################################	


	# ## write in my second file
	# ##############################################################################
	# f = open(file_dir2)
	# gridSize = []
	# time_G = []
	# solution_G = []
	# time_F = []
	# solution_F = []
	# time_Gr = []
	# solution_Gr = []
	# for line in f:
	# 	line = line.split()
	# 	gridSize.append(line[0])
	# 	time_G.append(line[1])
	# 	solution_G.append(line[2])
	# 	time_F.append(line[3])
	# 	solution_F.append(line[4])
	# 	# time_Gr.append(line[5])
	# 	# solution_Gr.append(line[6])

	# ## Let's plot the figures
	# plt.figure(3)
	# plt.plot(gridSize, time_G, 'bs', label='Greedy Search') 
	# plt.plot(gridSize, time_F, 'g^', label='FixedLabel Search')
	# ##plt.plot(gridSize, time_Gr, 'ro', label='GrowingTree Search')
	# plt.legend(loc='upper left')
	# plt.xlabel("gridSize(n)")
	# plt.xlim((30, 110))
	# plt.ylabel("Computation time(s)")
	# plt.ylim((0, 0.05))

	# plt.figure(4)
	# plt.plot(gridSize, solution_G, 'bs', label='Greedy Search') 
	# plt.plot(gridSize, solution_F, 'g^', label='FixedLabel Search')
	# ##plt.plot(gridSize, solution_Gr, 'ro', label='GrowingTree Search')
	# plt.legend(loc='upper left')
	# plt.xlabel("gridSize(n)")
	# plt.xlim((30, 110))
	# plt.ylabel("survivability(0-1)")
	# plt.ylim((0, 1.2))	
	# #############################################################################




