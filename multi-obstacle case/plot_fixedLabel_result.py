## This python script will visualize the statistics showing the performance of fixedLabel
## search in terms of the number of labels/poses

from __future__ import division
import IPython
import sys
import matplotlib.pyplot as plt
plt.switch_backend('TKagg')

if __name__ == '__main__':
	
	file_dir = "./statistics/fixedLabel_performance.txt";

	## write in my first file
	############################################################################
	f = open(file_dir)
	nlabels = []
	time = []
	solution = []

	_counter = 0;

	for line in f:
		_counter +=1
		print "****" + str(_counter) + "****"
		line = line.split()
		nlabels.append(line[0])
		time.append(line[1])

	## Let's plot the figures
	plt.figure(1)
	plt.plot(nlabels, time, 'bs-', label='fixedLabel Search') 

	plt.legend(loc='upper left')
	plt.xlabel("#labels/poses")
	plt.xlim((1, 23))
	plt.ylabel("Computation time(s)")
	# plt.ylim((0, 10))
	plt.yscale('log')

	##############################################################################	

	plt.show()