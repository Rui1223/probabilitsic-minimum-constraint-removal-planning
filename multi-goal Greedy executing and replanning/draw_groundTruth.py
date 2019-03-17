## This python script will visualize the ground the truth and the solution path from 
## all the solvers.

from __future__ import division
import sys
import IPython
import matplotlib.pyplot as plt
import matplotlib.colors as colors
import matplotlib.cm as cmx
import random
import numpy as np
plt.switch_backend('TKagg')

def cal_co(indx, col, row):
	x = int(indx) % col
	y = row - int(int(indx) / col)
	return [x, y]

if __name__ == "__main__":

	

	## read in my text file
	f = open("./groundTruth.txt", "r")
	# start to count the line
	n_line = 0;
	for line in f:
		line = line.split()
		n_line += 1

		## The first line specifies the size & number of obstacles
		if (n_line == 1):
			row = int(line[0])
			col = int(line[1])
			nobstacles = int(line[3])
			# set colormap
			gist_ncar = cm = plt.get_cmap('gist_ncar')
			cNorm  = colors.Normalize(vmin=0, vmax=1)
			scalarMap = cmx.ScalarMappable(norm=cNorm, cmap=gist_ncar)

			# color_values = [random.uniform(0, 0.9) for i in xrange(500)]
			color_values = np.linspace(0, 0.9, nobstacles)
			##random.shuffle(color_values);
			color_pool = [scalarMap.to_rgba(color_values[ii]) for ii in xrange(nobstacles)]			

			# canvas setting
			fig = plt.figure(num=None, figsize=(row+3, col+3), dpi=120, 
											facecolor='w', edgecolor='k')
			ax = fig.add_subplot(1, 1, 1)
			ax.set_xlim(-2, col+3)
			ax.set_ylim(-2, row+3)
			plt.gca().set_aspect('equal', adjustable='box')

		## the 2nd line specifies the obstacle each label belongs to
		elif (n_line == 2):
			## make this label_belongings a dict
			label_belongings = dict()
			for vv in xrange(0, len(line)-1, 2):
				label_belongings[line[vv]] = int(line[vv+1])

		## 3rd line contains true target, start and true goal
		elif (n_line == 3):
			trueTarget = line[0]
			start = int(line[1])
			trueGoal = int(line[2])
			# plot the start location
			ax.text(cal_co(start,col,row)[0]-0.2, cal_co(start,col,row)[1]+0.05, 
				"start", fontweight='bold', fontsize=8, zorder=3)
			# plot the goal location
			ax.text(cal_co(trueGoal,col,row)[0]-0.2, cal_co(trueGoal,col,row)[1]+0.05, 
				"goal", fontweight='bold', fontsize=8, zorder=3)

		## 4th line contains all the true obstacles
		truePoses = map(int, line)

		else:
			## starting lines storing edges and labels
			v1 = line[0];
			v2 = line[1];
			if len(line) == 3: ## there is a label information
				labels = line[2] # which is a string
				labels = labels.split(',') # now labels is a list of labels

				#plot the label as "beautiful" as possible
				if (cal_co(v1,col,row)[0] == cal_co(v2,col,row)[0]):
					## vertical line
					incr = 0
					for label in labels:
						if (label == ','):
							ax.text(cal_co(v1,col,row)[0]-0.06, 
								(cal_co(v1,col,row)[1]+cal_co(v2,col,row)[1])/2+incr, 
								label, fontsize=8, rotation=90, zorder=2)
						else:
							##only print the label which is a TRUE pose
							if (truePoses[int(label)] == 1)
								ax.text(cal_co(v1,col,row)[0]-0.06, 
									(cal_co(v1,col,row)[1]+cal_co(v2,col,row)[1])/2+incr, 
									label, color=color_pool[label_belongings[label]],
															fontsize=8, rotation=90, zorder=2)
						incr += 0.2
				else:
					## horizontal line
					incr = 0
					for label in labels:
						if (label == ','):
							ax.text(min(cal_co(v1,col,row)[0], cal_co(v2,col,row)[0]) 
								+ 1.0/(len(labels)+1) + incr, 
								cal_co(v1,col,row)[1]+0.01, label, fontsize=8, zorder=2)
						else:
							##only print the label which is a TRUE pose
							if (truePoses[int(label)] == 1)
								ax.text(min(cal_co(v1,col,row)[0], cal_co(v2,col,row)[0]) 
									+ 1.0/(len(labels)+1) + incr, 
									cal_co(v1,col,row)[1]+0.01, label,
									color=color_pool[label_belongings[label]], fontsize=8, zorder=2)
						incr += 0.2	
			
			#plot current edge
			ax.plot([cal_co(v1,col,row)[0], cal_co(v2,col,row)[0]], 
					[cal_co(v1,col,row)[1], cal_co(v2,col,row)[1]], zorder=1, color='lightgray', linestyle='dashed')

	# add text to indicate the true target
	ax.text(col+3-3, row+3-0.5-0.4*0, "density:"+trueTarget, fontsize=10)

	
	## also plot the solution for Greedy Algorithm as an illustration
	################################################################################
	f_greedy = open("./../GreedySearch_solution.txt", "r")
	n_line = 0;
	for line in f_greedy:
		line = line.split()
		n_line += 1
		if (n_line == 1):
			##solution_time = line[0]
			success_optimum = line[1]
			survival_optimum = line[2]
			#goal_optimum = line[3]
			#goal_optimum = int(goal_optimum)
			pose_optimum = line[4]
			#nPaths = int(line[5])
			
		# line 2 stores the labels
		elif (n_line == 2):
			if len(line) == 0:
				labels_optimum = " "
			else:
				labels_optimum = line[0]
		else:
			# we are plotting optimal path
			path = map(int, line)
			if (path):
				counter = 0
				while (counter != (len(path)-1)):
					v1 = path[counter]
					v2 = path[counter+1]
					ax.plot(cal_co(v1,col,row)[0], cal_co(v1,col,row)[1], "c*", zorder=4)
					ax.plot([cal_co(v1,col,row)[0], cal_co(v2,col,row)[0]], 
							[cal_co(v1,col,row)[1], cal_co(v2,col,row)[1]], "c--", zorder=4)
					counter += 1
				ax.plot(cal_co(v2,col,row)[0], cal_co(v2,col,row)[1], "c*", zorder=4)

				# plot labels
				##ax.text(col+3-3, -2+4.0, "G Time:"+solution_time, color="c", fontsize=10)
				ax.text(col+3-3, -2+3.5, "G OptPose:"+pose_optimum, color="c", fontsize=10)
				ax.text(col+3-3, -2+3.0, "G Success:"+success_optimum, color="c", fontsize=10)
				ax.text(col+3-3, -2+2.5, "G Survival:"+survival_optimum, color="c", fontsize=10)
				ax.text(col+3-3, -2+2.0, "G Labels:"+labels_optimum, color="c", fontsize=10)	

