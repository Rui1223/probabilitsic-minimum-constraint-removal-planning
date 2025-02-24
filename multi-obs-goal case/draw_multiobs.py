## This python script will visualize the graph stored in a text file with certain format of data

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

	# color_pool = ['green', 'blue', 'purple', 'orange', 'pink', 'olive', 'brown', 
	# 																'yellow', 'black']

	n = sys.argv[2]

	## write in my text file
	f = open("./" + sys.argv[1] + "/graph" + str(n) + ".txt", "r")
	# start to count the line
	n_line = 0
	for line in f:
		line = line.split()
		n_line +=1

		## The first line specifies the size & density of the graph
		if (n_line == 1):
			row = int(line[0])
			col = int(line[1])
			density = float(line[2])
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

		## The third line specifies the weight for each label and the obstacle it belongs to
		elif (n_line == 2):
			label_weights = line ## it's a list
			# label_weight_dic = dict()
			# for label_weight in label_weights:
			# 	label_weight = label_weight.split(':')
			# 	label_weight_dic[int(label_weight[0])] = int(label_weight[1])
			# IPython.embed()
		elif (n_line == 3):
			## make this label_belongings a dict
			label_belongings = dict()
			for vv in xrange(0, len(line)-1, 2):
				label_belongings[line[vv]] = int(line[vv+1])


		elif (n_line == 4):
			targetPoses = line
		elif (n_line == 5):
			start = int(line[0]);
			# plot the start location
			ax.text(cal_co(start,col,row)[0]-0.2, cal_co(start,col,row)[1]+0.05, 
				"start", fontweight='bold', fontsize=8, zorder=3)
		elif (n_line == 6):
			# plot the goal location
			goalSet = line;
			for g in goalSet:
				g = int(g);
				ax.text(cal_co(g,col,row)[0]-0.2, cal_co(g,col,row)[1]+0.05, 
					"goal", fontweight='bold', fontsize=8, zorder=3)

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
							##print label
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
							##print label
							ax.text(min(cal_co(v1,col,row)[0], cal_co(v2,col,row)[0]) 
								+ 1.0/(len(labels)+1) + incr, 
								cal_co(v1,col,row)[1]+0.01, label,
									color=color_pool[label_belongings[label]], fontsize=8, zorder=2)
						incr += 0.2	
			
			#plot current edge
			ax.plot([cal_co(v1,col,row)[0], cal_co(v2,col,row)[0]], 
					[cal_co(v1,col,row)[1], cal_co(v2,col,row)[1]], zorder=1, color='lightgray', linestyle='dashed')

	# plot label weights
	for ii in xrange(len(label_weights)):
		ii_str = str(ii)
		ax.text(col+3-3, row+3-0.5-0.4*ii, label_weights[ii], color=color_pool[label_belongings[ii_str]], fontsize=10)

	# plot density of graph
	ax.text(col+3-3, row+3-0.5-0.4*len(label_weights), "density:", fontsize=10)
	ax.text(col+3-3, row+3-0.5-0.4*(len(label_weights)+1), density, fontsize=10)
	ax.text(col+3-3, row+3-0.5-0.4*(len(label_weights)+2), "target pose", fontsize=10)
	ax.text(col+3-3, row+3-0.5-0.4*(len(label_weights)+3), targetPoses, fontsize=10)


	# ##Now plot the solution for Greedy Algorithm
	# ##################################################################
	f_greedy = open("./" + sys.argv[1] + "/GreedySearch_solution" + str(n) + ".txt", "r")
	n_line = 0;
	for line in f_greedy:
		line = line.split()
		n_line += 1
		if (n_line == 1):
			solution_time = line[0]
			success_optimum = line[1]
			survival_optimum = line[2]
			goal_optimum = line[3]
			goal_optimum = int(goal_optimum)
			pose_optimum = line[4]
			nPaths = int(line[5])
		# line 2 stores the labels
		elif (n_line == 2):
			if len(line) == 0:
				labels_optimum = " "
			else:
				labels_optimum = line[0]
		else:
			# start plotting the paths
			if (nPaths != 1):
				# we are plotting suboptimal path
				path = map(int, line)
				if (path):
					counter = 0
					while (counter != (len(path)-1)):
						v1 = path[counter]
						v2 = path[counter+1]
						ax.plot(cal_co(v1,col,row)[0], cal_co(v1,col,row)[1], "co", zorder=3)
						ax.plot([cal_co(v1,col,row)[0], cal_co(v2,col,row)[0]], 
								[cal_co(v1,col,row)[1], cal_co(v2,col,row)[1]], "c--", zorder=3)
						counter += 1
					ax.plot(cal_co(v2,col,row)[0], cal_co(v2,col,row)[1], "co", zorder=3)
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
					ax.text(col+3-3, -2+4.0, "G Time:"+solution_time, color="c", fontsize=10)
					ax.text(col+3-3, -2+3.5, "G OptPose:"+pose_optimum, color="c", fontsize=10)
					ax.text(col+3-3, -2+3.0, "G Success:"+success_optimum, color="c", fontsize=10)
					ax.text(col+3-3, -2+2.5, "G Survival:"+survival_optimum, color="c", fontsize=10)
					ax.text(col+3-3, -2+2.0, "G Labels:"+labels_optimum, color="c", fontsize=10)

					## highlight the goal
					ax.text(cal_co(goal_optimum,col,row)[0]-0.2, cal_co(goal_optimum,col,row)[1]+0.05, 
							"goal", color="fuchsia", fontweight='bold', fontsize=8)

			nPaths -= 1;


	plt.show()

	
	# ##Now plot the solution for FixedLabel Algorithm
	# ##################################################################
	# f_fixedLabel = open("./" + sys.argv[1] + "/FixedLabel_solution" + str(n) + ".txt", "r")
	# n_line = 0;
	# for line in f_fixedLabel:
	# 	line = line.split()
	# 	n_line += 1
	# 	if (n_line == 1):
	# 		solution_time = line[0]
	# 		success_optimum = line[1]
	# 		survival_optimum = line[2]
	# 		goal_optimum = line[3]
	# 		goal_optimum = int(goal_optimum)
	# 		pose_optimum = line[4]
	# 		nPaths = int(line[5])
	# 	# line 2 stores the labels
	# 	elif (n_line == 2):
	# 		if len(line) == 0:
	# 			labels_optimum = " "
	# 		else:
	# 			labels_optimum = line[0]
	# 	else:
	# 		# start plotting the paths
	# 		if (nPaths != 1):
	# 			# we are plotting suboptimal path
	# 			path = map(int, line)
	# 			if (path):
	# 				counter = 0
	# 				while (counter != (len(path)-1)):
	# 					v1 = path[counter]
	# 					v2 = path[counter+1]
	# 					ax.plot(cal_co(v1,col,row)[0], cal_co(v1,col,row)[1], "ro")
	# 					ax.plot([cal_co(v1,col,row)[0], cal_co(v2,col,row)[0]], 
	# 							[cal_co(v1,col,row)[1], cal_co(v2,col,row)[1]], "r--")
	# 					counter += 1
	# 				ax.plot(cal_co(v2,col,row)[0], cal_co(v2,col,row)[1], "ro")
	# 		else:
	# 			# we are plotting optimal path
	# 			path = map(int, line)
	# 			if (path):
	# 				counter = 0
	# 				while (counter != (len(path)-1)):
	# 					v1 = path[counter]
	# 					v2 = path[counter+1]
	# 					ax.plot(cal_co(v1,col,row)[0], cal_co(v1,col,row)[1], "r*")
	# 					ax.plot([cal_co(v1,col,row)[0], cal_co(v2,col,row)[0]], 
	# 							[cal_co(v1,col,row)[1], cal_co(v2,col,row)[1]], "r--")
	# 					counter += 1
	# 				ax.plot(cal_co(v2,col,row)[0], cal_co(v2,col,row)[1], "r*")

	# 				# plot labels
	# 				ax.text(col+3-3, -2+6.5, "F Time:"+solution_time, color="r", fontsize=10)
	# 				ax.text(col+3-3, -2+6.0, "F OptPose:"+pose_optimum, color="r", fontsize=10)
	# 				ax.text(col+3-3, -2+5.5, "F Success:"+success_optimum, color="r", fontsize=10)
	# 				ax.text(col+3-3, -2+5.0, "F Survival:"+survival_optimum, color="r", fontsize=10)
	# 				ax.text(col+3-3, -2+4.5, "F Labels:"+labels_optimum, color="r", fontsize=10)

	# 				## highlight the goal
	# 				ax.text(cal_co(goal_optimum,col,row)[0]-0.2, cal_co(goal_optimum,col,row)[1]+0.05, 
	# 						"goal", color="fuchsia", fontweight='bold', fontsize=8)

	# 		nPaths -= 1;