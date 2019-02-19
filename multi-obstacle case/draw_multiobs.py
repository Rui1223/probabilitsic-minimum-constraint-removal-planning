## This python script will visualize the graph stored in a text file with certain formaat of data

from __future__ import division
import sys
import IPython
import matplotlib.pyplot as plt
plt.switch_backend('TKagg')

def cal_co(indx, col, row):
	x = int(indx) % col
	y = row - int(int(indx) / col)
	return [x, y]

if __name__ == "__main__":

	color_pool = ['green', 'blue', 'purple', 'orange', 'pink', 'olive', 'gray', 'brown', 
																			'yellow', 'black']

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
		elif (n_line == 3):
			## make this label_belongings a dict
			label_belongings = dict()
			for vv in xrange(0, len(line)-1, 2):
				label_belongings[line[vv]] = int(line[vv+1])
			# print label_belongings

		else:
			## The lines starting from the 4th line are the lines storing edges and labels
			if (n_line >= 4):
				v1 = line[0];
				v2 = line[1];
				if len(line) == 3: ## there is a label information
					labels = line[2] # which is a string
					labels = labels.split(',') # now labels is a list of labels
					# if (len(labels)==1):
					# 	labels=list(labels)
					# else:
					# 	IPython.embed()
					# 	if (v1=='171' and v2=='172'):
					# 		## partition the string labels if the length of labels > 1
					# 		labels = list(labels.partition(','))
					# 	IPython.embed()
					# 		##IPython.embed()
					#plot the label as "beautiful" as possible
					if (cal_co(v1,col,row)[0] == cal_co(v2,col,row)[0]):
						## vertical line
						incr = 0
						for label in labels:
							if (label == ','):
								ax.text(cal_co(v1,col,row)[0]-0.06, 
									(cal_co(v1,col,row)[1]+cal_co(v2,col,row)[1])/2+incr, 
									label, fontsize=8, rotation=90)
							else:
								##print label
								ax.text(cal_co(v1,col,row)[0]-0.06, 
									(cal_co(v1,col,row)[1]+cal_co(v2,col,row)[1])/2+incr, 
									label, color=color_pool[label_belongings[label]],
																	 fontsize=8, rotation=90)
							incr += 0.25
					else:
						## horizontal line
						incr = 0
						for label in labels:
							if (label == ','):
								ax.text(min(cal_co(v1,col,row)[0], cal_co(v2,col,row)[0]) 
									+ 1.0/(len(labels)+1) + incr, 
									cal_co(v1,col,row)[1]+0.01, label, fontsize=8)
							else:
								##print label
								ax.text(min(cal_co(v1,col,row)[0], cal_co(v2,col,row)[0]) 
									+ 1.0/(len(labels)+1) + incr, 
									cal_co(v1,col,row)[1]+0.01, label,
										color=color_pool[label_belongings[label]], fontsize=8)
							incr += 0.25					
				
				#plot current edge
				ax.plot([cal_co(v1,col,row)[0], cal_co(v2,col,row)[0]], 
						[cal_co(v1,col,row)[1], cal_co(v2,col,row)[1]], 'k')

	# plot label weights
	for ii in xrange(len(label_weights)):
		ax.text(col+3-3, row+3-0.5-0.4*ii, label_weights[ii], fontsize=10)

	# plot density of graph
	ax.text(col+3-3, row+3-0.5-0.4*len(label_weights), "density:", fontsize=10)
	ax.text(col+3-3, row+3-0.5-0.4*(len(label_weights)+1), density, fontsize=10)

	
	# ##Now plot the solution for FixedLabel Algorithm
	# ##################################################################

	f_fixedLabel = open("./" + sys.argv[1] + "/FixedLabel_solution" + str(n) + ".txt", "r")
	n_line = 0;
	for line in f_fixedLabel:
		line = line.split()
		n_line += 1
		if (n_line == 1):
			start = int(line[0])
			goal = int(line[1])
			solution_time = line[2]
		elif (n_line == 2):
			path = map(int, line)
		elif (n_line == 3):
			solution_survival = line[0]
		else:
			if len(line) == 0:
				solution_labels = " ";
			else:
				solution_labels = line[0]

	# First plot the start and goal location
	ax.text(cal_co(start,col,row)[0]-0.2, cal_co(start,col,row)[1]+0.05, 
		"start", fontweight='bold', fontsize=8)
	ax.text(cal_co(goal,col,row)[0]-0.2, cal_co(goal,col,row)[1]+0.05, 
		"goal", fontweight='bold', fontsize=8)

	#plot the optimal path 
	counter = 0
	while (counter != (len(path)-1)):
		v1 = path[counter]
		v2 = path[counter+1]
		ax.plot(cal_co(v1,col,row)[0], cal_co(v1,col,row)[1], "ro")
		ax.plot([cal_co(v1,col,row)[0], cal_co(v2,col,row)[0]], 
				[cal_co(v1,col,row)[1], cal_co(v2,col,row)[1]], "r")
		counter += 1
	ax.plot(cal_co(v2,col,row)[0], cal_co(v2,col,row)[1], "ro")

	# plot solution labels & weights & time
	ax.text(col+3-3, -2+0.5, "F time:"+solution_time, color="r", fontsize=10)
	ax.text(col+3-3, -2+1, "F survival:"+solution_survival, color="r", fontsize=10)
	ax.text(col+3-3, -2+1.5, "F labels:"+solution_labels, color="r", fontsize=10)



	# ##Now plot the solution for Greedy Algorithm
	# ##################################################################
	f_greedy = open("./" + sys.argv[1] + "/GreedySearch_solution" + str(n) + ".txt", "r")
	n_line = 0;
	for line in f_greedy:
		line = line.split()
		n_line += 1
		if (n_line == 1):
			start = int(line[0])
			goal = int(line[1])
			solution_time = line[2]
		elif (n_line == 2):
			path = map(int, line)
		elif (n_line == 3):
			solution_survival = line[0]
		else:
			if len(line) == 0:
				solution_labels = " "
			else:
				solution_labels = line[0]

	#plot the optimal path
	if (path):
		counter = 0
		while (counter != (len(path)-1)):
			v1 = path[counter]
			v2 = path[counter+1]
			ax.plot(cal_co(v1,col,row)[0], cal_co(v1,col,row)[1], "co")
			ax.plot([cal_co(v1,col,row)[0], cal_co(v2,col,row)[0]], 
					[cal_co(v1,col,row)[1], cal_co(v2,col,row)[1]], "c--")
			counter += 1
		ax.plot(cal_co(v2,col,row)[0], cal_co(v2,col,row)[1], "co")

		# plot solution labels and weights
		ax.text(col+3-3, -2+2.0, "G time:"+solution_time, color="c", fontsize=10)
		ax.text(col+3-3, -2+2.5, "G survival:"+solution_survival, color="c", fontsize=10)
		ax.text(col+3-3, -2+3.0, "G labels:"+solution_labels, color="c", fontsize=10)


	# ##Now plot the solution for GrowingTree Algorithm
	# ##################################################################
	# f_growingTree = open("./" + sys.argv[1] + "/GrowingTree_solution" + str(n) + ".txt", "r")
	# n_line = 0;
	# for line in f_growingTree:
	# 	line = line.split()
	# 	n_line += 1
	# 	if (n_line == 1):
	# 		start = int(line[0])
	# 		goal = int(line[1])
	# 		solution_time = line[2]
	# 	elif (n_line == 2):
	# 		path = map(int, line)
	# 	elif (n_line == 3):
	# 		solution_survival = line[0]
	# 	else:
	# 		if len(line) == 0:
	# 			solution_labels = " ";
	# 		else:
	# 			solution_labels = line[0]

	# #plot the optimal path 
	# counter = 0
	# while (counter != (len(path)-1)):
	# 	v1 = path[counter]
	# 	v2 = path[counter+1]
	# 	ax.plot(cal_co(v1,col,row)[0], cal_co(v1,col,row)[1], "yo")
	# 	ax.plot([cal_co(v1,col,row)[0], cal_co(v2,col,row)[0]], 
	# 			[cal_co(v1,col,row)[1], cal_co(v2,col,row)[1]], "y--")
	# 	counter += 1
	# ax.plot(cal_co(v2,col,row)[0], cal_co(v2,col,row)[1], "yo")

	# # plot solution labels and weights
	# ax.text(col+3-3, -2+3.5, "Gr time:"+solution_time, color="y", fontsize=10)
	# ax.text(col+3-3, -2+4.0, "Gr weight:"+solution_survival, color="y", fontsize=10)
	# ax.text(col+3-3, -2+4.5, "Gr labels:"+solution_labels, color="y", fontsize=10)
	
	plt.show()
