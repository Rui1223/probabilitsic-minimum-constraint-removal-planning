## This python script will visualize the graph stored in a text file with certain formaat of data

from __future__ import division
import IPython
import matplotlib.pyplot as plt
plt.switch_backend('TKagg')

def cal_co(indx, col, row):
	x = int(indx) % col
	y = row - int(int(indx) / col)
	return [x, y]

if __name__ == "__main__":


	## write in my text file
	f = open("graph.txt", "r")
	# start to count the line
	n_line = 0
	for line in f:
		line = line.split()
		n_line +=1

		## The first line specifies the rows and cols of the graph
		if (n_line == 1):
			row = int(line[0])
			col = int(line[1])
			# canvas setting
			fig = plt.figure(num=None, figsize=(row+3, col+3), dpi=120, 
											facecolor='w', edgecolor='k')
			ax = fig.add_subplot(1, 1, 1)
			ax.set_xlim(-2, col+3)
			ax.set_ylim(-2, row+3)
			plt.gca().set_aspect('equal', adjustable='box')

		## The third line specifies the weight for each label
		elif (n_line == 2):
			label_weights = line ## it's a list

		else:
			## The lines starting from the fifth line are the lines storing edges and labels
			if (n_line >= 3):
				v1 = line[0];
				v2 = line[1];
				if len(line) == 3: ## there is a label information
					labels = line[2]
					#plot the label as "beautiful" as possible
					if (cal_co(v1,col,row)[0] == cal_co(v2,col,row)[0]):
						## vertical line
						ax.text(cal_co(v1,col,row)[0]-0.06, 
								(cal_co(v1,col,row)[1]+cal_co(v2,col,row)[1])/2, 
								labels, fontsize=8, rotation=90)
					else:
						## horizontal line
						ax.text(min(cal_co(v1,col,row)[0], cal_co(v2,col,row)[0]) 
							+ 1.0/(len(labels)+1), 
							cal_co(v1,col,row)[1]+0.01, labels, fontsize=8)
				#plot current edge and label
				ax.plot([cal_co(v1,col,row)[0], cal_co(v2,col,row)[0]], 
						[cal_co(v1,col,row)[1], cal_co(v2,col,row)[1]], 'k')

	# plot label weights
	for ii in xrange(len(label_weights)):
		ax.text(col+3-3, row+3-0.5-0.4*ii, label_weights[ii], fontsize=10)

	
	##Now plot the solution for FixedLabel Algorithm
	##################################################################

	f_fixedLabel = open("FixedLabel_solution.txt", "r")
	n_line = 0;
	for line in f_fixedLabel:
		line = line.split()
		n_line += 1
		if (n_line == 1):
			start = int(line[0])
			goal = int(line[1])
		elif (n_line == 2):
			path = map(int, line)
		elif (n_line == 3):
			solution_weight = line[0]
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

	# plot solution labels and weights
	ax.text(col+3-3, -2+1, "F weight:"+solution_weight, color="r", fontsize=10)
	ax.text(col+3-3, -2+1.5, "F labels:"+solution_labels, color="r", fontsize=10)


	##Now plot the solution for Greedy Algorithm
	##################################################################
	f_fixedLabel = open("GreedySearch_solution.txt", "r")
	n_line = 0;
	for line in f_fixedLabel:
		line = line.split()
		n_line += 1
		if (n_line == 1):
			start = int(line[0])
			goal = int(line[1])
		elif (n_line == 2):
			path = map(int, line)
		elif (n_line == 3):
			solution_weight = line[0]
		else:
			if len(line) == 0:
				solution_labels = " ";
			else:
				solution_labels = line[0]

	#plot the optimal path 
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
	ax.text(col+3-3, -2+2.0, "G weight:"+solution_weight, color="c", fontsize=10)
	ax.text(col+3-3, -2+2.5, "G labels:"+solution_labels, color="c", fontsize=10)

	
	plt.show()


		




