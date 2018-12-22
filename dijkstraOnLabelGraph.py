"""
This code file executes dijkstra algorithm on a general labeled
graph to find a path which has the highest survivability (each edge 
is assigned with a label which contains the index of obstacle poses 
and its occurrence probability) 
"""

from __future__ import division
from collections import defaultdict
from heapq import *
import numpy as np
import IPython
import random

import matplotlib.pyplot as plt
plt.switch_backend('TKagg')

'''
global variables
'''
N_NODES = 18

class Node:
	def __init__(self, x, y):
		self.x = x
		self.y = y


class Edge:
	def __init__(self, v1, v2, labels):
		self.v1 = v1
		self.v2 = v2
		self.labels = labels
		self.ce = sum(labels.values())
		self.se = 1 - self.ce
		self.cost = -np.log(self.se)


def dijkstra(edges, f, t):
	g = defaultdict(list)
	for edge in edges:
	    g[edge.v1].append((edge.cost,edge.v2))
	    g[edge.v2].append((edge.cost,edge.v1))

	# dist records the min value of each node in heap.
	q, seen, dist = [(0,f,())], set(), {f: 0}

	while q:
	    (cost,v1,path) = heappop(q)

	    if v1 in seen: continue

	    seen.add(v1)
	    path += (v1,)
	    if v1 == t: return (cost, path)

	    for c, v2 in g.get(v1, ()):
	        if v2 in seen: continue

	        # Not every edge will be calculated. The edge which can improve the value of node in heap will be useful.
	        if v2 not in dist or cost+c < dist[v2]:
	            dist[v2] = cost+c
	            heappush(q, (cost+c, v2, path))

	return float("inf")


def visualize(nodes, edges, path):
	'''
	Visualize the graph and the optimal path
	'''

	fig = plt.figure(num=None, figsize=(10, 10), dpi=120, facecolor='w', edgecolor='k')
	ax = fig.add_subplot(1,1,1)
	ax.set_xlim(0,10)
	ax.set_ylim(0,10)

	## visualize the nodes
	for idx, node in nodes.iteritems():
		ax.plot(node.x, node.y, "ko")
		ax.text(node.x+0.05, node.y+0.05, idx)

	## visualize the edges
	for edge in edges:
		ax.plot([nodes[edge.v1].x, nodes[edge.v2].x], [nodes[edge.v1].y, nodes[edge.v2].y], 'k')
		ax.text((nodes[edge.v1].x + nodes[edge.v2].x) / 2, (nodes[edge.v1].y + nodes[edge.v2].y) / 2, str(edge.labels.keys()))

	## visualize the optimal path
	counter = 0
	while ( counter != (len(path)-1) ):
		idx1 = path[counter]
		idx2 = path[counter+1]
		ax.plot(nodes[idx1].x, nodes[idx1].y, "ro")
		ax.plot([nodes[idx1].x, nodes[idx2].x], [nodes[idx1].y, nodes[idx2].y], 'r')
		ax.text((nodes[edge.v1].x + nodes[edge.v2].x) / 2, (nodes[edge.v1].y + nodes[edge.v2].y) / 2, str(edge.labels.keys()))
		counter += 1
	ax.plot(nodes[path[counter]].x, nodes[path[counter]].y, 'ro')

	ax.text(nodes[path[counter]].x-0.05, nodes[path[counter]].y-0.2, "goal")
	ax.text(nodes[path[0]].x-0.05, nodes[path[0]].y-0.2, "start")



	plt.show()



if __name__ == "__main__":
    

	nodes = {}
	nodes["A"] = Node(1, 2)
	nodes["B"] = Node(1, 5)
	nodes["C"] = Node(2, 9)
	nodes["D"] = Node(3, 3)
	nodes["E"] = Node(4, 7)
	nodes["F"] = Node(4, 9)
	nodes["G"] = Node(4, 1)
	nodes["H"] = Node(4, 4)
	nodes["I"] = Node(5, 2)
	nodes["J"] = Node(5, 7)
	nodes["K"] = Node(5, 9)
	nodes["L"] = Node(6, 3)
	nodes["M"] = Node(6, 9)
	nodes["N"] = Node(7, 3)
	nodes["O"] = Node(7, 8)
	nodes["P"] = Node(7, 9)
	nodes["Q"] = Node(8, 4)
	nodes["R"] = Node(9, 7)


	edges = []
	edges.append(Edge("A", "B", {1:0.4}))
	edges.append(Edge("A", "D", {1:0.4, 2:0.2}))
	edges.append(Edge("A", "G", {1:0.4}))
	edges.append(Edge("B", "C", {5:0.1}))
	edges.append(Edge("B", "E", {3:0.1}))
	edges.append(Edge("C", "F", {3:0.1, 5:0.1}))
	edges.append(Edge("D", "E", {1:0.4, 2:0.2, 3:0.1}))
	edges.append(Edge("D", "H", {1:0.4, 2:0.2, 3:0.1}))
	edges.append(Edge("D", "I", {1:0.4, 2:0.2}))
	edges.append(Edge("E", "F", {3:0.1, 5:0.1}))
	edges.append(Edge("E", "J", {1:0.4, 3:0.1}))
	edges.append(Edge("F", "K", {3:0.1, 5:0.1}))
	edges.append(Edge("G", "I", {2:0.2}))
	edges.append(Edge("G", "N", {3:0.1}))
	edges.append(Edge("H", "J", {1:0.4, 2:0.2, 3:0.1}))
	edges.append(Edge("H", "L", {1:0.4, 2:0.2, 3:0.1}))
	edges.append(Edge("I", "L", {2:0.2, 3:0.1}))
	edges.append(Edge("J", "K", {1:0.4, 3:0.1}))
	edges.append(Edge("J", "N", {1:0.4, 3:0.1}))
	edges.append(Edge("K", "M", {1:0.4, 3:0.1, 5:0.1}))
	edges.append(Edge("L", "N", {2:0.2, 3:0.1}))
	edges.append(Edge("M", "N", {3:0.1, 5:0.1}))
	edges.append(Edge("M", "P", {3:0.1, 5:0.1}))
	edges.append(Edge("N", "O", {3:0.1}))
	edges.append(Edge("N", "Q", {3:0.1}))
	edges.append(Edge("O", "P", {5:0.1}))
	edges.append(Edge("O", "R", {4:0.2}))
	edges.append(Edge("Q", "R", {3:0.1, 4:0.2}))


	



	'''
	edges = [                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                           
	("A", "B", 7),
	("A", "D", 5),
	("B", "C", 9),
	("B", "D", 1),
	("B", "E", 2),
	("C", "E", 5),
	("D", "E", 15),
	("D", "F", 6),
	("E", "F", 8),
	("E", "G", 9),
	("F", "G", 11)
	]
	'''


	print "=== Dijkstra ==="
	start = random.choice(list(nodes))
	goal = random.choice(list(nodes))
	while (start == goal):
		goal = random.choice(list(nodes))
	print str(start) + " -> " + str(goal) + ":"
	(cost, path) = dijkstra(edges, start, goal)
	print "cost: "
	print cost
	print "The optimal path: "
	print path
	visualize(nodes, edges, path)