import numpy as np
import os, sys
from edges import Edge
from graphs import Graph
from dijkstra import Dijkstra, DijkstraMatrix


# def read_map(file_name):

info = np.zeros(7)
f = open('./CARP_samples/simple.dat')
lines = f.readlines()

# Read map info to numpy array info

for i in range(1, 8):
    info[i-1] = lines[i].strip().split()[-1]
f.close()
info = info.astype(np.int64)
print(info)

VERTICES = info[0]
DEPOT = info[1]
REQUIRED_EDGES = info[2]
NON_REQUIRED_EDGES = info[3]
VEHICLES = info[4]
CAPACITY = info[5]
TOTAL_COST_OF_REQUIRED_EDGES = info[6]

# Build the graph
G = Graph(n=VERTICES, directed=False)

for i in range(9, REQUIRED_EDGES + NON_REQUIRED_EDGES + 9):
    G.add_edge(Edge(int(lines[i].strip().split()[0]),
                    int(lines[i].strip().split()[1]),
                    int(lines[i].strip().split()[2]),
                    int(lines[i].strip().split()[3])))

print(list(G.iternodes()))
print(G.show())
print(sorted((G.degree(v) for v in G.iternodes()), reverse=True))
print(list(v for v in G. iternodes() if G. degree(v) == 1))
#print(sum(edge.cost for edge in G.iteredges()))

#solution = [[(1, 2), (2, 4), (4, 1)], [(1, 4), (4, 3), (3, 1)]]


def calculate_cost(solution):
    cost = 0
    for route in solution:
        for edge in route:
            source, target = edge
            cost += G[source][target].cost
    print(cost)

algorithm = Dijkstra(G)
algorithm.run(1)
print(algorithm.distance[4])

print (algorithm.path(4))


for dead in list(v for v in G. iternodes() if G. degree(v) == 1):
    print(algorithm.path(dead))




'''
for source in G.iternodes():
    for edge in G.iteroutedges(source):
        while edge.demand != 0:
            capacity_left = CAPACITY
            if capacity_left >= edge.cost:
                edge.cost = 0
                capacity_left -= edge.cost
            else:
                edge.cost -= capacity_left
                capacity_left = 0
        print("%s(%s,%s)" % (edge.target, edge.cost, edge.demand))
'''