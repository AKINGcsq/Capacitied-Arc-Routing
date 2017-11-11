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
# As follows:

# VERTICES
# DEPOT
# REQUIRED EDGES
# NON-REQUIRED EDGES
# VEHICLES
# CAPACITY
# TOTAL COST OF REQUIRED EDGES

for i in range(1, 8):
    info[i-1] = lines[i].strip().split()[-1]
f.close()
info = info.astype(np.int64)
print(info)

# Build the graph
G = Graph(n=info[0], directed=False)

for i in range(9, info[2] + info[3] + 9):
    G.add_edge(Edge(int(lines[i].strip().split()[0]),
                    int(lines[i].strip().split()[1]),
                    int(lines[i].strip().split()[2]),
                    int(lines[i].strip().split()[3])))

print(list(G.iternodes()))
print(G.show())
print(sorted((G. degree(v) for v in G. iternodes()), reverse=True))
print(list(v for v in G. iternodes() if G. degree(v) == 1))
print(sum(edge.cost for edge in G.iteredges()))

algorithm = DijkstraMatrix(G)
algorithm.run(1)
print(algorithm.distance[4])
print(algorithm.path(4))


