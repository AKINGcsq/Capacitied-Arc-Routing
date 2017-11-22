import numpy as np
import os, sys
from edges import Edge
from graphs import Graph
from dijkstra import Dijkstra
from random import randint
import time
import re

start = time.time()

f = open('./eglese/egl-s1-A.dat')
lines = f.readlines()

VERTICES = int(lines[2].strip().split()[-1])
DEPOT = 1
REQUIRED_EDGES = int(lines[3].strip().split()[-1])
NON_REQUIRED_EDGES = int(lines[4].strip().split()[-1])
VEHICLES = int(lines[5].strip().split()[-1])
CAPACITY = int(lines[6].strip().split()[-1])
TOTAL_COST_OF_REQUIRED_EDGES = int(lines[8].strip().split()[-1])
f.close()

G = Graph(n=VERTICES, directed=False)

for i in range(10, REQUIRED_EDGES + 10):
    _ = re.split('[( , ) coste demanda \r\n]', lines[i])
    _ = filter(lambda x: x != '', _)
    G.add_edge(Edge(int(_[0]), int(_[1]), int(_[2]), int(_[3])))
for i in range(11 + REQUIRED_EDGES, 11 + REQUIRED_EDGES + NON_REQUIRED_EDGES):
    _ = re.split('[( , ) coste \r\n]', lines[i])
    _ = filter(lambda x: x != '', _)
    G.add_edge(Edge(int(_[0]), int(_[1]), int(_[2])))

'''
# def read_map(file_name):
info = np.zeros(7)
f = open('./CARP_samples/egl-s1-A.dat')
lines = f.readlines()

# Read map info to numpy array info

for i in range(1, 8):
    info[i-1] = lines[i].strip().split()[-1]
f.close()
info = info.astype(int)
info = info.tolist()
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
'''

# Initialize a shortest path matrix
shortestPath = np.zeros((VERTICES + 1, VERTICES + 1))
shortestPath = np.int_(shortestPath)
for i in range(1, VERTICES+1):
    algorithm = Dijkstra(G)
    algorithm.run(i)
    for j in range(i, VERTICES+1):
        shortestPath[i][j] = algorithm.distance[j]
shortestPath = np.maximum(shortestPath, shortestPath.transpose())
shortestPath = shortestPath.tolist()

initial = time.time()


def initialization():
    INIT_R = []
    INIT_COST = []
    INIT_LOAD = []

    for i in range(100):
        # Tasks and inverse tasks
        free = list([e.source, e.target] for e in G.iteredges() if e.demand != 0)
        free_inv = list([e.target, e.source] for e in G.iteredges() if e.demand != 0)
        task = free + free_inv
        R = []
        COST = []
        LOAD = []
        while True:
            route = []
            load = 0
            cost = 0
            current = DEPOT
            while True:
                d = np.inf
                for u in task:
                    if shortestPath[current][u[0]] < d:
                        if load + G[u[0]][u[1]].demand <= CAPACITY:
                            d = shortestPath[current][u[0]]
                            current_task = u
                    elif shortestPath[current][u[0]] == d:
                        if load + G[u[0]][u[1]].demand <= CAPACITY:
                            rand = randint(0, 1)
                            if rand == 0:
                                current_task = u
                if d != np.inf:
                    route.append(current_task)
                    task.remove(current_task)
                    task.remove(current_task[::-1])
                    load += G[current_task[0]][current_task[1]].demand
                    cost += shortestPath[current_task[0]][current_task[1]] + d
                    current = current_task[1]
                if not task or d == np.inf:
                    break
            cost += shortestPath[current][DEPOT]
            R.append(route)
            LOAD.append(load)
            COST.append(cost)
            if not task:
                break
        if sum(INIT_COST) == 0 or sum(COST) <= sum(INIT_COST):
            INIT_R = R
            INIT_COST = COST
            INIT_LOAD = LOAD

    return INIT_R, INIT_COST, INIT_LOAD


def initialization1():
    INIT_R = []
    INIT_COST = []
    INIT_LOAD = []

    for i in range(100):
        # Tasks and inverse tasks
        free = list([e.source, e.target] for e in G.iteredges() if e.demand != 0)
        free_inv = list([e.target, e.source] for e in G.iteredges() if e.demand != 0)
        task = free + free_inv
        R = []
        COST = []
        LOAD = []
        while True:
            route = []
            load = 0
            cost = 0
            current = DEPOT
            while True:
                d = np.inf
                candidate = np.inf
                candidates = []
                flag = False
                for u in task:
                    if shortestPath[current][u[0]] < candidate:
                        candidate = shortestPath[current][u[0]]
                        candidates = [u]
                    elif shortestPath[current][u[0]] == candidate:
                        candidates.append(u)
                for u in candidates:
                    if load + G[u[0]][u[1]].demand > CAPACITY:
                        flag = False
                    else:
                        flag = True
                        break
                if flag:
                    for u in task:
                        if shortestPath[current][u[0]] < d:
                            if load + G[u[0]][u[1]].demand <= CAPACITY:
                                d = shortestPath[current][u[0]]
                                current_task = u
                        elif shortestPath[current][u[0]] == d:
                            if load + G[u[0]][u[1]].demand <= CAPACITY:
                                rand = randint(0, 1)
                                if rand == 0:
                                    current_task = u
                    if d != np.inf:
                        route.append(current_task)
                        task.remove(current_task)
                        task.remove(current_task[::-1])
                        load += G[current_task[0]][current_task[1]].demand
                        cost += shortestPath[current_task[0]][current_task[1]] + d
                        current = current_task[1]
                    if not task or d == np.inf:
                        break
                else:
                    break
            cost += shortestPath[current][DEPOT]
            R.append(route)
            LOAD.append(load)
            COST.append(cost)
            if not task:
                break
        if sum(INIT_COST) == 0 or sum(COST) <= sum(INIT_COST):
            INIT_R = R
            INIT_COST = COST
            INIT_LOAD = LOAD

    return INIT_R, INIT_COST, INIT_LOAD


INIT_R, INIT_COST, INIT_LOAD = initialization()
INIT_R1, INIT_COST1, INIT_LOAD1 = initialization1()

print(INIT_COST)
print(INIT_LOAD)
print(len(INIT_COST))
print(sum(INIT_COST))
print(sum(INIT_LOAD))
print(len(INIT_COST1))
print(INIT_COST1)
print(INIT_LOAD1)
print(sum(INIT_COST1))
print(sum(INIT_LOAD1))

initial = time.time() - initial
run_time = time.time() - start
print(initial)
print(run_time)
