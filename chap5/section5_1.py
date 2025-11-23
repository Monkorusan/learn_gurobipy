import numpy as np
import matplotlib.pyplot as plt
import gurobipy as grbpy
import sys
import networkx
from collections import defaultdict

def addcut(edges:list[tuple[int, int]], V:list, model:grbpy, x)->bool:
    G = networkx.Graph()
    G.add_nodes_from(V)
    for (i,j) in edges:
        G.add_edge(i,j)
    Components = list(networkx.connected_components(G))
    if len(Components)==1:
        return False
    for S in Components:
        model.addConstr(
            grbpy.quicksum(x[i,j] for i in S for j in S if j>i)<=len(S)-1)
    return True


def solve_tsp(V,c):
    x = {}
    model = grbpy.Model("tsp")
    for i in V:
        for j in V:
            if j>i:
                x[i,j] = model.addVar(ub=1)
    model.update()

    for i in V:
        cstr1 = grbpy.quicksum(x[j,i] for j in V if j<i)
        cstr2 = grbpy.quicksum(x[i,j] for j in V if j>i)
        model.addConstr(cstr1 + cstr2 == 2)
    
    model.setObjective(grbpy.quicksum(
        c[i,j]*x[i,j] for i in V for j in V if j>i),grbpy.GRB.MINIMIZE)
    
    EPS = 1.e-6
    while True:
        model.optimize()
        edges = []

        for (i,j) in x:
            if x[i,j].X > EPS:
                edges.append( (i,j) )
        
        if addcut(edges,V,model,x) == False:
            if model.IsMIP:
                break
            for (i,j) in x:
                x[i,j].Vtype = grbpy.GRB.BINARY
            model.update()

    return model.ObjVal, edges


def calc_dist_matrix(X:np.ndarray,Y:np.ndarray,round_decimal:int=0)-> np.ndarray:
    if len(X) != len(Y): raise ValueError
    dist_matrix = np.zeros((len(X),len(Y)))
    for x in range(len(X)):
        for y in range(len(Y)):
            if x == y:
                pass
            else:
                dist = np.sqrt( (X[x]-X[y])**2 + (Y[x]-Y[y])**2 )
                if round_decimal != 0:
                    dist = np.round(dist,round_decimal)
                dist_matrix[x,y] = dist
    return dist_matrix  


def extract_tour(edges, n):
    adj = defaultdict(list) #adjacency dictionary
    for i,j in edges:
        adj[i].append(j)
        adj[j].append(i)
    tour = [0] #assume that we start from node 0
    current = 0
    prev = -1
    for temp in range(n-1):
        next_nodes = [node for node in adj[current] if node!= prev]
        next_node = next_nodes[0]
        tour.append(next_node)
        prev, current = current, next_node
    return tour

def main():
    n = 100

    V:list = list(range(n))
    E:list[tuple[int, int]] = [(i, j) for i in V for j in V if i < j]
    X:list[int] = np.random.randint(1, 40, n).tolist()
    Y:list[int] = np.random.randint(1, 40, n).tolist()
    c = calc_dist_matrix(X,Y,round_decimal=3)
    obj, edges = solve_tsp(V,c)
    tour = extract_tour(edges,len(V))
    tour.append(tour[0])
    tour_x,tour_y = [X[i] for i in tour], [Y[i] for i in tour]
    print(obj)

    plt.figure(1)
    colors = ['red'] + ['blue'] * (n - 1)
    plt.scatter(X,Y,c=colors)
    plt.plot(tour_x,tour_y,color='green')
    plt.plot()
    plt.grid()
    plt.show()


if __name__ == "__main__":
    main()

    



