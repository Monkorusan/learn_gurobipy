# 5.1.3  Single Commodify Flow Formulation

import gurobipy as grbpy
import numpy as np
import networkx
import matplotlib.pyplot as plt
from collections import defaultdict
import matplotlib.animation as anime

def scf(n,c):
    model = grbpy.Model("Asymmetric_TSP-Single_Flow_Commodity")
    x,f ={},{}
    for i in range(1,n+1):
        for j in range(1,n+1):
            if i != j:
                x[i,j] = model.addVar(vtype=grbpy.GRB.BINARY)
                if i==1:
                    f[i,j] = model.addVar(ub=n-1, vtype=grbpy.GRB.CONTINUOUS)
                else:
                    f[i,j] = model.addVar(ub=n-2, vtype=grbpy.GRB.CONTINUOUS)
    model.update()
    for i in range(1,n+1):
        model.addConstr(grbpy.quicksum(x[i,j] for j in range(1,n+1) if j!=i)==1)
        model.addConstr(grbpy.quicksum(x[j,i] for j in range(1,n+1) if j!=i)==1)
    model.addConstr(grbpy.quicksum(f[1,j] for j in range(2,n+1))==n-1)
    for i in range(2,n+1):
        model.addConstr(grbpy.quicksum(f[j,i] for j in range(1,n+1) if j!=i)
                        -grbpy.quicksum(f[i,j] for j in range(1,n+1) if j!=i)==1)
    for j in range(2,n+1):
        model.addConstr(f[1,j] <= (n-1)*x[1,j])
        for i in range(2,n+1):
            if i != j :
                model.addConstr(f[i,j] <= (n-2)*x[i,j]) 
    model.setObjective(grbpy.quicksum(c[i-1,j-1]*x[i,j] for (i,j) in x),grbpy.GRB.MINIMIZE)
    model.update()
    model.__data = x,f
    return model

def addcut(edges:list[tuple[int, int]], V:list, model, x)->bool:
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
    tour = [1] #assume that we start from node 1 (in 5_1_1, it's [0])
    current = 1
    prev = -1
    for temp in range(n-1):
        next_nodes = [node for node in adj[current] if node!= prev]
        next_node = next_nodes[0]
        tour.append(next_node)
        prev, current = current, next_node
    return [node - 1 for node in tour]
    # return tour

def solve_tsp(V:list, c:np.ndarray, n:int):
    EPS = 1e-6
    model = scf(n,c)
    x, f = model.__data 
    
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

def main():
    is_animated = True
    np.random.seed(24)

    n = 100
    V:list = list(range(1,n+1))
    E:list[tuple[int, int]] = [(i, j) for i in V for j in V if i < j]
    start_point_x = 20
    start_point_y = 20
    X = [start_point_x] + np.random.randint(1, 40, n-1).tolist()
    Y = [start_point_y] +  np.random.randint(1, 40, n-1).tolist()
    c = calc_dist_matrix(X,Y,round_decimal=3)
    opt_ans, edges = solve_tsp(V,c,n)
    print(f"opt_ans = {opt_ans}")

    tour = extract_tour(edges,len(V))
    tour.append(tour[0])
    tour_x,tour_y = [X[i] for i in tour], [Y[i] for i in tour]
    colors = ['red'] + ['blue'] * (n - 1)

    if is_animated:
        fig,ax = plt.subplots()
        ax.set_title("TSP animated plot")
        ax.scatter(X[0], Y[0], c='red', edgecolors='black', zorder=3)  # Highlight start node
        ax.scatter(X[1:], Y[1:], c='blue', edgecolors='black',)
        line, = ax.plot([],[],color='green') #tuple unpacking
        ani = anime.FuncAnimation(
            fig,
            lambda num: (line.set_data(tour_x[:num+1], tour_y[:num+1]) or (line,)),
            frames=len(tour_x),
            init_func=line.set_data([],[]),
            blit=True,
            interval=100,
            repeat=False)
        plt.show()

    else: #static plot
        plt.figure(1)
        plt.title("TSP static plot")
        plt.scatter(X,Y,c=colors)
        plt.plot(tour_x,tour_y,color='green')
        plt.xlabel('x')
        plt.ylabel('y')
        plt.plot()
        plt.grid()
        plt.show()

if __name__ == "__main__":
    main()