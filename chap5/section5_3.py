# 5.3 Capacitated Vehicle Routing Problem
# standard VRP is both capacitated and only allow 1 trip per truck(only refill at the start)

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.markers import MarkerStyle
import matplotlib.animation as anime
import gurobipy as grbpy
import networkx
from collections import defaultdict
from typing import Callable

def vrp(V:list[int], c:np.ndarray, m:int, q:list[int], Q:int)->tuple[grbpy.Model, Callable[[grbpy.Model, int], None]]:
    def vrp_callback(model,where): #type hint is alr abovementioned as a Callable
        if where != grbpy.GRB.Callback.MIPSOL:
            return
        edges = []
        for (i,j) in x:
            if model.cbGetSolution(x[i,j])>0.5:
                if i!=V[0] and j!=V[0]:
                    edges.append((i,j))
        G = networkx.Graph()
        G.add_edges_from(edges)
        Components = list(networkx.connected_components(G))
        for S in Components:
            S_card = len(S)
            q_sum = sum(q[i] for i in S)
            NS = int(np.ceil(float(q_sum)/Q))
            S_edges = [(i,j) for i in S for j in S if i<j and (i,j) in edges]
            if S_card >=3 and (len(S_edges)>=S_card or NS>1):
                model.cbLazy(grbpy.quicksum(x[i,j] for i in S for j in S if j>i)<=S_card-NS)
                print(f"adding cuts for {S_edges}")
        return 

    model = grbpy.Model("Capacitated Vehicle Routing Problem")
    x = {}
    for i in V:
        for j in V:
            if j>i and i ==V[0]:
                x[i,j]=model.addVar(vtype=grbpy.GRB.INTEGER,ub=2)
            elif j>i:
                x[i,j]=model.addVar(vtype=grbpy.GRB.INTEGER,ub=1)
    model.update()
    model.addConstr(grbpy.quicksum(x[V[0],j] for j in V[1:])==2*m)
    for i in V[1:]:
        cstr1 = grbpy.quicksum(x[j,i] for j in V if j<i)
        cstr2 = grbpy.quicksum(x[i,j] for j in V if j>i)
        model.addConstr(cstr1 + cstr2 ==2)
    model.setObjective(grbpy.quicksum(
        c[i,j]*x[i,j] for i in V for j in V if j>i),grbpy.GRB.MINIMIZE)
    model.update()
    model.__data = x
    return model, vrp_callback

def addcut_vrp(edges:list[tuple[int, int]], V:list[int], model:grbpy.Model, x, q:list[int], Q:int)->bool:
    """a generalized subtour elimination constraints for VRP"""
    
    G = networkx.Graph() 
    G.add_nodes_from(V)
    
    for (i,j) in edges:
        G.add_edge(i,j)
    
    Components = list(networkx.connected_components(G))
    
    if len(Components) == 1:
        return False # If only one component, no cuts needed
    
    for S in Components:
        if V[0] in S:
            continue # Skip if depot is in this component (it's the main route)
            
        demand_S = sum(q[i] for i in S) 
        
        
        r_S = np.ceil(demand_S / Q) # minimum vehicles needed
        
        # generalized subtour elimination constraint for VRP
        # Sum of edges in S must be <= |S| - r(S)
        model.addConstr(
            grbpy.quicksum(x[i,j] for i in S for j in S if j>i and (i,j) in x) 
            <= len(S) - r_S
        )
    
    return True


def solve_vrp(V:list, c:np.ndarray, m:int, q:list[int], Q:int)->tuple[float,list[tuple[int,int,int]]]:
    EPS = 1e-6
    model, vrp_callback_func = vrp(V,c,m,q,Q)
    x = model.__data
    
    model.Params.LazyConstraints = 1
    model.optimize(vrp_callback_func)
    
    if model.SolCount == 0:
        raise ValueError("No feasible solution found")
    
    edge_list_with_multiplicity = []
    for (i,j) in x:
        val = int(round(x[i,j].X))
        if val > 0:
            edge_list_with_multiplicity.append((i, j, val))
    
    return model.ObjVal, edge_list_with_multiplicity


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

def extract_vrp_routes(edges_with_mult:list[tuple[int,int,int]], V:list[int], m:int)->list[list[int]]:
    depot = V[0]
    edge_count = defaultdict(int)
    adj = defaultdict(set)
    for (i,j,mult) in edges_with_mult:
        a, b = (i, j) if i < j else (j, i)
        edge_count[(a,b)] += int(mult)
        adj[i].add(j)
        adj[j].add(i)

    def has_unused_depot_edge():
        return any(edge_count[(min(depot, nbr), max(depot, nbr))] > 0 for nbr in adj[depot])

    def consume_edge(u, v):
        a, b = (u, v) if u < v else (v, u)
        if edge_count[(a,b)] <= 0:
            return False
        edge_count[(a,b)] -= 1
        return True

    routes = []

    # While we still can start a new route from depot and haven't reached m routes
    while len(routes) < m and has_unused_depot_edge():
        # pick a depot neighbor with an available edge
        neighbors = [nbr for nbr in adj[depot] if edge_count[(min(depot, nbr), max(depot, nbr))] > 0]
        if not neighbors:
            break
        next_node = neighbors[0]

        # start route and consume depot->next_node
        if not consume_edge(depot, next_node):
            break
        route = [depot]
        prev = depot
        current = next_node

        # Walk until we return to depot
        while True:
            route.append(current)

            # if current is depot (shouldn't be at first step), stop
            if current == depot:
                break

            # choose a neighbor to continue: prefer any neighbor with unused edges excluding prev
            next_candidates = [nbr for nbr in adj[current]
                               if edge_count[(min(current, nbr), max(current, nbr))] > 0 and nbr != prev]

            if next_candidates:
                # go to the first available candidate
                prev, current = current, next_candidates[0]
                consume_edge(prev, current)
                continue

            # no non-prev neighbors with unused edges -> try to return to depot
            if edge_count[(min(current, depot), max(current, depot))] > 0:
                prev, current = current, depot
                consume_edge(prev, current)
                route.append(current)  # append depot
                break

            route.append(depot) #the final destination is always the depot
            break

        if route[-1] != depot:
            route.append(depot) # Ensure route ends with depot

        routes.append(route)

    return routes

def main():
    is_animated = True
    save_video = False
    np.random.seed(1)
    n = 24 # num of destinations
    m = 4  # num of vehicles
    Q = 6  # capacity of each vehicle
    q = [0] + np.random.randint(1, 2, n).tolist()  # Depot has 0 demand, assume all guest only have 1 demand each
    V = list(range(n+1))
    start_point_x = 20
    start_point_y = 20
    X = [start_point_x] + np.random.randint(1, 40, n).tolist()
    Y = [start_point_y] + np.random.randint(1, 40, n).tolist()
    c = calc_dist_matrix(X,Y,round_decimal=3)
    
    print(f"Solving VRP with {n} nodes, {m} vehicles, capacity {Q}")
    print(f"Total demand: {sum(q)}, Average demand per vehicle: {sum(q)/m:.1f}")
    
    opt_ans, edges = solve_vrp(V, c, m, q, Q)
    print(f"Optimal solution: {opt_ans:.2f}")
    print("Edge list (with multiplicity):")
    for (i,j,mult) in edges:
        print(f"  ({i},{j}) x {mult}")
    
    # Extract separate routes for each vehicle
    routes = extract_vrp_routes(edges, V, m)
    print(f"Number of routes found: {len(routes)}")
    for i, route in enumerate(routes):
        route_demand = sum(q[node] for node in route[1:-1])  # Exclude depot
        print(f"Route {i+1}: {route} (demand: {route_demand})")
    # Define colors for different routes
    route_colors = ['green', 'orange', 'purple', 'cyan', 'magenta', 'yellow']
    colors = ['red'] + ['blue'] * (n - 1)
    if is_animated:
        fig, ax = plt.subplots(figsize=(10, 8))
        ax.set_title(f"VRP animated plot ({m} vehicles)")
        ax.scatter(X[0], Y[0], c='red', s=200, edgecolors='black', zorder=5, marker=MarkerStyle('s'), label='Depot')
        ax.scatter(X[1:], Y[1:], c='blue', s=50, edgecolors='black', zorder=3, label='Customers')
        
        # Create lines for each route
        lines = [ax.plot([], [], color=route_colors[i % len(route_colors)], 
                        linewidth=2, label=f'Vehicle {i+1}')[0] 
                for i in range(len(routes))]
        
        ax.legend()
        ax.grid(True, alpha=0.3)
        
        def animate(frame):
            for i, (line, route) in enumerate(zip(lines, routes)):
                nodes_per_frame = len(route) / total_frames  
                num_nodes = min(len(route), int((frame + 1) * nodes_per_frame) + 1)
                
                if num_nodes > 1:
                    route_x = [X[node] for node in route[:num_nodes]]
                    route_y = [Y[node] for node in route[:num_nodes]]
                    line.set_data(route_x, route_y)
            return lines
        
        total_frames = 50
        
        ani = anime.FuncAnimation(
            fig,
            animate,
            frames=total_frames,
            blit=True,
            interval=100,
            repeat=False)
        
        if save_video:
            ani.save('vrp_animation.mp4', writer='ffmpeg', fps=10, dpi=300)
        plt.show()

    else:  # static plot
        plt.figure(figsize=(10, 8))
        plt.title(f"VRP static plot ({m} vehicles)")
        plt.scatter(X[0], Y[0], c='red', s=200, edgecolors='black', zorder=5, marker=MarkerStyle('s'), label='Depot')
        plt.scatter(X[1:], Y[1:], c='blue', s=50, edgecolors='black', zorder=3, label='Customers')
        
        # Plot each route with different color
        for i, route in enumerate(routes):
            route_x = [X[node] for node in route]
            route_y = [Y[node] for node in route]
            plt.plot(route_x, route_y, color=route_colors[i % len(route_colors)], 
                    linewidth=2, marker='o', markersize=4, label=f'Vehicle {i+1}')
        
        plt.xlabel('x')
        plt.ylabel('y')
        plt.legend()
        plt.grid(True, alpha=0.3)
        plt.show()

if __name__ == "__main__":
    main()

