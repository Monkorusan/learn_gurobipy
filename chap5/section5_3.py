# 5.3 Capacitated Vehicle Routing Problem

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.markers import MarkerStyle
import matplotlib.animation as anime
import gurobipy as grbpy
import networkx
from collections import defaultdict

def vrp(V,c,m,q,Q):
    def vrp_callback(model,where):
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

#addcut method is not suitable for directed graph like VRP
# def addcut(edges:list[tuple[int, int]], V:list, model, x, demand, capacity)->bool:
#     G = networkx.Graph()
#     G.add_nodes_from(V)
#     for (i,j) in edges:
#         G.add_edge(i,j)
#     Components = list(networkx.connected_components(G))
#     if len(Components)==1:
#         return False
#     for S in Components:
#         model.addConstr(
#             grbpy.quicksum(x[i,j] for i in S for j in S if j>i)<=len(S)-1)
#     return True

def addcut_vrp(edges:list[tuple[int, int]], V:list, model:grbpy.Model, x, q:list[int], Q:int):
    """Add generalized subtour elimination constraints for VRP"""
    
    G = networkx.Graph()  # Use undirected since x[i,j] represents undirected edges
    G.add_nodes_from(V)
    
    for (i,j) in edges:
        G.add_edge(i,j)
    
    Components = list(networkx.connected_components(G))
    
    # If only one component, no cuts needed
    if len(Components) == 1:
        return False
    
    for S in Components:
        # Skip if depot is in this component (it's the main route)
        if V[0] in S:
            continue
            
        # Calculate total demand of this component
        demand_S = sum(q[i] for i in S)
        
        # Calculate minimum vehicles needed
        r_S = np.ceil(demand_S / Q)
        
        # Add generalized subtour elimination constraint
        # Sum of edges in S must be <= |S| - r(S)
        model.addConstr(
            grbpy.quicksum(x[i,j] for i in S for j in S if j>i and (i,j) in x) 
            <= len(S) - r_S
        )
    
    return True


def solve_vrp(V:list, c:np.ndarray, m, q, Q, use_callback=True):
    EPS = 1e-6
    model, vrp_callback_func = vrp(V,c,m,q,Q)
    x = model.__data
    
    if use_callback:
        # Callback approach (automated)
        model.Params.LazyConstraints = 1
        model.optimize(vrp_callback_func)
    else:
        # Manual branch-and-cut approach
        while True:
            model.optimize()
            
            if model.SolCount == 0:
                raise ValueError("No feasible solution found")
            
            edges = []
            for (i,j) in x:
                if x[i,j].X > EPS:
                    edges.append((i,j))
            
            # Add VRP-specific cuts
            if not addcut_vrp(edges, V, model, x, q, Q):
                break
    
    if model.SolCount == 0:
        raise ValueError("No feasible solution found")
    
    edges = [(i,j) for (i,j) in x if x[i,j].X > EPS]
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

def extract_vrp_routes(edges, V, m):
    """Extract m separate routes from VRP solution"""
    depot = V[0]
    
    # Build adjacency list
    adj = defaultdict(list)
    for i,j in edges:
        adj[i].append(j)
        adj[j].append(i)
    
    routes = []
    visited_edges = set()
    
    # Start from depot and follow each route
    for next_node in adj[depot]:
        if (depot, next_node) in visited_edges or (next_node, depot) in visited_edges:
            continue
            
        route = [depot]
        current = next_node
        prev = depot
        
        # Follow the route until returning to depot
        while current != depot:
            route.append(current)
            visited_edges.add((min(prev, current), max(prev, current)))
            
            # Find next node
            next_nodes = [node for node in adj[current] if node != prev]
            if not next_nodes:
                break
            prev, current = current, next_nodes[0]
        
        route.append(depot)  # Close the route
        routes.append(route)
        
        if len(routes) >= m:
            break
    
    return routes

# def solve_vrp(V:list, c:np.ndarray, m, q , Q, n:int):
#     EPS = 1e-6
#     model, vrp_callbackfunc = vrp(V,c,m,q,Q)
#     model.Params.LazyConstraints = 1
#     x = model.__data 
    
#     while True:
#         model.optimize(vrp_callbackfunc)
#         if model.SolCount == 0:
#             raise ValueError("no feasible sol!")
#         edges = []
#         for (i,j) in x:
#             if x[i,j].X > EPS:
#                 edges.append( (i,j) )

#         if addcut_vrp(edges,V,model,x) == False:
#             if model.IsMIP:
#                 break
#             for (i,j) in x:
#                 x[i,j].Vtype = grbpy.GRB.BINARY
#             model.update()

#     return model.ObjVal, edges

def main():
    is_animated = False
    use_callback = True  # Set to False to use manual branch-and-cut
    np.random.seed(24)

    n = 10 # num of destinations
    m = 5  # num of vehicles
    Q = 2  # capacity of each vehicle(gemini)
    q = [0] + np.random.randint(1, 2, n-1).tolist()  # Depot has 0 demand
    V:list = list(range(n))
    E:list[tuple[int, int]] = [(i, j) for i in V for j in V if i < j]
    start_point_x = 20
    start_point_y = 20
    X = [start_point_x] + np.random.randint(1, 40, n-1).tolist()
    Y = [start_point_y] + np.random.randint(1, 40, n-1).tolist()
    c = calc_dist_matrix(X,Y,round_decimal=3)
    
    print(f"Solving VRP with {n} nodes, {m} vehicles, capacity {Q}")
    print(f"Total demand: {sum(q)}, Average demand per vehicle: {sum(q)/m:.1f}")
    
    opt_ans, edges = solve_vrp(V, c, m, q, Q, use_callback=use_callback)
    print(f"Optimal solution: {opt_ans:.2f}")
    
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
                # Calculate how many nodes to show for this route
                nodes_per_frame = len(route) / 50  # Adjust speed
                num_nodes = min(len(route), int((frame + 1) * nodes_per_frame) + 1)
                
                if num_nodes > 1:
                    route_x = [X[node] for node in route[:num_nodes]]
                    route_y = [Y[node] for node in route[:num_nodes]]
                    line.set_data(route_x, route_y)
            return lines
        
        # Calculate total frames needed
        max_route_len = max(len(route) for route in routes)
        total_frames = 50
        
        ani = anime.FuncAnimation(
            fig,
            animate,
            frames=total_frames,
            blit=True,
            interval=100,
            repeat=False)
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

