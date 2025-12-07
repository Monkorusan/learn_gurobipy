# Chapter 5: Traveling Salesman Problem and Routing Problems

## Table of Contents
1. [Section 5.1.1: Subtour Elimination Formulation](#section-511-subtour-elimination-formulation)
2. [Section 5.1.3: Single Commodity Flow Formulation](#section-513-single-commodity-flow-formulation)
3. [Section 5.3: Capacitated Vehicle Routing Problem](#section-53-capacitated-vehicle-routing-problem)

---

# Section 5.1.1: Subtour Elimination Formulation

## Mathematical Background

### Graph Notation
- Given an undirected graph $G=(V,E)$, where:
  - $V$ = set of nodes (cities)
  - $E$ = set of edges (connections between cities)
  - $x_e$ = binary decision variable where $e \in E$ (1 if edge is selected, 0 otherwise)

### Key Definitions
- **E(S)**: Set of edges with both endpoints in subset $S \subseteq V$
  - Formally: $E(S) = \{(i,j) \in E : i \in S, j \in S\}$
  
- **δ(S)**: Set of edges with exactly one endpoint in $S$ (cut-set)
  - Formally: $\delta(S) = \{(i,j) \in E : i \in S, j \notin S\}$ or vice versa
  - This represents the boundary/interface between subset $S$ and its complement

### TSP Requirements
To form a valid TSP tour, we need:

1. **Degree Constraint** (star1): Each node must be connected to exactly 2 edges
   - This ensures we can enter and exit each city exactly once

2. **Subtour Elimination** (star2): No subset of nodes should form a closed loop by themselves
   - For any subset $S$ with $2 \leq |S| < |V|$, the number of edges in $E(S)$ must be $\leq |S|-1$
   - This prevents solutions like having two separate tours: (0→1→2→0) and (3→4→5→3)

## Mathematical Formulation (SEC - Subtour Elimination Constraints)

$$
\begin{align}
\min \quad & \sum_{e \in E} c_e \cdot x_e \\
\text{s.t.} \quad & \sum_{e \in \delta(\{i\})} x_e = 2 && \forall i \in V && \text{(degree constraint)} \\
& \sum_{e \in E(S)} x_e \leq |S| - 1 && \forall S \subseteq V, 2 \leq |S| \leq |V|-2 && \text{(subtour elimination)} \\
& x_e \in \{0,1\} && \forall e \in E
\end{align}
$$

### Constraint Explanation

1. **Objective Function**: Minimize total distance traveled
   - $c_e$ is the distance/cost of edge $e$

2. **Degree Constraint**: $\sum_{e \in \delta(\{i\})} x_e = 2$
   - For each node $i$, exactly 2 edges incident to $i$ must be selected
   - Ensures each city is visited exactly once

3. **Subtour Elimination**: $\sum_{e \in E(S)} x_e \leq |S| - 1$
   - For any subset $S$ of nodes, the edges within $S$ cannot form a complete cycle
   - A cycle in $S$ would have $|S|$ edges, but we limit it to $|S|-1$
   - This forces the tour to be connected to the rest of the graph

### Alternative: Cutset Constraint

Instead of using both degree and subtour elimination constraints, we can use:

$$
\sum_{e \in \delta(S)} x_e \geq 2 \quad \forall S \subseteq V, 2 \leq |S| \leq |V|-2
$$

This single constraint ensures:
- At least 2 edges cross the boundary of any subset $S$
- Combines the effects of both degree constraint and subtour elimination
- More elegant but computationally similar

---

## Code Implementation

### Overview
The implementation uses the **Cutting Plane Method** - a technique where we start with a relaxed problem and iteratively add violated constraints.

### Method 1: `addcut(edges, V, model, x)` - Subtour Detection & Elimination

```python
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
```

**Purpose**: Detects if the current solution contains subtours and adds constraints to eliminate them.

**Step-by-step explanation**:
1. **Create graph from edges**: Build a NetworkX graph with the selected edges
2. **Find connected components**: Use `connected_components()` to identify separate tours
   - If there's only 1 component → valid tour, return `False`
   - If multiple components exist → subtours detected
3. **Add elimination constraints**: For each disconnected component $S$:
   - Add constraint: $\sum_{i,j \in S, j>i} x_{i,j} \leq |S| - 1$
   - This prevents that subset from forming a closed loop
4. **Return True**: Indicates new constraints were added

**Key insight**: We only add constraints for violated subtours, not all possible subsets (exponentially many!)

### Method 2: `solve_tsp(V, c, use_callback)` - Main TSP Solver

```python
def solve_tsp(V:list, c:np.ndarray, use_callback=False):
    x = {}
    EPS = 1e-6
    model = grbpy.Model("TSP-Subtour_Elimination")
    
    # Create binary variables for each edge
    for i in V:
        for j in V:
            if j>i:
                x[i,j] = model.addVar(ub=1,vtype=grbpy.GRB.BINARY)
    model.update()

    # Add degree constraints
    for i in V:
        cstr1 = grbpy.quicksum(x[j,i] for j in V if j<i)
        cstr2 = grbpy.quicksum(x[i,j] for j in V if j>i)
        model.addConstr(cstr1 + cstr2 == 2)
    
    # Set objective
    model.setObjective(grbpy.quicksum(
        c[i,j]*x[i,j] for i in V for j in V if j>i),grbpy.GRB.MINIMIZE)
```

**Initial Setup**:
1. **Decision variables**: Create $x_{i,j}$ for each edge $(i,j)$ where $j>i$
   - We only create variables for $j>i$ to avoid duplicate edges in undirected graph
   - E.g., edge (1,3) is represented by $x_{1,3}$ not both $x_{1,3}$ and $x_{3,1}$

2. **Degree constraints**: For each node $i$:
   - Sum edges where $i$ is the smaller index: $\sum_{j<i} x_{j,i}$
   - Sum edges where $i$ is the larger index: $\sum_{j>i} x_{i,j}$
   - Total must equal 2 (enter once, exit once)

3. **Objective**: Minimize total distance

### Cutting Plane Loop (Non-Callback Version)

```python
    if use_callback:
        # ... callback implementation ...
    else:
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
```

**Cutting Plane Algorithm**:
1. **Solve relaxed problem**: Initially solve without subtour elimination constraints
2. **Extract selected edges**: Get edges where $x_{i,j} > \epsilon$ (accounting for numerical precision)
3. **Check for subtours**: Call `addcut()`
   - If subtours found → add constraints and repeat
   - If no subtours → solution is valid
4. **MIP conversion**: If still solving LP relaxation, convert to MIP and resolve
5. **Return solution**: Optimal distance and tour edges

**Why this works**:
- Start with few constraints → faster initial solve
- Only add constraints as needed → avoid exponential constraint set
- Iteratively refine until valid tour found

### Callback Version (Lazy Constraints)

```python
    if use_callback:
        model.Params.DualReductions = 0
        model.Params.LazyConstraints = 1
        def tsp_callback(model,where):
            if where != grbpy.GRB.Callback.MIPSOL:
                return
            edges=[]
            for (i,j) in x:
                if model.cbGetSolution(x[i,j])>EPS:
                    edges.append((i,j))
            G = networkx.Graph()
            G.add_edges_from(edges)
            Components = list(networkx.connected_components(G))
            if len(Components) == 1:
                return
            for S in Components:
                model.cbLazy(grbpy.quicksum(x[i,j] for i in S for j in S if j>i)<=len(S)-1)
            return
        model.optimize(tsp_callback)
        edges = [(i,j) for (i,j) in x if x[i,j].X > EPS]
        return model.ObjVal, edges
```

**Lazy Constraint Approach**:
- **When triggered**: `GRB.Callback.MIPSOL` - when an integer solution is found
- **How it works**: Gurobi internally manages constraint addition during branch-and-bound
- **Advantages**: More efficient than manual cutting plane for many problems
- **Note**: Code comment suggests textbook version may have issues

### Method 3: `calc_dist_matrix(X, Y, round_decimal)` - Distance Calculation

```python
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
```

**Purpose**: Computes Euclidean distance matrix between all node pairs

**Implementation**:
- Input: X coordinates, Y coordinates
- Output: $n \times n$ matrix where $c_{i,j} = \sqrt{(x_i-x_j)^2 + (y_i-y_j)^2}$
- Optional rounding for numerical stability

### Method 4: `extract_tour(edges, n)` - Tour Reconstruction

```python
def extract_tour(edges, n):
    adj = defaultdict(list)
    for i,j in edges:
        adj[i].append(j)
        adj[j].append(i)
    tour = [0]
    current = 0
    prev = -1
    for temp in range(n-1):
        next_nodes = [node for node in adj[current] if node!= prev]
        next_node = next_nodes[0]
        tour.append(next_node)
        prev, current = current, next_node
    return tour
```

**Purpose**: Converts edge list to ordered tour sequence

**Algorithm**:
1. Build adjacency list from edges
2. Start at node 0
3. At each step, move to a neighbor (not the previous node)
4. Continue for $n-1$ steps to visit all nodes
5. Return ordered sequence

**Example**: 
- Edges: {(0,1), (1,2), (2,0)} 
- Tour: [0, 1, 2]

---

# Section 5.1.3: Single Commodity Flow Formulation

## Mathematical Background

### The Flow-Based Approach

The Single Commodity Flow (SCF) formulation uses a **flow variable** in addition to edge selection variables to prevent subtours. This is particularly elegant for **asymmetric TSP** where distances may differ in each direction.

**Key Idea**: Imagine node 0 (depot) produces $n-1$ units of a commodity, and each other node must consume exactly 1 unit. The flow must follow the selected tour edges.

### Decision Variables

1. **$x_{i,j}$**: Binary variable, 1 if we travel from $i$ to $j$, 0 otherwise
   - Note: For asymmetric TSP, we have both $x_{i,j}$ AND $x_{j,i}$ as separate variables
   
2. **$f_{i,j}$**: Continuous flow variable representing units of commodity flowing from $i$ to $j$
   - Flow originates from node 0 (depot)
   - Each non-depot node consumes 1 unit
   - Bounds:
     - $0 \leq f_{0,j} \leq n-1$ (depot can send up to $n-1$ units)
     - $0 \leq f_{i,j} \leq n-2$ for $i \neq 0$ (other nodes forward at most $n-2$ units)

## Mathematical Formulation (SCF)

$$
\begin{align}
\min \quad & \sum_{i \in V} \sum_{j \in V, j \neq i} c_{i,j} \cdot x_{i,j} \\
\text{s.t.} \quad & \sum_{j \in V, j \neq i} x_{i,j} = 1 && \forall i \in V && \text{(leave each node once)} \\
& \sum_{j \in V, j \neq i} x_{j,i} = 1 && \forall i \in V && \text{(enter each node once)} \\
& \sum_{j=1}^{n-1} f_{0,j} = n-1 && && \text{(depot supplies n-1 units)} \\
& \sum_{j \in V, j \neq i} f_{j,i} - \sum_{j \in V, j \neq i} f_{i,j} = 1 && \forall i \in V \setminus \{0\} && \text{(flow balance: consume 1 unit)} \\
& f_{0,j} \leq (n-1) \cdot x_{0,j} && \forall j \in V \setminus \{0\} && \text{(flow-edge coupling)} \\
& f_{i,j} \leq (n-2) \cdot x_{i,j} && \forall i,j \in V \setminus \{0\}, i \neq j && \text{(flow-edge coupling)} \\
& x_{i,j} \in \{0,1\} && \forall i,j \in V, i \neq j \\
& f_{i,j} \geq 0 && \forall i,j \in V, i \neq j
\end{align}
$$

### Constraint Explanation

1. **Leave/Enter Constraints**: Same as degree constraints but for directed graphs
   - Each node has exactly 1 outgoing edge
   - Each node has exactly 1 incoming edge

2. **Flow Supply**: Depot (node 0) produces exactly $n-1$ units
   - This equals the number of non-depot nodes

3. **Flow Conservation**: For each non-depot node $i$:
   - Inflow - Outflow = 1
   - Receives commodity from predecessors, consumes 1 unit, sends remainder to successors
   - Example: Node receives 5 units, consumes 1, sends out 4 units

4. **Flow-Edge Coupling**: Flow can only use selected edges
   - If $x_{i,j} = 0$ (edge not selected), then $f_{i,j} = 0$ (no flow)
   - If $x_{i,j} = 1$ (edge selected), then $f_{i,j}$ can be up to its upper bound
   - This is the **key to subtour prevention**: subtours can't maintain flow balance!

### Why This Prevents Subtours

Consider an invalid solution with a subtour not containing the depot:
- Subtour nodes: {2, 3, 4}
- For this subtour to exist, flow must circulate within {2,3,4}
- But each node consumes 1 unit → net consumption in the subtour > 0
- No flow comes in from depot → **impossible to satisfy flow conservation!**
- Therefore, subtours are automatically prevented by flow constraints

**Advantages over SEC**:
- Polynomial number of constraints: $O(n^2)$ instead of exponential
- No need for cutting plane algorithm
- Can be solved directly as a single MIP

**Disadvantages**:
- More variables: need $O(n^2)$ flow variables
- Typically slower for symmetric TSP
- Better suited for asymmetric TSP

---

## Code Implementation (Section 5.1.3)

### Method 1: `scf(n, c)` - Build SCF Model

```python
def scf(n,c):
    model = grbpy.Model("Asymmetric_TSP-Single_Flow_Commodity")
    x,f ={},{}
    
    # Create variables
    for i in range(n):
        for j in range(n):
            if i != j:
                x[i,j] = model.addVar(vtype=grbpy.GRB.BINARY)
                if i==0:
                    f[i,j] = model.addVar(ub=n-1, vtype=grbpy.GRB.CONTINUOUS)
                else:
                    f[i,j] = model.addVar(ub=n-2, vtype=grbpy.GRB.CONTINUOUS)
    model.update()
```

**Variable Creation**:
1. **Edge variables $x_{i,j}$**: Binary for all $i \neq j$
2. **Flow variables $f_{i,j}$**:
   - From depot ($i=0$): upper bound = $n-1$ (can supply all demand)
   - From other nodes: upper bound = $n-2$ (received flow minus own consumption)

```python
    # Degree constraints
    for i in range(n):
        model.addConstr(grbpy.quicksum(x[i,j] for j in range(n) if j!=i)==1)
        model.addConstr(grbpy.quicksum(x[j,i] for j in range(n) if j!=i)==1)
```

**Degree Constraints**:
- Each node has exactly 1 outgoing edge: $\sum_{j \neq i} x_{i,j} = 1$
- Each node has exactly 1 incoming edge: $\sum_{j \neq i} x_{j,i} = 1$

```python
    # Flow supply from depot
    model.addConstr(grbpy.quicksum(f[0,j] for j in range(1,n))==n-1)
    
    # Flow conservation for non-depot nodes
    for i in range(1,n):
        model.addConstr(grbpy.quicksum(f[j,i] for j in range(n) if j!=i)
                        -grbpy.quicksum(f[i,j] for j in range(n) if j!=i)==1)
```

**Flow Constraints**:
1. **Depot supply**: Total outflow from node 0 = $n-1$
2. **Conservation**: For each node $i$ (except depot):
   - Inflow - Outflow = 1
   - Net consumption of 1 unit per node

```python
    # Flow-edge coupling
    for j in range(1,n):
        model.addConstr(f[0,j] <= (n-1)*x[0,j])
        for i in range(1,n):
            if i != j :
                model.addConstr(f[i,j] <= (n-2)*x[i,j])
```

**Coupling Constraints**:
- If edge $(i,j)$ not selected ($x_{i,j}=0$), then $f_{i,j}=0$
- If edge selected ($x_{i,j}=1$), flow up to capacity allowed
- This "big-M" formulation links flow to edge selection

```python
    model.setObjective(grbpy.quicksum(c[i,j]*x[i,j] for (i,j) in x),grbpy.GRB.MINIMIZE)
    model.update()
    model.__data = x,f
    return model
```

**Finalization**:
- Set objective: minimize total travel cost
- Store variables in model for later retrieval
- Return configured model

### Method 2: `solve_tsp(V, c, n)` - Solve with Cutting Plane

```python
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
```

**Hybrid Approach**:
- Uses SCF formulation as base
- **Still applies cutting plane** for additional subtour elimination
- This seems redundant since SCF should prevent subtours on its own
- Possibly an educational demonstration or for handling numerical issues

**Why might this hybrid approach exist?**
1. **Numerical stability**: Flow constraints might not perfectly eliminate subtours due to floating-point precision
2. **Performance**: Adding explicit subtour cuts might help the solver
3. **Teaching**: Shows both methods can be combined

**Note**: The `addcut` function used here is the same one from Section 5.1.1

### Comparison: SCF vs SEC

| Aspect | Subtour Elimination (SEC) | Single Commodity Flow (SCF) |
|--------|---------------------------|----------------------------|
| **Variables** | $O(n^2)$ binary | $O(n^2)$ binary + $O(n^2)$ continuous |
| **Constraints** | Exponential (needs cutting plane) | Polynomial $O(n^2)$ |
| **Best for** | Symmetric TSP | Asymmetric TSP |
| **Solution time** | Often faster for symmetric | Can be slower due to flow vars |
| **Formulation** | Simpler concept | More complex with flows |
| **Implementation** | Needs callback/cutting plane | Direct solve possible |

---

# Section 5.3: Capacitated Vehicle Routing Problem

## Mathematical Background

### Problem Definition

The **Capacitated Vehicle Routing Problem (CVRP)** extends TSP to multiple vehicles with capacity constraints:

**Given**:
- Set of customers $V = \{1, 2, ..., n\}$ with demands $q_i$
- A depot (node 0) where all vehicles start and end
- $m$ identical vehicles, each with capacity $Q$
- Distance matrix $c_{i,j}$ between all locations

**Goal**: Find $m$ routes starting and ending at the depot that:
1. Visit all customers exactly once
2. Minimize total travel distance
3. Respect vehicle capacity: total demand on each route $\leq Q$

### Key Differences from TSP

| TSP | VRP |
|-----|-----|
| 1 tour visiting all nodes | $m$ tours, each starting/ending at depot |
| No capacity constraints | Each route limited by capacity $Q$ |
| Each node degree = 2 | Depot degree = $2m$, others = 2 |
| Simple cycle | Multiple cycles sharing depot |

## Mathematical Formulation (CVRP)

$$
\begin{align}
\min \quad & \sum_{i \in V_0} \sum_{j \in V_0, j>i} c_{i,j} \cdot x_{i,j} \\
\text{s.t.} \quad & \sum_{j \in V, j \neq 0} x_{0,j} = 2m && \text{(depot degree: } 2m \text{ edges)} \\
& \sum_{j \in V_0, j < i} x_{j,i} + \sum_{j \in V_0, j > i} x_{i,j} = 2 && \forall i \in V && \text{(customer degree: 2)} \\
& \sum_{i \in S, j \in S, j>i} x_{i,j} \leq |S| - r(S) && \forall S \subseteq V, |S| \geq 2 && \text{(generalized subtour elim.)} \\
& x_{0,j} \in \{0, 1, 2\} && \forall j \in V && \text{(depot edges can be used twice)} \\
& x_{i,j} \in \{0, 1\} && \forall i,j \in V, j>i && \text{(other edges used once)} 
\end{align}
$$

where $V_0 = V \cup \{0\}$ (customers + depot)

### The Key Component: $r(S)$ - Minimum Vehicles Needed

For any subset $S$ of customers:

$$
r(S) = \left\lceil \frac{\sum_{i \in S} q_i}{Q} \right\rceil
$$

This represents the **minimum number of vehicle routes** needed to serve all customers in $S$ given capacity $Q$.

**Example**:
- Subset $S = \{1, 2, 3\}$ with demands $q_1=3, q_2=4, q_3=5$
- Total demand: $3+4+5 = 12$
- Vehicle capacity: $Q = 7$
- $r(S) = \lceil 12/7 \rceil = \lceil 1.71 \rceil = 2$ vehicles needed

### Constraint Explanation

1. **Depot Degree Constraint**: $\sum_{j \neq 0} x_{0,j} = 2m$
   - Depot must connect to exactly $2m$ edges (in/out for each of $m$ vehicles)
   - Each route starts and ends at depot

2. **Customer Degree Constraint**: Standard TSP degree constraint
   - Each customer visited exactly once (2 incident edges)

3. **Generalized Subtour Elimination**: $\sum_{i,j \in S, j>i} x_{i,j} \leq |S| - r(S)$
   - **More sophisticated than TSP!**
   - If subset $S$ requires $r(S)$ vehicles, it needs $r(S)$ connections to depot
   - Edges within $S$ limited to $|S| - r(S)$ to force depot connections
   
   **Intuition**:
   - A tree connecting $|S|$ nodes has $|S|-1$ edges
   - But if we need $r(S)$ separate routes, we have $r(S)$ trees
   - Total internal edges: $|S| - r(S)$ (distributed among the trees)
   - Remaining $r(S)$ connections must go to depot

4. **Variable Bounds**:
   - Depot edges: can be used up to 2 times (some routes might share edges near depot)
   - Customer edges: used at most once

### Example Scenario

Consider 6 customers with demands: $q = [3, 2, 4, 3, 5, 2]$, capacity $Q=7$, $m=3$ vehicles

**Valid solution might be**:
- Route 1: Depot → 1 → 2 → Depot (demand: 3+2=5 ≤ 7) ✓
- Route 2: Depot → 3 → 6 → Depot (demand: 4+2=6 ≤ 7) ✓
- Route 3: Depot → 4 → 5 → Depot (demand: 3+5=8 > 7) ✗ **INFEASIBLE!**

Need to redistribute: 
- Route 3: Depot → 4 → Depot (demand: 3 ≤ 7) ✓
- Route 4: Depot → 5 → Depot (demand: 5 ≤ 7) ✓ (need 4 vehicles!)

---

## Code Implementation (Section 5.3)

### Method 1: `vrp(V, c, m, q, Q)` - Build VRP Model with Callback

```python
def vrp(V:list[int], c:np.ndarray, m:int, q:list[int], Q:int)->tuple[grbpy.Model, Callable[[grbpy.Model, int], None]]:
    def vrp_callback(model,where):
        if where != grbpy.GRB.Callback.MIPSOL:
            return
        edges = []
        for (i,j) in x:
            if model.cbGetSolution(x[i,j])>0.5:
                if i!=V[0] and j!=V[0]:
                    edges.append((i,j))
```

**Callback Function** (nested inside `vrp`):
- **Trigger**: Called when Gurobi finds an integer solution (`MIPSOL`)
- **Extract edges**: Get selected edges excluding depot connections
  - We check `> 0.5` since variables are integer (0, 1, or 2)
  - Filter out depot edges: only consider customer-to-customer edges

```python
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
```

**Subtour Detection Logic**:
1. **Build graph** from customer-only edges
2. **Find components** (potential subtours)
3. **For each component $S$**:
   - Calculate $|S|$ = number of customers
   - Calculate total demand: $q(S) = \sum_{i \in S} q_i$
   - Calculate minimum vehicles needed: $r(S) = \lceil q(S) / Q \rceil$
   - Get edges within $S$
4. **Add lazy constraint if violation detected**:
   - Condition: $|S| \geq 3$ AND (too many internal edges OR requires multiple vehicles)
   - Constraint: $\sum_{i,j \in S} x_{i,j} \leq |S| - r(S)$

**Why check `S_card >= 3`?**
- 2 nodes can form a simple edge (not a subtour)
- Need at least 3 nodes to form a meaningful subtour

**Why check `len(S_edges) >= S_card or NS > 1`?**
- `len(S_edges) >= S_card`: Too many edges → likely a cycle
- `NS > 1`: Capacity violation → needs multiple vehicles → must connect to depot

```python
    model = grbpy.Model("Capacitated Vehicle Routing Problem")
    x = {}
    for i in V:
        for j in V:
            if j>i and i ==V[0]:
                x[i,j]=model.addVar(vtype=grbpy.GRB.INTEGER,ub=2)
            elif j>i:
                x[i,j]=model.addVar(vtype=grbpy.GRB.INTEGER,ub=1)
    model.update()
```

**Variable Creation**:
- **Depot edges** $(0,j)$: Integer with upper bound 2
  - Can be traversed twice if two routes use similar initial path
- **Customer edges** $(i,j)$ where $i,j \neq 0$: Integer with upper bound 1
  - Each customer edge used at most once

```python
    model.addConstr(grbpy.quicksum(x[V[0],j] for j in V[1:])==2*m)
    for i in V[1:]:
        cstr1 = grbpy.quicksum(x[j,i] for j in V if j<i)
        cstr2 = grbpy.quicksum(x[i,j] for j in V if j>i)
        model.addConstr(cstr1 + cstr2 ==2)
```

**Constraints**:
1. **Depot degree**: Total edges from depot = $2m$ (m vehicles, each leaves and returns)
2. **Customer degree**: Each customer has degree 2 (visited exactly once)

```python
    model.setObjective(grbpy.quicksum(
        c[i,j]*x[i,j] for i in V for j in V if j>i),grbpy.GRB.MINIMIZE)
    model.update()
    model.__data = x
    return model, vrp_callback
```

**Return**:
- Configured model
- Callback function (to be passed to `model.optimize()`)

### Method 2: `addcut_vrp(edges, V, model, x, q, Q)` - Manual Subtour Elimination

```python
def addcut_vrp(edges:list[tuple[int, int]], V:list[int], model:grbpy.Model, x, q:list[int], Q:int)->bool:
    """a generalized subtour elimination constraints for VRP"""
    
    G = networkx.Graph() 
    G.add_nodes_from(V)
    
    for (i,j) in edges:
        G.add_edge(i,j)
    
    Components = list(networkx.connected_components(G))
    
    if len(Components) == 1:
        return False
```

**Setup**: Similar to TSP's `addcut`, but adapted for VRP

```python
    for S in Components:
        if V[0] in S:
            continue
            
        demand_S = sum(q[i] for i in S) 
        r_S = np.ceil(demand_S / Q)
        
        model.addConstr(
            grbpy.quicksum(x[i,j] for i in S for j in S if j>i and (i,j) in x) 
            <= len(S) - r_S
        )
    
    return True
```

**Key Difference from TSP**:
1. **Skip depot component**: If depot is in component $S$, it's the main route structure
2. **Calculate $r(S)$**: Minimum vehicles needed for subset
3. **Add constraint**: $|E(S)| \leq |S| - r(S)$ (generalized subtour elimination)

**This enforces**: Subsets needing multiple vehicles must have connections to depot

### Method 3: `solve_vrp(V, c, m, q, Q)` - Main VRP Solver

```python
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
```

**Solve Process**:
1. **Build model** with callback
2. **Enable lazy constraints**: Required for callback to work
3. **Optimize** with callback function
4. **Check feasibility**: Ensure solution exists
5. **Extract edges with multiplicity**: 
   - Format: `(i, j, count)` where count is how many times edge is used
   - Important for depot edges which can be used twice

### Method 4: `extract_vrp_routes(edges_with_mult, V, m)` - Route Reconstruction

This is the most complex method - it converts the edge solution into separate vehicle routes.

```python
def extract_vrp_routes(edges_with_mult:list[tuple[int,int,int]], V:list[int], m:int)->list[list[int]]:
    depot = V[0]
    edge_count = defaultdict(int)
    adj = defaultdict(set)
    
    for (i,j,mult) in edges_with_mult:
        a, b = (i, j) if i < j else (j, i)
        edge_count[(a,b)] += int(mult)
        adj[i].add(j)
        adj[j].add(i)
```

**Data Structure Setup**:
- `edge_count`: Dictionary tracking how many times each edge is used
- `adj`: Adjacency list for graph traversal
- Normalize edges to $(min, max)$ format for consistent lookup

```python
    def has_unused_depot_edge():
        return any(edge_count[(min(depot, nbr), max(depot, nbr))] > 0 for nbr in adj[depot])

    def consume_edge(u, v):
        a, b = (u, v) if u < v else (v, u)
        if edge_count[(a,b)] <= 0:
            return False
        edge_count[(a,b)] -= 1
        return True
```

**Helper Functions**:
- `has_unused_depot_edge()`: Check if we can start a new route from depot
- `consume_edge(u, v)`: Mark an edge as used (decrement count)

```python
    routes = []
    while len(routes) < m and has_unused_depot_edge():
        neighbors = [nbr for nbr in adj[depot] if edge_count[(min(depot, nbr), max(depot, nbr))] > 0]
        if not neighbors:
            break
        next_node = neighbors[0]
        
        if not consume_edge(depot, next_node):
            break
        route = [depot]
        prev = depot
        current = next_node
```

**Route Extraction Loop**:
1. Continue while we haven't found all $m$ routes
2. Find available neighbor from depot
3. Start new route: Depot → first neighbor
4. Consume the depot edge

```python
        while True:
            route.append(current)
            
            if current == depot:
                break
            
            next_candidates = [nbr for nbr in adj[current]
                               if edge_count[(min(current, nbr), max(current, nbr))] > 0 and nbr != prev]
            
            if next_candidates:
                prev, current = current, next_candidates[0]
                consume_edge(prev, current)
                continue
            
            if edge_count[(min(current, depot), max(current, depot))] > 0:
                prev, current = current, depot
                consume_edge(prev, current)
                route.append(current)
                break
            
            route.append(depot)
            break
        
        if route[-1] != depot:
            route.append(depot)
        
        routes.append(route)
    
    return routes
```

**Route Following Algorithm**:
1. **Add current node** to route
2. **Check if back at depot** → route complete
3. **Find next node**: 
   - Look for unused edges from current (excluding previous node)
   - If found → continue walking
4. **Return to depot**:
   - If no forward options, try edge back to depot
   - Close the route
5. **Ensure route closure**: Always end at depot
6. **Save route** and start next one

**Example Walkthrough**:
- Edges: `{(0,1,1), (1,2,1), (2,0,1), (0,3,1), (3,4,1), (4,0,1)}`
- Route 1: Start at 0 → consume (0,1) → at 1 → consume (1,2) → at 2 → consume (2,0) → back at 0 → **[0,1,2,0]**
- Route 2: Start at 0 → consume (0,3) → at 3 → consume (3,4) → at 4 → consume (4,0) → back at 0 → **[0,3,4,0]**

### Visualization Code

The `main()` function creates animated or static visualizations:

**Key Features**:
- **Different colors** for each vehicle route
- **Depot marked** with red square
- **Customers marked** with blue circles
- **Animation** shows routes being constructed sequentially

```python
route_colors = ['green', 'orange', 'purple', 'cyan', 'magenta', 'yellow']
```

Each vehicle gets a distinct color for easy visual identification.

---

## Summary Comparison

| Feature | TSP (5.1.1) | TSP-SCF (5.1.3) | CVRP (5.3) |
|---------|-------------|-----------------|------------|
| **Vehicles** | 1 | 1 | m (multiple) |
| **Constraints** | Subtour elim. | Flow conservation | Capacity + Subtour elim. |
| **Depot degree** | 2 | 2 | 2m |
| **Variables** | $x_{ij}$ only | $x_{ij}$ + $f_{ij}$ | $x_{ij}$ (integer) |
| **Edge bounds** | All {0,1} | All {0,1} | Depot: {0,1,2}, Others: {0,1} |
| **Key formula** | $\|E(S)\| \leq \|S\|-1$ | Flow balance | $\|E(S)\| \leq \|S\|-r(S)$ |
| **Complexity** | Exponential constraints | Polynomial | Exponential constraints |
| **Best for** | Symmetric distances | Asymmetric distances | Multiple vehicles with capacity |

---

## Practical Insights

### When to Use Each Formulation

**TSP with Subtour Elimination (5.1.1)**:
- ✅ Symmetric distances
- ✅ Need fast solution
- ✅ Can implement lazy constraints
- ❌ Asymmetric distances

**TSP with Single Commodity Flow (5.1.3)**:
- ✅ Asymmetric distances
- ✅ Prefer polynomial constraints
- ✅ Educational purposes
- ❌ Very large instances (flow variables add overhead)

**CVRP (5.3)**:
- ✅ Multiple vehicles needed
- ✅ Capacity constraints present
- ✅ Real-world routing problems
- ❌ Simple single-vehicle scenarios

### Performance Tips

1. **Use Gurobi parameters**:
   ```python
   model.Params.LazyConstraints = 1  # Enable lazy constraints
   model.Params.TimeLimit = 300      # Set time limit
   model.Params.MIPGap = 0.01        # Accept 1% optimality gap
   ```

2. **Problem-specific improvements**:
   - **TSP**: Use strong valid inequalities (comb inequalities, etc.)
   - **CVRP**: Add initial valid routes as starting solution
   - **Both**: Preprocess to eliminate obviously poor edges

3. **Scaling**:
   - TSP: Solvable to 100s-1000s of nodes with modern methods
   - CVRP: More challenging, typically 100-300 customers realistic
   - For larger instances: use metaheuristics or column generation

---

*End of Chapter 5 Documentation*