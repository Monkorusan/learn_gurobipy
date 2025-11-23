# Traveling Salesman Problem (TSP) with Gurobi: Math and Implementation

## Formulation and Theory

The Traveling Salesman Problem (TSP) is a classic combinatorial optimization problem:
- **Given:** A set of $n$ cities (nodes) and a cost $c_{ij}$ for traveling from city $i$ to city $j$.
- **Goal:** Find the shortest possible tour that visits each city exactly once and returns to the starting city.

### Mathematical Formulation

Let $x_{ij}$ be a binary variable:
- $x_{ij} = 1$ if the tour includes the edge from $i$ to $j$
- $x_{ij} = 0$ otherwise

The standard integer programming formulation is:

$$
\begin{align*}
\text{minimize} \quad & \sum_{i < j} c_{ij} x_{ij} \\
\text{subject to} \quad & \sum_{j: j \ne i} x_{ij} = 2 \quad \forall i \in V \quad \text{(degree constraint)} \\
& \sum_{i, j \in S, i < j} x_{ij} \leq |S| - 1 \quad \forall S \subset V, 2 \leq |S| \leq n-1 \quad \text{(subtour elimination)} \\
& x_{ij} \in \{0, 1\}
\end{align*}
$$

- **Degree constraint:** Each city has exactly two incident edges (enters and leaves once).
- **Subtour elimination:** Prevents solutions with multiple disconnected cycles (subtours).
- **Binary variables:** $x_{ij}$ are 0 or 1.


## How Is the TSP Solved in Practice?

The TSP, as formulated above, is a nonconvex combinatorial optimization problem. The main mathematical approach to solving it is called **branch-and-cut**, which combines:

- **LP Relaxation:** First, the integer constraints ($x_{ij} \in \{0,1\}$) are relaxed to $0 \leq x_{ij} \leq 1$, making the problem a linear program (LP). This LP is solved efficiently, but its solution may contain fractional $x_{ij}$ values and subtours (disconnected cycles).
- **Subtour Elimination via Cutting Planes:** After solving the LP, the solution is checked for subtours. If any are found, additional constraints (cuts) are added to eliminate them. These are the subtour elimination constraints: $\sum_{i, j \in S, i < j} x_{ij} \leq |S| - 1$ for each subtour $S$.
- **Branch-and-Bound:** If the solution is still fractional (not all $x_{ij}$ are 0 or 1), the algorithm branches on a fractional variable, creating two subproblems (one with $x_{ij}=0$, one with $x_{ij}=1$), and recursively solves them. This process continues until an integer (binary) solution is found.

This approach is called **branch-and-cut** because it combines branch-and-bound (for integrality) with cutting planes (for subtour elimination). The process is repeated: solve LP, add cuts, branch if needed, until an optimal tour is found.

**Why is this nonconvex?** The set of all valid tours is not convex because of the binary/integer constraints and the combinatorial nature of the subtour elimination constraints. The feasible region is a union of discrete points (tours), not a convex set.

Modern solvers like Gurobi automate this process, efficiently searching for the optimal tour using these mathematical ideas.

## Code Implementation

### Variables and Objective
- The code creates variables $x_{ij}$ for $i < j$ using `model.addVar(ub=1)`.
- The objective is set as the sum of $c_{ij} x_{ij}$ for all $i < j$:
  ```python
  model.setObjective(grbpy.quicksum(c[i,j]*x[i,j] for i in V for j in V if j>i), grbpy.GRB.MINIMIZE)
  ```

### Degree Constraints
- For each node $i$, the code ensures exactly two incident edges:
  ```python
  for i in V:
      cstr1 = grbpy.quicksum(x[j,i] for j in V if j<i)
      cstr2 = grbpy.quicksum(x[i,j] for j in V if j>i)
      model.addConstr(cstr1 + cstr2 == 2)
  ```
  This matches $\sum_{j: j \ne i} x_{ij} = 2$.

### Subtour Elimination (Cutting Planes)
- After solving the relaxed problem, the code checks for subtours using NetworkX:
  ```python
  Components = list(networkx.connected_components(G))
  for S in Components:
      model.addConstr(
          grbpy.quicksum(x[i,j] for i in S for j in S if j>i) <= len(S)-1)
  ```
  This implements $\sum_{i, j \in S, i < j} x_{ij} \leq |S| - 1$ for each component $S$ (subtour) found.

### Integer Constraints
- The code sets variables to binary if subtours are eliminated:
  ```python
  for (i,j) in x:
      x[i,j].Vtype = grbpy.GRB.BINARY
  model.update()
  ```

### Nonconvexity
- The problem is nonconvex because the feasible set (all valid tours) is not a convex set, due to the binary/integer constraints and the subtour elimination constraints.
- Gurobi solves this using branch-and-cut: it relaxes the integer constraints, solves the LP, and adds subtour cuts as needed, branching on variables to enforce integrality.

---

**References:**
- [Wikipedia: Traveling Salesman Problem](https://en.wikipedia.org/wiki/Travelling_salesman_problem)
- [Gurobi TSP Example](https://www.gurobi.com/documentation/current/examples/tsp_py.html)
