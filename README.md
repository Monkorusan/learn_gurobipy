# Learn Gurobipy

This repository contains implementations of various optimization problems using Gurobi Python API (gurobipy), focusing on Traveling Salesman Problem (TSP) and Vehicle Routing Problem (VRP) formulations.

## Prerequisites

### Installation

1. **Install Python dependencies:**
   ```bash
   pip install -r requirements.txt
   ```

2. **Install Gurobi and gurobipy:**
   
   Gurobi requires a license. Academic users can obtain a free academic license.
   
   - Visit [Gurobi Downloads](https://www.gurobi.com/downloads/)
   - Register and obtain a license
   - Install Gurobi Optimizer
   - Install gurobipy: `pip install gurobipy`
   - Activate your license following [Gurobi's instructions](https://www.gurobi.com/documentation/)

## Chapter 1: Introduction to Optimization

### Section 1.2: Linear Programming

**Description:** Basic linear programming example with continuous variables. Demonstrates setting up a model, adding constraints, and maximizing an objective function.

**Run:**
```bash
python chap1/section1_2.py
```

**Problem Formulation:**

$$
\begin{align*}
\text{maximize} \quad & 15x_1 + 18x_2 + 30x_3 \\
\text{subject to} \quad & 2x_1 + x_2 + x_3 \leq 60 \\
& x_1 + 2x_2 + x_3 \leq 60 \\
& x_3 \leq 30 \\
& x_1, x_2, x_3 \geq 0
\end{align*}
$$

---

### Section 1.3: Integer Linear Programming

**Description:** Integer programming problem solving a puzzle-type optimization. Demonstrates use of integer variables (`vtype="I"`).

**Run:**
```bash
python chap1/section1_3.py
```

**Problem Formulation:**

$$
\begin{align*}
\text{minimize} \quad & y + z \\
\text{subject to} \quad & x + y + z = 32 \\
& 2x + 4y + 8z = 80 \\
& x, y, z \in \mathbb{Z}_{\geq 0}
\end{align*}
$$

---

### Section 1.4 & 1.5: Transportation Problem and Duality

**Description:** Classic transportation problem with 3 factories and 5 guests. Finds optimal allocation to minimize transportation costs while satisfying demand and supply constraints. Also explores dual variables (shadow prices) and slack analysis.

**Run:**
```bash
python chap1/section1_4and5.py
```

**Problem Formulation:**

Given:
- 5 guests with demands: $d_1 = 80, d_2 = 270, d_3 = 250, d_4 = 160, d_5 = 180$
- 3 factories with supplies: $M_1 = 500, M_2 = 500, M_3 = 500$
- Transportation cost matrix $c_{ij}$ (guest $i$, factory $j$):

| Guest $i$ | Factory 1 | Factory 2 | Factory 3 |
|-----------|-----------|-----------|-----------|
| 1         | 4         | 6         | 9         |
| 2         | 5         | 4         | 7         |
| 3         | 6         | 3         | 4         |
| 4         | 8         | 5         | 3         |
| 5         | 10        | 8         | 4         |

$$
\begin{align*}
\text{minimize} \quad & \sum_{i=1}^{5} \sum_{j=1}^{3} c_{ij} x_{ij} \\
\text{subject to} \quad & \sum_{i=1}^{5} x_{ij} \leq M_j \quad \forall j \in \{1, 2, 3\} \quad \text{(supply constraints)} \\
& \sum_{j=1}^{3} x_{ij} = d_i \quad \forall i \in \{1, 2, 3, 4, 5\} \quad \text{(demand constraints)} \\
& x_{ij} \geq 0 \quad \forall i, j
\end{align*}
$$

**Key Features:**
- Continuous decision variables for transportation quantities
- Supply and demand constraints
- Cost minimization
- Dual solution analysis (Pi and Slack values)
- Exports model to LP file format

---

### Section 1.6: Multi-Commodity Transportation Problem

**Description:** Extended transportation problem with multiple product types (tennis ball, football, basketball, rugby ball). Each factory can produce different subsets of products with weight-based costs.

**Run:**
```bash
python chap1/section1_6.py
```

**Problem Formulation:**

Given:
- 5 guests (indexed by $i$)
- 3 factories (indexed by $j$)
- 4 products (indexed by $k$): 1=tennis ball, 2=football, 3=basketball, 4=rugby ball
- Product weights: $w_1 = 5, w_2 = 2, w_3 = 3, w_4 = 4$ kg
- Transportation cost matrix $c_{ij}$ (per kg) and factory supplies $M_j = 3000$ for all $j$
- Demand matrix $d_{ik}$ (guest $i$, product $k$)
- Factory production constraints:
  - Factory 1 can produce products: {2, 4}
  - Factory 2 can produce products: {1, 2, 3}
  - Factory 3 can produce products: {2, 3, 4}

$$
\begin{align*}
\text{minimize} \quad & \sum_{i,j,k} c_{ij} \cdot w_k \cdot x_{ijk} \\
\text{subject to} \quad & \sum_{j} x_{ijk} = d_{ik} \quad \forall i \in \{1,\ldots,5\}, \forall k \in \{1,2,3,4\} \quad \text{(demand)} \\
& \sum_{i,k} x_{ijk} \leq M_j \quad \forall j \in \{1,2,3\} \quad \text{(supply)} \\
& x_{ijk} = 0 \quad \text{if factory } j \text{ cannot produce product } k \\
& x_{ijk} \geq 0 \quad \forall i, j, k
\end{align*}
$$

where $x_{ijk}$ is the amount of product $k$ transported from factory $j$ to guest $i$.

**Key Features:**
- 3-dimensional decision variables $x_{ijk}$ (guest, factory, product)
- Different product availability per factory
- Weight-based cost calculation
- Multiple demand constraints per guest

---

## Chapter 5: Routing Problems

### Section 5.1.1: TSP - Subtour Elimination Formulation

**Description:** Implements the classic Traveling Salesman Problem using the subtour elimination constraint formulation. Dynamically adds constraints to eliminate subtours during optimization. Includes animated visualization of the tour.

**Run:**
```bash
python chap5/section5_1_1.py
```

**Problem Formulation:**

Given:
- Set of $n$ cities (nodes) $V = \{1, 2, \ldots, n\}$
- Distance/cost matrix $c_{ij}$ for traveling from city $i$ to city $j$

$$
\begin{align*}
\text{minimize} \quad & \sum_{i < j} c_{ij} x_{ij} \\
\text{subject to} \quad & \sum_{j: j \ne i} x_{ij} = 2 \quad \forall i \in V \quad \text{(degree constraint)} \\
& \sum_{i, j \in S, i < j} x_{ij} \leq |S| - 1 \quad \forall S \subset V, 2 \leq |S| \leq n-1 \quad \text{(subtour elimination)} \\
& x_{ij} \in \{0, 1\} \quad \forall i < j
\end{align*}
$$

where:
- $x_{ij} = 1$ if the edge between cities $i$ and $j$ is in the tour, 0 otherwise
- Degree constraint: each city has exactly two incident edges
- Subtour elimination: prevents disconnected cycles

**Key Features:**
- Subtour elimination via lazy constraints or iterative constraint addition
- Networkx-based subtour detection
- Animated tour visualization using matplotlib

---

### Section 5.1.3: TSP - Single Commodity Flow Formulation

**Description:** Solves the Asymmetric TSP using the single commodity flow formulation. Uses continuous flow variables to prevent subtours implicitly without requiring dynamic constraint generation.

**Run:**
```bash
python chap5/section5_1_3.py
```

**Problem Formulation:**

Given:
- Set of $n$ cities (nodes) $V = \{1, 2, \ldots, n\}$, with city 1 as the depot
- Distance/cost matrix $c_{ij}$ for traveling from city $i$ to city $j$

$$
\begin{align*}
\text{minimize} \quad & \sum_{i \ne j} c_{ij} x_{ij} \\
\text{subject to} \quad & \sum_{j: j \ne i} x_{ij} = 1 \quad \forall i \in V \quad \text{(leave each city once)} \\
& \sum_{j: j \ne i} x_{ji} = 1 \quad \forall i \in V \quad \text{(enter each city once)} \\
& \sum_{j \ne 1} f_{1j} = n-1 \quad \text{(depot sends } n-1 \text{ flow units)} \\
& \sum_{j \ne i} f_{ji} - \sum_{j \ne i} f_{ij} = 1 \quad \forall i \in V \setminus \{1\} \quad \text{(flow conservation)} \\
& f_{1j} \leq (n-1) x_{1j} \quad \forall j \ne 1 \quad \text{(capacity from depot)} \\
& f_{ij} \leq (n-2) x_{ij} \quad \forall i, j \ne 1, i \ne j \quad \text{(capacity between cities)} \\
& x_{ij} \in \{0, 1\} \quad \forall i \ne j \\
& f_{ij} \geq 0 \quad \forall i \ne j
\end{align*}
$$

where:
- $x_{ij} = 1$ if the tour goes from city $i$ to city $j$, 0 otherwise
- $f_{ij}$ is the flow of commodity along edge $(i,j)$
- Flow variables eliminate subtours implicitly

**Key Features:**
- Flow-based formulation (no subtour detection needed)
- Works for asymmetric distance matrices
- Animated tour visualization

---

### Section 5.3: Capacitated Vehicle Routing Problem (CVRP)

**Description:** Implements the Capacitated Vehicle Routing Problem where multiple vehicles with capacity constraints serve customers from a central depot. Uses generalized subtour elimination constraints with capacity considerations.

**Run:**
```bash
python chap5/section5_3.py
```

**Problem Formulation:**

Given:
- Set of customers $V = \{2, 3, \ldots, n+1\}$ with depot at node 1
- Set of $m$ vehicles, each with capacity $Q$
- Customer demands $q_i$ for $i \in V$
- Distance/cost matrix $c_{ij}$ between all nodes

$$
\begin{align*}
\text{minimize} \quad & \sum_{i,j} c_{ij} x_{ij} \\
\text{subject to} \quad & \sum_{j \ne i} x_{ij} = 1 \quad \forall i \in V \quad \text{(each customer visited once)} \\
& \sum_{j \ne i} x_{ji} = 1 \quad \forall i \in V \quad \text{(each customer left once)} \\
& \sum_{j \ne 1} x_{1j} = m \quad \text{(m vehicles leave depot)} \\
& \sum_{i \ne 1} x_{i1} = m \quad \text{(m vehicles return to depot)} \\
& \sum_{i,j \in S, i \ne j} x_{ij} \leq |S| - \left\lceil \frac{\sum_{i \in S} q_i}{Q} \right\rceil \quad \forall S \subseteq V, S \ne \emptyset \quad \text{(capacity cuts)} \\
& x_{ij} \in \{0, 1\} \quad \forall i, j
\end{align*}
$$

where:
- $x_{ij} = 1$ if a vehicle travels from node $i$ to node $j$, 0 otherwise
- The capacity cuts ensure that if total demand in subset $S$ exceeds vehicle capacity, multiple vehicles must serve $S$
- $\left\lceil \frac{\sum_{i \in S} q_i}{Q} \right\rceil$ is the minimum number of vehicles needed for subset $S$

**Key Features:**
- Multiple vehicle routing with capacity constraints
- Lazy constraint callbacks for subtour elimination
- Route extraction and validation
- Animated visualization showing parallel vehicle dispatch
- Color-coded routes for each vehicle

**Parameters (configurable in `main()`):**
- `n`: Number of customers (default: 24)
- `m`: Number of vehicles (default: 4)
- `Q`: Vehicle capacity (default: 6)
- `is_animated`: Enable/disable animation (default: True)
- `save_video`: Save animation as MP4 (default: False)

---

## Project Structure

```
learn_gurobipy/
├── chap5/
│   ├── section5_1_1.py    # TSP - Subtour Elimination
│   ├── section5_1_3.py    # TSP - Single Commodity Flow
│   ├── section5_3.py      # Capacitated VRP
│   ├── math.md            # Mathematical formulations
│   └── note5_1.md         # Additional notes
├── requirements.txt
└── README.md
```

## Dependencies

- **numpy**: Numerical computations and matrix operations
- **matplotlib**: Visualization and animation
- **networkx**: Graph operations and connected component detection
- **gurobipy**: Gurobi optimization solver (requires license)

## Notes

- All scripts generate random problem instances with fixed seeds for reproducibility
- Animations can be disabled by setting `is_animated = False` in the `main()` function
- The VRP animation accurately represents real-world parallel vehicle dispatch
- Gurobi output shows the optimization process including cuts being added dynamically

## License

Academic/Educational use only (due to Gurobi licensing requirements)
