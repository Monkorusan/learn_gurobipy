# Subtour Elimination Formulation
- Given an undirected graph G=(V,E), where V and E are the set of
  nodes and edges respectively,let x_e be a binary variable
  such that $e \in E $. 
- let S be a subset of some arbitrary points, we can denote 
  the edge with both ends connected to points \in S as E(S).
- let $delta(S)$ be the set of edges where only 1 end of the said edges is 
  included in S but the other end of the said edges is not.
   - we can use this property to show neighboring nodes and edges.
- in order for the edges and nodes to form a path, any node has to be
  connected to at least 2 or more edges. (let's call this star1)
- in order to form a closed loop path, under star1 constraint,
  - total number of edges in set S equals 
    total number of node in S subtracted by 1 (let's call this star2)
--we will write the 2 abovementioned property in our constraints!)

## Formulation of SEC

minimize sum_e in E          dist_e*x_e 
    s.t. sum_e in delta({i}) x_e == 2        for all i in V
         sum_e in E(S)       x_e <= |S|-1    for all subsetS in V, |S|>=2
         x_e \in {0,1}

# Cutset Constraint
- we change star1 constraint to
  - sum_e in delta(S) x_e => 2 for all S in V, |S|>=2
- this constraint alone is essentially the same as both star1 and star2 combined!

# Cutting Plane method
- This method is implemeted in `def solve_tsp` method