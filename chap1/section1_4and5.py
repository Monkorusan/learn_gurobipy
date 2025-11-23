# 1.4: Transportation problem

""" ====================  Description ===================  
- given 5 guests, 3 factories (say, sport goods)
- the goods has to be transported from each factories to each guests.
- each factory can produce M_j amount of goods.
- each guests having different amount of demands d_i .
- the transportation cost is shown in table1
- decide on which factory should supply goods to which guests to 
  minimize transportation cost?

                     table
guest    i |  1   2   3   4   5       |
demand d_i | 80 270 250 160 180       |
--------------------------------------|----------
factory    | transportation cost c_ij |supply amount M_j
    1      |  4   5   6   8  10       |   500
    2      |  6   4   3   5   8       |   500
    3      |  9   7   4   3   4       |   500

- solution: 
  - use continuous var x_ij:=amount to be transported from factory j to guest i
- hint:
  - store d_ij and M_j as a dictionary
  - remember5steps: model -> addVar -> addConstr -> setObjective -> optimize
"""

from gurobipy import *
import numpy as np
model = Model("transportation")

demand = [80, 270, 250, 160, 180]
supply = [500, 500, 500]
cost = [[4, 5, 6, 8, 10],
        [6, 4, 3, 5,  8],
        [9, 7, 4, 3,  4]]
cost=np.array(cost)
cost = cost.T

d = {key+1:value for key,value in enumerate(demand)} #guest_id:demand amount dict
M = {key+1:value for key,value in enumerate(supply)} #factory_id:supply amount dict
I = [i for i in range(1,len(demand)+1)] # 1 2 3 4 5
J = [j for j in range(1,len(supply)+1)] # 1 2 3
c = {(i, j): cost[i-1][j-1] for i in I for j in J}

x = {} # x_ij dictionary: key ij, value variable x_ij
for j in J:
    for i in I:
        x[i,j] = model.addVar(vtype='C',lb=0,name=f'x({i},{j})')

model.update()

for j in J:
    sum_terms = (f"x_{i}{j}" for i in I)
    sum_string = "+".join(sum_terms)
    model.addConstr(quicksum(x[i,j] for i in I)<=M[j], 
                    name=f"transportation_cost_from_factory_{j}___{sum_string}<={M[j]}")
for i in I:
    sum_terms = (f"x_{i}{j}" for j in J)
    sum_string = "+".join(sum_terms)
    model.addConstr(quicksum(x[i,j] for j in J)==d[i],
                    name=f"demands_from_guest_{i}___{sum_string}<={d[i]}")
                    
# totalcost = 0
# for j in J:
#     for i in I:
#         totalcost = totalcost + c[j,i]*x[j,i]
model.setObjective(quicksum(c[i,j]*x[i,j] for (i,j) in x), GRB.MINIMIZE)
model.optimize()
print("\n")
print(f"minimized cost = {model.ObjVal}\n")
EPS = 1e-6 # stands for Epsilon, represents tolerable calcualtion error.

for (i,j) in x:
    sol = x[i,j].X
    if sol > EPS:
        print(f"sending quantity {sol} from factory {j} to guest {i}")

print(f"optimal solution = {[x[i,j].X for (i,j) in x]}")

# 1.5: Dual problem version of 1.4
CC = model.getConstrs()
print("in a constraint: c_ji*x_ji means that it costs c_ij units to move from factory j to guest i")
print("CONSTRAINTS ~ SLACK ~ PI table")
for CCC in CC:
  print(f"{CCC.ConstrName} ~ {CCC.Slack} ~ {CCC.Pi}")

""" ============================ about Slack and Pi ============================ 
- Slack: The slack of a constraint is the difference between the left-hand side (LHS) 
         and the right-hand side (RHS) of the constraint, evaluated at the current 
         solution. For a constraint (a^T)*x <= b, its slack is described as b-(a^T)*x .
         Slack tells us how "loose" the constraint is at the solution x 
         (slack is close to zero if things get too tight)
- Pi:    The Pi attribute represents the dual value (shadow price) of the constraint 
         in the optimal solution. It indicates how much the objective function would
         improve per unit increase in the RHS of the constraint, holding all else 
         constant.
"""

model.write("transport_problem.lp")