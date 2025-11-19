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
model = Model("transportation")

demand = [80, 270, 250, 160, 180]
supply = [500, 500, 500]
cost = [[4, 5, 6, 8, 10],
        [6, 4, 3, 5,  8],
        [9, 7, 4, 3,  4]]

d = {key+1:value for key,value in enumerate(demand)} #guest_id:demand amount dict
M = {key+1:value for key,value in enumerate(supply)} #factory_id:supply amount dict
I = [i for i in range(1,len(demand)+1)] # 1 2 3 4 5
J = [j for j in range(1,len(supply)+1)] # 1 2 3
c = {(j, i): cost[j-1][i-1] for j in J for i in I  }

x = {} # x_ij dictionary: key ij, value variable x_ij
for j in J:
    for i in I:
        x[j,i] = model.addVar(vtype='C',name=f'x({j},{i})')
        model.addConstr(x[j,i]>=0,name=f"x_{j}{i}=>0")
model.update()

for j in J:
    sum_terms = (f"{c[j,i]}x_{j}{i}" for i in I)
    sum_string = " + ".join(sum_terms)
    model.addConstr(quicksum(x[j,i] for i in I)<=M[j], 
                    name=f"transportation cost from factory {j}: {sum_string} <= {M[j]}")
for i in I:
    sum_terms = (f"{c[j,i]}x_{j}{i}" for j in J)
    sum_string = " + ".join(sum_terms)
    model.addConstr(quicksum(x[j,i] for j in J)==d[i],
                    name=f"demands from guest {i}: {sum_string} <= {d[i]}")
                    
# totalcost = 0
# for j in J:
#     for i in I:
#         totalcost = totalcost + c[j,i]*x[j,i]
model.setObjective(quicksum(c[j,i]*x[j,i] for (j,i) in x), GRB.MINIMIZE)
model.optimize()
print("\n")
print(f"minimized cost = {model.ObjVal}\n")
EPS = 1e-6 # stands for Epsilon, represents tolerable calcualtion error.

for (j,i) in x:
    sol = x[j,i].X
    if sol > EPS:
        print(f"sending quantity {sol} from factory {j} to guest {i}")

print(f"optimal solution = {[x[j,i].X for (j,i) in x]}")

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