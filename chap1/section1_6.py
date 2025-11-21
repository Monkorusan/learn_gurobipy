# 1.6: Multi-Commodity transportation problem
# same as section 1.4 but add an extra spice:
#                       commodity now is no longer singular. hence, the name.
# minimize sum_{guest in guests} sum_{factory in factories} sum_{commodity in commodities}
# objective variable = x_ijk = amount of goods k being sent from factory j to guest i


import gurobipy as grbpy
import numpy as np

# dict, key=factory id , val=supplyable products
# labeled with 1,2,3,4 and tennisball,football,basketball,rugby ball respectively
factory1product = [z for z in range(2,5) if z!=3]
factory2product = [z for z in range(1,4)]
factory3product = [z for z in range(2,5)]
production_list = [factory1product,factory2product,factory3product]
production_dict = {z+1:commodity for z,commodity in enumerate(production_list)} #ok

# row:=ballname = tennisball,football,basketball,rugby ball
# col:=guest id = 1,2,3,4,5
demand = np.array([ [80 , 85,300,6],
                    [270,160,400,7],
                    [250,130,350,4],
                    [160, 60,200,3],
                    [180, 40,150,5]])
supply = [3000, 3000, 3000]
costt = [[4, 5, 6, 8, 10],
         [6, 4, 3, 5,  8],
         [9, 7, 4, 3,  4]]
Weights = [5,2,3,4] # mass per product, say,tennisball = 5kg lol
costt = np.array(costt) #ok
costt = costt.T

I = {i for i in range(1,len(demand)+1)} # 1 2 3 4 5 #guest id
J = {j for j in range(1,len(supply)+1)} # 1 2 3 # factory set
K = {k for k in range(1,5)} # 1 2 3 4 # product set
d = {(i,k):demand[i-1][k-1] for i in I for k in K} #ok = # dict (i,k):distane value
M = {key+1:value for key,value in enumerate(supply)} #ok
cost = {(i,j):costt[i-1][j-1] for i in I for j in J} #ok
weight = {key+1:value for key,value in enumerate(Weights)} #weightdict # ok

model = grbpy.Model("Multi-Commodity Transportation Problem")
c = {}
x = {}
for i in I:
    for j in J:
        for k in production_dict[j]:
            c[i,j,k] = cost[i,j]*weight[k]

for (i,j,k) in c:
    x[i,j,k] = model.addVar(lb=0,vtype=grbpy.GRB.CONTINUOUS,
            name=f'x_{i}{j}{k} = amount of goods numer {k} being sent from factory {j} to guest {i}')

for i in I:
    for k in K:
        model.addConstr(grbpy.quicksum(x[i,j,k] for j in J if (i,j,k) in x)==d[i,k],
                        name=f"all factories must satify guest {i}'s demand on product number {k}: demand = {d[i,k]}")

for j in J:
    model.addConstr(grbpy.quicksum(x[i,j,k] for (i,j2,k) in x if j2 == j)<=M[j],
                    name=f"factory {j} can only produce products from list={production_dict[j]} in total amount of {M[j]}")

model.update()
model.setObjective(expr=grbpy.quicksum(c[i,j,k]*x[i,j,k] for (i,j,k) in x),sense=grbpy.GRB.MINIMIZE)
model.optimize()
print(f"optimal value = {model.ObjVal}")

EPS = 1e-6 
print("\n================= Detailed Solution Description =================")
for (i,j,k) in x:
    sol = x[i,j,k].X
    if sol > EPS:
        #print(f"sending {sol} amount of product No {k} from factory {j} to guest {i}")
        print(f"sending {sol:8.1f}  unit of product No {k:<3}  from factory No {j:<3}  to guest {i:<3}")

print("\n================= Constraints Description ========================")
CC = model.getConstrs()
for CCC in CC:
  print(f"{CCC.ConstrName}")








