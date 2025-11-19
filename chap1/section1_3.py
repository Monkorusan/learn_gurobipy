# 1.2: Integer Linear Programming
from gurobipy import *
model = Model("puzzle")
x = model.addVar(name="x",vtype="I")
y = model.addVar(name="y",vtype="I")
z = model.addVar(name="z",vtype="I")
model.update()
model.addConstr(     x +    y +    z == 32)
model.addConstr(   2*x +  4*y +  8*z == 80)
model.setObjective(         y +    z, GRB.MINIMIZE)
model.optimize()
print(f"\nmin(y+z) = {model.ObjVal}\n")
print(f"(x_opt,y_opt,z_opt) = {x.X}, {y.X}, {z.X}")
# print([attr for attr in dir(x) if not attr.startswith('__')]) #print attribute of x (there will be X where opt ans is stored)