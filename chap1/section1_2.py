from gurobipy import *
model = Model("lo1")
x1 = model.addVar(name="x1")
x2 = model.addVar(name="x2")
x3 = model.addVar(name="x3")
model.update()
model.addConstr(    2*x1 +    x2 +    x3 <= 60)
model.addConstr(      x1 +  2*x2 +    x3 <= 60)
model.addConstr(                      x3 <= 30)
model.setObjective(15*x1 + 18*x2 + 30*x3, GRB.MAXIMIZE)
model.optimize()
print(f"Optimal Value = {model.ObjVal}")
for i in model.getVars():
    print(i.VarName,i.X)