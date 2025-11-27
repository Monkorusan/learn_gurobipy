import numpy as np
import gurobipy as grbpy
n = 20 # num of destinations
m = 5  # num of vehicles
Q = 2  # capacity of all vehicles combined

# a list of demands by each guest, assume random
q = [0] + np.random.randint(1, 2, n).tolist()  # Depot has 0 demand


demand_sum = np.sum(q)
min_vehicles = np.ceil(demand_sum / Q)
# print(min_vehicles)

# print(q)
# if np.max(q)>Q*m:
#     print("infeasible")
# else:
#     print("all fine")

model = grbpy.Model('mymodel')
x = {}
print(type(x))