## temporarily skip section 5.1 and 5.2

""" ==================== Formulation =============================
minimize sum{ dist[ij]*x_ij }
    s.t. sum_j x_ij = 1 for all i in range(n) if i!=j
         sum_j x_ji = 1 for all i in range(n) if i!=j

## NOTE = for crazy large problem, itz better to model.addVar in 
#         calc dist matrix to prevent 2n^2 big O
"""
import numpy as np
import matplotlib.pyplot as plt
import gurobipy as grbpy

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

def formulate(X:np.ndarray,Y:np.ndarray):
    model = grbpy.Model("Travelling Salesman Problem")
    for x in range(len(X)):
        for y in range(len(Y)):
            if x == y:
                pass
            else:
                model.addVar(vtype=grbpy.GRB.BINARY , name=f"d_[{x+1},{y+1}]")

    return model
    


def main():
    n=10
    X=np.random.rand(n)
    Y=np.random.rand(n)
    print(f"distance matrix = {calc_dist_matrix(X,Y,round_decimal = 2)}")
    plt.figure(1)
    plt.scatter(X,Y)
    plt.show()

if __name__ == "__main__":
    main()
