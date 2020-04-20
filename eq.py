from sympy import symbols, Eq, solve
import numpy as np



"""
u = [ [0,0] , [1,1] ]
v = [ [4,0] , [-1,1] ] 
t, s = symbols('t s')
eq1 = Eq( (u[1][0] * ( t*u[1][0] + u[0][0] - s*v[1][0] - v[0][0] ) + u[1][1] * ( t*u[1][1] + u[0][1] - s*v[1][1] - v[0][1] ) ) , 0 )
eq2 =  Eq( (v[1][0] * ( t*u[1][0] + u[0][0] - s*v[1][0] - v[0][0] ) + v[1][1] * ( t*u[1][1] + u[0][1] - s*v[1][1] - v[0][1] ) ) , 0 )
t_s = solve((eq1,eq2),(t,s))
print(len(t_s))
"""


print("{0:5d}0".format(25))
print("======Workspace Dimensions=========")
print("______________________________________________________")
print("|[1]                    [2]                       [3]|")
print("|                                                    |")
print("|                                                    |")
print("|                                                    |")
print("|                                                    |")
print("|                                                    |")
print("|                                                    |")
print("|[4]______________________________________________[5]|")
        