import quatmath


def dot(t1: tuple, t2: tuple):
    return t1[0]*t2[0] + t1[1]*t2[1] + t1[2]*t2[2] + t1[3]*t2[3]


pi = 3.1415926535

print(dot((0.020826, 0.758795, -0.650521, -0.0248716), (0.0193663, 0.661348, -0.748369, -0.0467608)))

#print(quatmath.mult(quatmath.getchildlocalrot((2,5,1,1), (2,5,1,1)), quatmath.getchildlocalrot((2,5,1,1),(2,5,1,1))))

