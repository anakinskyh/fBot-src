import numpy as np

z = [0,0,0,0]
g3 = [1,1,0,1]
g2 = [4,4,0,1]
g1 = [13,13,0,1]
g0 = [40,40,0,1] #rgba(241, 196, 15,1.0)

right = np.array([z*20+g0*20])
# right = np.append(right,right)

# right =

right = right.reshape(40,4)

left = right[::-1]

tmp = right
right = left
left = tmp
# right = right.reshape(40,4)

# top move
d0 = [40,40,0,1]
d1 = [13,13,0,1]
d2 = [4,4,0,1]
d3 = [1,1,0,1]

top0 = np.array([z*5+d3*2+d2*4+d1*4+d0*10+d1*4+d2*4+d3*2+z*5])
top0 = top0.reshape(40,4)
