import numpy as np

z = [0,0,0,0]
g3 = [0,1,0,1]
g2 = [0,4,0,1]
g1 = [0,13,0,1]
g0 = [0,40,0,1] #rgba(241, 196, 15,1.0)

left = np.array(z*5+g3*5+g2*4+g1*3+g0*3)
left = np.append(left,left)

# right = 

left = left.reshape(40,4)

right = left[::-1]

tmp = left
left = right
right = tmp
# right = right.reshape(40,4)

# top move
d0 = [0,40,0,1]
d1 = [0,13,0,1]
d2 = [0,4,0,1]
d3 = [0,1,0,1]

top0 = np.array([z*5+d3*2+d2*4+d1*4+d0*10+d1*4+d2*4+d3*2+z*5])
top0 = top0.reshape(40,4)