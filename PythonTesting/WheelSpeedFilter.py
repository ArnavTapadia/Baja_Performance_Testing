import numpy as np
import scipy as sp

# importing .dat file

data = np.genfromtxt('/Users/arnav/Google Drive/Cornell/Baja Racing/Performance Testing/Sensor Processing/Data/overrun 2wd 4.dat')
print('hello')
# print(data)
print(np.shape(data))
print('finding type')
print(type(data))

# we care about column 2 and 4 (v0 and v2)
# i think v1 and v3 are just v2 and v4 shifted 90 degrees out of phase
# y0 - y3 are just repeated versions of v0-v3

time = data[:,0]
print(time)

# x = [[5,5,5,2,5,2,5],[5,5,5,2,5,2,5]]
# print(x[1][3])