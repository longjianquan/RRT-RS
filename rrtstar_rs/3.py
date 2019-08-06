import matplotlib.pyplot as plt
import numpy as np
import math

a=np.loadtxt("/home/ljq/qt/rrtstar_rs/obs.txt")

f=plt.figure();
ax=f.add_subplot(111)
plt.xlim(-1 ,501 )
plt.ylim(-1 ,501 )

plt.scatter(a[:,0],a[:,1])
plt.show()

