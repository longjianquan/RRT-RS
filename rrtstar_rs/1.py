import matplotlib.pyplot as plt
import numpy as np
import math

a=np.loadtxt("/home/ljq/qt/rrtstar_rs/getpath.txt")
b=np.loadtxt("/home/ljq/qt/rrtstar_rs/obs.txt")
f=plt.figure();
ax=f.add_subplot(111)
ax.set(xlim=[0, 500], ylim=[0, 500])
	
plt.plot(a[0,0],a[0,1],'ro')
plt.arrow(a[0,0], a[0,1], 50.0 * math.cos(a[0,2]), 50.0 * math.sin(a[0,2]),
                  fc="r", ec="k", head_width=50, head_length=50)
plt.plot(a[len(a)-1,0],a[len(a)-1,1],'yo')
plt.arrow(a[len(a)-1,0], a[len(a)-1,1], 50.0 * math.cos(a[len(a)-1,2]), 50.0 * math.sin(a[len(a)-1,2]),
                  fc="r", ec="k", head_width=50, head_length=50)
plt.plot(a[:,0],a[:,1],'-')
plt.scatter(b[:,0],b[:,1])
plt.show()
