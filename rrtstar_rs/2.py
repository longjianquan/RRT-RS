import matplotlib.pyplot as plt
from PIL import Image
import numpy as np
img=Image.open("/home/ljq/qt/rrtstar_rs/3.png")
img = img.convert('L')
img = np.array(img)
cc=[];
for i in range(img.shape[0]):
	for j in range(img.shape[1]):
		if img[i,j]==255 :
			img[i,j]=0
		else:
			img[i,j]=1
			cc.append([i,j])


np.savetxt('/home/ljq/qt/rrtstar_rs/map.txt',img)		
np.savetxt('/home/ljq/qt/rrtstar_rs/obs.txt',cc)

