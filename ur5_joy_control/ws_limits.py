import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits import mplot3d
from shapely.geometry import Point
from shapely.geometry.polygon import Polygon

np.random.seed(42)

boxes = np.random.rand(1,4,2)*10
boxes = np.concatenate((boxes,np.zeros((1,4,1))),axis=2)

heights = np.array([10,3])

nbox = np.array([[[5,3,2],\
                [5,9,2],\
                [7,9,2],\
                [7,3,2]]])

boxes = np.concatenate((boxes,nbox),axis=0)
print("Boxes: ")
print(boxes)


ax = plt.axes(projection='3d')


for i in range(boxes.shape[0]):
    x = np.append(boxes[i,:,0],boxes[i,0,0])
    x = np.append(x,x)
    y = np.append(boxes[i,:,1],boxes[i,0,1])
    y = np.append(y,y)
    z = np.append(boxes[i,:,2],boxes[i,0,2])
    z = np.append(z,z+heights[i])
    ax.plot3D(x,y,z)

### Check if point is inside or outside of quadrilateral

p = np.array([6,8,2.5])

ax.plot3D([p[0]],[p[1]],[p[2]],'.r')

polygons = [Polygon(boxes[k,:,:2]) for k in range(boxes.shape[0])]

### Loop applying speed to point + boundary checking

v = np.array([-0.01,0.01,0])*10

t = 0
T = 0.05 # 50ms
n = 0

while t < 1000*T:
    
    if n > 10:
        n = 0
        # Change velocity:
        v = (2*np.random.rand(3)-1)/100*10
        
    p = p + v 
    point = Point(p[0],p[1])
    
    inside = False
    for i in range(boxes.shape[0]):
        z_min = boxes[i,0,-1]
        z_max = z_min + heights[i]
        if polygons[i].contains(point) and z_min < p[2] and p[2] < z_max:
            inside = True
            
    #print(inside)
    
    if not inside:
        p = p - 2*v
        ax.plot3D([p[0]],[p[1]],[p[2]],'*b')   
        
    else:
        ax.plot3D([p[0]],[p[1]],[p[2]],'.r')
    
    n += 1
    t += T
    
    plt.pause(T)

plt.show()
