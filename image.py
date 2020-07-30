import numpy as np
import glob
from PIL import Image


f_list = glob.glob('data/img/true-*')
width, height = 150 , 150
data = np.zeros((len(f_list),height,width,3))
c=0
# print(f_list)
for i in f_list:
    img = Image.open(i).resize(((width,height)))
    x = np.asarray(img)
    # print(x)
    data[c]=x
    c+=1
    
np.save('./x.npy',data)
data = np.load('./x.npy')
print(data.shape)

# f_list = glob.glob('data/img/false*')
# width, height = 150, 150
# data = np.zeros((len(f_list), height, width, 3))
# c = 0
# # print(f_list)
# for i in f_list:
#     img = Image.open(i).resize(((width, height)))
#     x = np.asarray(img)
#     # print(x)
#     data[c] = x
#     c += 1

# np.save('./y.npy', data)
