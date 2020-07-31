import numpy as np
import glob
from PIL import Image


f_list = glob.glob('data/img/true-*') #True 라벨링된 파일의 경로를 전부 리스트로 만듬
width, height = 150 , 150 # 리사이즈할 높이와 넓이 픽셀수를 정함
data = np.zeros((len(f_list),height,width,3)) # (파일갯수,높이,넓이,색채널) 크기의
                                              # 0으로된 넘파이 배열을 미리 만듬

c=0 #배열에 데이터를 집어 넣기위한 카운터

# print(f_list)
for i in f_list: #파일 경로를 하나씩 처리
    img = Image.open(i).resize(((width,height))) #이미지를 불러오고 미리 정해놓은 사이즈로 리사이즈
    x = np.asarray(img) #이미지를 넘파이 배열로 저장
    # print(x)
    data[c]=x 
    c+=1
    
np.save('./x.npy',data)
# data = np.load('./x.npy')
# print(data.shape)

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
