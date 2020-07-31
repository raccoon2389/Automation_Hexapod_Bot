import numpy as np
import tensorflow as tf
from tensorflow.keras.models import Sequential
from tensorflow.keras.layers import Conv2D, MaxPool2D, Dropout, Dense, Flatten
from tensorflow_core import metrics
from sklearn.model_selection import train_test_split


true = np.load('./x.npy') #True의 데이터값 로드
false = np.load('./y.npy') #False의 데이터값 로드

t = true.shape[0] #True와 False의 배치사이즈 저장
f = false.shape[0]

x = np.concatenate((true,false),axis=0)/255. # True와 False 의 데이터 값을 하나의 배열로 묶는다
# print(x.shape)  # (1625, 150, 150, 3)
y = np.zeros((x.shape[0],)) #데이터의 배치사이즈 길이의 1차원 벡터를 만들어준다

y[:t] = 1 # true까지의 라벨링을 1로 해준다

x,x_test , y, y_test = train_test_split(x,y,test_size=0.2,shuffle=True, random_state=666) 
#train 과 test 분리해줌



model = Sequential()

model.add(Conv2D(128, (2, 2), strides=1,
                 padding='valid', activation='relu', input_shape=(150, 150, 3)))
model.add(Conv2D(64, (2, 2), strides=1
                 , activation='relu',padding='valid'))
model.add(MaxPool2D(2,2))
model.add(Dropout(0.2))
model.add(Conv2D(32, (2, 2), strides=1,
                 padding='valid', activation='relu'))
model.add(Conv2D(32, (2, 2), strides=1,
                 padding='valid', activation='relu'))
model.add(MaxPool2D(2, 2))
model.add(Conv2D(16, (2, 2), strides=1,
                 padding='valid', activation='relu'))
model.add(Conv2D(8, (2, 2), strides=1,
                 padding='valid', activation='relu'))
model.add(MaxPool2D(2, 2))
model.add(Conv2D(8, (2, 2), strides=1,
                 padding='valid', activation='relu'))
model.add(Conv2D(4, (2, 2), strides=1,
                 padding='valid', activation='relu'))
model.add(MaxPool2D(2, 2))
model.add(Dropout(0.2))
model.add(Flatten())
model.add(Dense(256,activation='relu'))
model.add(Dense(128,activation='relu'))
model.add(Dense(64, activation='relu'))
model.add(Dense(1, activation='sigmoid'))
model.summary()
model.compile(optimizer='adam',loss='binary_crossentropy',metrics=['acc'])

model.fit(x,y,batch_size = 30, epochs=30, validation_split=0.3,shuffle=True)

loss = model.evaluate(x_test,y_test)
print(f"loss : {loss[0]}\nacc : {loss[1]}")
