import numpy as np
import tensorflow as tf
from keras.models import Sequential
from keras.layers import Conv2D,MaxPool2D,Dropout,Dense,Flatten
from tensorflow_core import metrics

true = np.load('./x.npy')
false = np.load('./y.npy')

t = true.shape[0]
f = false.shape[0]

x = np.concatenate((true,false),axis=0)
y = np.zeros((x.shape[0],))
y[t:]=1
# print(data.shape)#(1625, 150, 150, 3)

model = Sequential()

model.add(Conv2D(128, (3, 3), strides=1,
                 padding='valid', activation='relu', input_shape=(150, 150, 3)))
model.add(Conv2D(64, (3, 3), strides=1
                 , activation='relu',padding='valid'))
model.add(MaxPool2D(2,2))
model.add(Dropout(0.2))
model.add(Conv2D(32, (3, 3), strides=1,
                 padding='valid', activation='relu'))
model.add(Conv2D(32, (3, 3), strides=1,
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

model.fit(x,y,batch_size = 30, epochs=30, validation_split=0.3)


