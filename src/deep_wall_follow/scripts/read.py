import numpy as np
data = np.load('label_data')
d = data['arr_0']
print(d)
print(d.shape)

data1 = np.load('training_data')
d1 = data1['arr_0']
print(d1.shape)