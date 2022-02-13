import numpy as np
import json

a = np.load('mapa_para_potencial.npz')
a = a['arr_0']

l = a.tolist()
print(l)
