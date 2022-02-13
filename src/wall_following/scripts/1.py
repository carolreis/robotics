from math import cos, sin, atan

# vectors = list(range(0,727))
vectors = [1] * 727


vectors = [x if x < 2 else 2 for x in vectors]

right_bottom_vector = vectors[0:145] # [0, 145] - 145
right_top_vector = vectors[145:290] # [146:291] - 145
front_vector = vectors[290:437] # [291:435] - 144
left_top_vector = vectors[437:582] # [436:581] - 145
left_bottom_vector = vectors[582:727] # [582:727] - 145

intervalos = [(0,144),(145,289),(290,436),(437,581),(582,726)]

vectors[363:726] = [0.5] * len(range(0,363))

print(vectors)

somas_x = []
somas_y = []
degs = []

for intervalo in intervalos:
    somax = 0
    somay = 0
    print(intervalo)
    for i in (range(intervalo[0], intervalo[1]+1)):
        somax += vectors[i] * cos(i * 0.0057)
        somay += vectors[i] * sin(i * 0.0057)

    somas_x.append(somax)
    somas_y.append(somay)
    degs.append(atan(somay/somax))

somax=0
somay=0
for i in range(0,len(vectors)):
    somax += vectors[i] * cos(i * 0.0057)
    somay += vectors[i] * sin(i * 0.0057)
deg = atan(somay/somax)

global_vec = [somax, somay, deg]
print('\nglobal vec:')
print(global_vec)

print('\nsetores:')
print(somas_x)
print(somas_y)
print(degs)

right_bottom = (somas_x[0], somas_y[0], degs[0])
right_top = (somas_x[1], somas_y[1], degs[0])
front = (somas_x[2], somas_y[2], degs[0])
left_top = (somas_x[3], somas_y[3], degs[0])
left_bottom = (somas_x[4], somas_y[4], degs[0])

print(right_bottom)
print(right_top)
print(left_bottom)
print(left_top)
print(front)

""" Casos específicos """
# Casos A:
