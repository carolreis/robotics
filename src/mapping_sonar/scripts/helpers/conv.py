import numpy as np
KERNEL = [
    [0.5, 0.5, 0.5],
    [0.5, 1, 0.5],
    [0.5, 0.5, 0.5]
]
MATRIX = [
    [1  ,2  ,3  ,4  ,5], # 0
    [6  ,7  ,8  ,9  ,10], # 1
    [11 ,12 ,13 ,14 ,15], # 2
    [16 ,17 ,18 ,19 ,20], # 3
    [21 ,22 ,23 ,24 ,25] # 4
]
'''
Para os do centro da matriz:
L: 1,2,3
    C: 1,2,3
        Para o filtro e os elemntos que serao usados
        L-1: 
            C: 0,1,2
        L:
            C: 0,2
        L+1: 
            C: 0,1,2
'''

for slide in range(0,int(len(MATRIX)/len(KERNEL))):
    for l in range(1,len(MATRIX)-1): # [1,3]
        for c in range(1,len(MATRIX)-1): # [1,3]
            l = l + slide
            c = c + slide

            print("Elementos da matriz: ", MATRIX[l][c])
            atual = MATRIX[l][c]

            somatorio = 0

            intervalos = [l-1,l,l+1]
            for i in range(0,len(intervalos)):
                indice_linha_matriz = intervalos[i]

                # Pega elementos da linha anterior:
                print("Elm linha anterior: ", MATRIX[indice_linha_matriz])
                print("Elm linha atual: ", MATRIX[indice_linha_matriz])
                print("Elm linha posterior: ", MATRIX[indice_linha_matriz])

                intervalo_coluna = [c-1, c, c+1]

                for ic in range(0,len(intervalo_coluna)):

                    indice_coluna_matriz = intervalo_coluna[ic]
                    # if (l,c) != (i,indice_coluna_matriz):

                    print('KERNEL: ', KERNEL[i][ic])
                    print("ELM C: ", MATRIX[indice_linha_matriz][indice_coluna_matriz])

                    somatorio += KERNEL[i][ic] * MATRIX[indice_linha_matriz][indice_coluna_matriz]

                print('\n')

            MATRIX[l][c] = somatorio
            print("nova matriz: ", MATRIX)
            print('\n')

        print('\n')
    print('\n')


