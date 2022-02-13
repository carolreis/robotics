# -*- coding: utf-8 -*-
import math
import numpy as np
import matplotlib.pyplot as plt
from matplotlib import animation
from numpy.core.fromnumeric import argmin
from numpy.lib.shape_base import array_split
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.animation import PillowWriter
from matplotlib import cm
from skimage import color
from skimage import io
import json
from math import sqrt
import time


class HarmonicPotentialField:

    VALOR_CONTORNO = 1
    VALOR_OBJETIVO = -50
    VALOR_LIVRE = 0.8
    # VALOR_LIVRE = 0.999999999999

    def __init__(self, cells, goal=(0,0)):
        self.mapa = cells
        self.goal = goal
        self.mapa_shape = self.mapa.shape

    def normaliza_gradiente(self, dx, dy, gradients):

        new_dx = []
        new_dy = []

        for i in range(0, len(gradients)):
            for j in range(0, len(gradients[i])):

                modulo = sqrt(gradients[i][j][0]**2 + gradients[i][j][1]**2)

                try:
                    new_x = dx[i][j] / modulo
                    new_x == 0.0 if new_x == -0.0 else new_x

                except FloatingPointError as e:
                    new_x = 0

                try:
                    new_y = dy[i][j] / modulo
                    new_y == 0.0 if new_y == -0.0 else new_y

                except FloatingPointError as e:
                    new_y = 0

                new_dx.append(new_x)
                new_dy.append(new_y)

        new_dx = np.array(new_dx)
        new_dy = np.array(new_dy)

        new_dx = new_dx.reshape(self.mapa_shape)
        new_dy = new_dy.reshape(self.mapa_shape)

        return new_dx, new_dy

    def compute_potential(self, mapa, n_iter):
        length = len(mapa[0])
        for n in range(n_iter):
            for i in range(1, length-1):
                for j in range(1, length-1):

                    if mapa[j][i] != 1:
                        mapa[j][i] = 1/4 * (mapa[j+1][i] + mapa[j-1][i] + mapa[j][i+1] + mapa[j][i-1])
            potencial = mapa
        return potencial

    def calc(self):
        #  Obstacles
        mapa = np.where(self.mapa >= 1, HarmonicPotentialField.VALOR_CONTORNO, self.mapa)
        #  Free cells
        mapa = np.where(self.mapa == 0, HarmonicPotentialField.VALOR_LIVRE, self.mapa)
        #  Goal
        # mapa[self.goal[0]][self.goal[1]] = HarmonicPotentialField.VALOR_OBJETIVO

        #  Contornos - bordas do mapa
        # mapa[0,:]= HarmonicPotentialField.VALOR_CONTORNO
        # mapa[-1,:]= HarmonicPotentialField.VALOR_CONTORNO
        # mapa[:,0]= HarmonicPotentialField.VALOR_CONTORNO
        # mapa[:,-1]= HarmonicPotentialField.VALOR_CONTORNO

        #  Calcula o campo potencial
        potential = self.compute_potential(mapa, n_iter=10)

        gradient = np.gradient(potential)
        ax = -1 * gradient[1] # Faz *(-1) porque quero o gradiente decrescente  - é invertido
        ay = -1 * gradient[0] # Faz *(-1) porque quero o gradiente decrescente  - é invertido
        dx, dy = (ax,ay)

        gradients = np.dstack((dx,dy))
        # print('gradientes: ', gradients)
        # print('gradientes shape: ', gradients.shape)

        new_dx, new_dy = self.normaliza_gradiente(dx, dy, gradients)

        # linspc = np.linspace(0, 1, self.mapa_shape[0])
        linspc = np.linspace(0, 1, self.mapa_shape[0])

        # print('shape dx dy: ', (new_dx.shape, new_dy.shape))
        # print('linspc: ', linspc)

        xx = np.outer(linspc, np.ones(self.mapa_shape[0])).T
        # print('xx: ', xx)
        # print('xx shape: ', xx.shape)

        yy = np.outer(linspc, np.ones(self.mapa_shape[0]))
        # print('yy: ', yy)
        # print('y shape: ', yy.shape)

        plt.quiver(xx,yy,new_dx,new_dy, color="blue")
        # plt.quiver(yy,xx,new_dx,new_dy, color="blue")

        # plt.rc('text')
        # plt.rc('font', family='sans-serif')
        # plt.rc('xtick',labelsize=10)
        # plt.rc('ytick',labelsize=10)

        # plt.figure(figsize=(40,40))
        # ax = plt.axes(projection='3d')
        # ax.set_xlabel('x', fontsize = 10)
        # ax.set_ylabel('y',fontsize = 10)
        # ax.set_zlabel('f(x,y)',fontsize = 10)
        # ax.plot_surface(xx, yy, potential,cmap='nipy_spectral', edgecolor='none')

        # plt.show()

        return new_dx, new_dy, potential

if __name__ == '__main__':
    mapa = [
        [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
        [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
        [0, 3, 0, 0, 3, 0, 0, 0, 0, 0, 0, 0, 0],
        [0, 3, 0, 0, 3, 0, 0, 3, 3, 0, 0, 0, 0],
        [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
        [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
        [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
        [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
        [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
        [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
        [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
        [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
        [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
    ]
    mapa = np.array(mapa)
    hpf = HarmonicPotentialField(mapa)
    hpf.calc()
