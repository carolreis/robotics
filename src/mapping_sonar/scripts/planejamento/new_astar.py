 #! /usr/bin/env python3
import ast
import numpy as np
from math import exp, sqrt, isinf
import operator
import json

''' Exemplo de grafo:
 [
     [
         [ # (0,0)
             # Lista de adjacência => (x,y,heurística)
             [(0, 0, 0), (1, 0, 1), (0, 1, 1), (1, 1, 1.4142135623730951)], 4.242640687119285
         ],
         [ # (0,1)
             [(0, 1, 0), (0, 0, 1), (1, 1, 1), (0, 2, 1), (1, 0, 1.4142135623730951), (1, 2, 1.4142135623730951)], 3.605551275463989
         ], # (0,2)
         [
             [(0, 2, 0), (0, 1, 1), (1, 2, 1), (0, 3, 1), (1, 1, 1.4142135623730951), (1, 3, 1.4142135623730951)], 3.1622776601683795
         ],
         [
             [(0, 3, 0), (0, 2, 1), (1, 3, 1), (0, 4, 1), (1, 2, 1.4142135623730951), (1, 4, 1.4142135623730951)], 3.0
         ],
         [
             [(0, 4, 0), (0, 3, 1), (1, 4, 1), (1, 3, 1.4142135623730951)], 3.1622776601683795
         ]
     ],
     [
         [ # (1,0)
             [(1, 0, 0), (0, 0, 1), (2, 0, 1), (1, 1, 1), (0, 1, 1.4142135623730951), (2, 1, 1.4142135623730951)], 3.605551275463989
         ],
         [ # (1,1)
             [(1, 1, 0), (1, 0, 1), (0, 1, 1), (2, 1, 1), (1, 2, 1), (0, 0, 1.4142135623730951), (2, 0, 1.4142135623730951), (0, 2, 1.4142135623730951), (2, 2, inf)], 2.8284271247461903
         ],
         [ # (1,2)
             [(1, 2, 0), (1, 1, 1), (0, 2, 1), (1, 3, 1), (0, 1, 1.4142135623730951), (2, 1, 1.4142135623730951), (0, 3, 1.4142135623730951), (2, 3, 1.4142135623730951), (2, 2, inf)], 2.23606797749979
         ],
         [
             [(1, 3, 0), (1, 2, 1), (0, 3, 1), (2, 3, 1), (1, 4, 1), (0, 2, 1.4142135623730951), (0, 4, 1.4142135623730951), (2, 4, 1.4142135623730951), (2, 2, inf)], 2.0
         ],
         [
             [(1, 4, 0), (1, 3, 1), (0, 4, 1), (2, 4, 1), (0, 3, 1.4142135623730951), (2, 3, 1.4142135623730951)], 2.23606797749979
         ]
     ]
]
'''
class Graph:

    displacement = [
        (-1,-1), (0,-1), (1,-1),
        (-1, 0), (0,0), (1,0),
        (-1, 1), (0,1), (1,1),
    ]

    def __init__(self, map, goal):
        self.map = map
        print('MAPAAA: ', map)
        self.adjusted_map = []
        self.goal = goal

    def _adjust_map(self, _map):
        map = np.array(_map)
        map = np.where(map >= 0.5, 1, map)
        map = np.where(map < 0.5, 0, map)
        return map

    def build_graph(self):
        self.adjusted_map = self._adjust_map(self.map)
        map = self.adjusted_map
        print('MAP: ', map)

        graph = [[0 for c in map] for row in map]

        for x in range(0,len(map)):
            for y in range(0,len(map[x])):

                l = []
                for d in Graph.displacement:
                    indicex = x+d[0]
                    indicey = y+d[1]

                    if indicex >= 0 and indicey >= 0 and indicex < len(map) and indicey < len(map[0]):

                        # Se não for célula de obstáculo
                        if map[x][y] == 0:

                            if d[0] == 0 and d[1] == 0:
                                custo = 0
                            elif abs(d[0]) == abs(d[1]):
                                custo = sqrt(2)
                            else:
                                custo = 1

                            if map[indicex][indicey] > 0:
                                custo = float('inf')

                            l.append((indicex, indicey,custo))
            
                if len(l) > 0:
                    new_list = sorted(l, key=operator.itemgetter(2))
                    #  Adiciona própria heurística
                    g_dist = sqrt((self.goal[0] - x)**2 + (self.goal[1] - y)**2)
                    graph[x][y] = [new_list, g_dist]
                else:
                    graph[x][y] = [[], float('inf')]

        return graph

class AStar:

    def __init__(self, start, goal, _map):
        self.current_queue = []
        self.done_queue = []

        self.graph = []

        self.start = start
        self.goal = goal
        self.map = _map

    class Node:
        def __init__(self, x, y, cost, parent_index, heuristica, sibilings):
            self.x = x  # index of grid
            self.y = y  # index of grid
            self.cost = cost
            self.parent_index = parent_index
            self.heuristica = heuristica
            self.sibilings = sibilings

        def __str__(self) -> str:
            return "x: " + str(self.x) + ", y: " + str(self.y) + ", cost: " + str(
                self.cost) + ", parent: " + str(self.parent_index) + ", heuristic: " + str(
                self.heuristica) + ", sibilings: " + str(self.sibilings)

    def _total_cost(self, node):
        return node.cost + node.heuristica

    def path_planning(self):

        g = Graph(self.map, self.goal)
        graph = g.build_graph()
        print("GRAPH: ", graph)

        self.map = g.adjusted_map
        self.graph = graph

        print("START: ", self.start)
        print("GOAL: ", self.goal)

        start_x = self.start[0]
        start_y = self.start[1]
        start_node_data = graph[start_x][start_y]  #  [(x, y, custo), (x, y, custo)], heuristica_no_atual)
        start_node = self.Node(
            x=start_x,
            y=start_y,
            cost=0,
            parent_index=(start_x, start_y),
            heuristica=start_node_data[1],
            sibilings=start_node_data[0]
        )

        goal_node = self.Node(
            x=self.goal[0],
            y=self.goal[1],
            cost=0,
            parent_index=(start_x, start_y),  # Não sei se isso vai dar certo, no exemplo o cara usou -1
            heuristica=0,
            sibilings=[]  # Pensar melhor sobre
        )

        self.current_queue.append(start_node)

        print("\nLista: ")
        for item in self.current_queue:
            print(item)

        while True:

            if len(self.current_queue) == 0:
                print("Pilha vazia!")
                break

            # Ordenar pilha
            # print("\nOrdenando pilha")
            self.current_queue.sort(key=self._total_cost)
            # print("...Pilha: ", self.current_queue[0])

            current = self.current_queue[0]  # Nó atual

            # Verifica se o atual é o objetivo
            if current.x == goal_node.x and current.y == goal_node.y:
                print("\n ==== FOUND! ==== \n")
                goal_node.parent_index = current.parent_index
                goal_node.cost = current.cost
                self.done_queue.append(goal_node)
                break
        
            # print("Não é o objetivo.")

            # Coloca na lista de pronto
            # print("Adiciona na lista de prontos")
            self.done_queue.append(self.current_queue[0])

            # Remove atual
            # print("Remove da lista atual")
            del self.current_queue[0]

            # print("Atual: ", current)
            # Expande os filhos
            # print("Expande os filhos...")
            for sibiling in current.sibilings:
                '''
                    sibiling: (x,y,custo)
                    custo = em relação ao pai, não é a heurística
                '''
                node_x = sibiling[0]
                node_y = sibiling[1]
                node = self.Node(
                    x = node_x,
                    y = node_y,
                    cost = sibiling[2],
                    parent_index = (current.x, current.y),
                    heuristica = graph[node_x][node_y][1],  # Busca heurística na sua estrutura de dados lá nos grafos
                    sibilings = graph[node_x][node_y][0]  # Busca adjacentes lá nos grafos
                )

                #  TODO: Mudar para que não precise fazer um for, usar um dict talvez

                # Pesquisa na lista de prontos
                in_done = False
                for done_node in self.done_queue:
                    #  Verifica se o nó já está na lista de prontos
                    if node.x == done_node.x and node.y == done_node.y:
                        in_done = done_node
                        break

                if in_done:
                    print('in done: ', in_done.x, ' ', in_done.y)
                    continue

                # Pesquisa na lista atual
                in_current = False
                current_index = 0
                for curr_node in self.current_queue:
                    if node.x == curr_node.x and node.y == curr_node.y:
                        in_current = curr_node
                        current_index += 1
                        break
                if not in_current:
                    self.current_queue.append(node)
                else:
                    if in_current.cost > node.cost:
                        self.current_queue[current_index] = node

        print("Lista de prontos: ")
        path = []
        for done in self.done_queue:
            print("Pronto: ", done.x, done.y)
            if (done.x,done.y) not in path:
                path.append((done.x,done.y))

        return path

if __name__ == "__main__":
    mapa = [
        [0, 0, 0, 0, 0],
        [0, 0, 0, 0, 0],
        [0, 0, 15, 0, 0],
        [0, 0, 15, 0, 0],
        [0, 0, 15, 0, 0]
    ]

    start = (4,0)
    goal = (4,4)
    a = AStar(start, goal, mapa)
    path = a.path_planning()
    print('path: ', path)