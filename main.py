import heapq
import argparse

def cli() -> str:
    parser = argparse.ArgumentParser()

    parser.add_argument('-w', '--way', type=str, required=True, nargs='+',
                        help='Escreva a origem e destina da pesquisa')

    arg = parser.parse_args()
    _origin, _dest = arg.way

    return _origin, _dest

class AStar:
    def __init__(self, _straight_dist, _real_dist) -> None:
        self.matrix = _straight_dist
        self.graph = _real_dist
    
    def get_neighbors(self, root_node):
        return self.graph[root_node]

    def heuristic(self, _root_node, _goal):
        return self.matrix[_root_node][_goal]
    
    def change_line(self, _parents, _neighbor_color):
        _transfer = 0
        
        if _parents:
            if _parents[len(_parents) - 1][1] != _neighbor_color:
                _transfer = 4
    
        return _transfer

    def connect(self, root_node, parents, auxColor) -> bool:
        connect = False
        if parents:
            for (auxNode, _, color) in self.graph[parents[len(parents)-1][0]]:
                if auxNode == root_node and color == auxColor:
                    connect = True
        else:
            connect = True     
        
        return connect
    
    
    def build_way(self, start, goal):
        visited = []
        parents = []
        cost = {start: 0}
        transfer = 0

        pq = [(0, start, '')]
        while pq:
            print(f'Fronteira: {pq}\n')
            (_, root_node, curr_color) = heapq.heappop(pq)

            # Só vamos expandir a fronteira se o nó analisado pela heap seja conectado com o anterior
            if self.connect(root_node, parents, curr_color):
                if root_node == goal:
                    parents.append((root_node, curr_color))
                    break

                visited.append(root_node)
                neighbors = self.get_neighbors(root_node)
                
                for border_node, dist, neighbor_color in neighbors:
                    if border_node in visited:
                        if len(neighbors) == 1:
                            print(f'*** No way out in {border_node} -> {root_node}! Try another way ***\n')
                        continue
                    
                    cost_aux = cost[root_node] + dist # G(n)
                    if border_node not in cost or cost_aux < cost[border_node]:
                        check = False
                        for node, _ in parents:
                            if node == root_node:
                                check = True
                        
                        if not check:
                            # a cor do nó inicial da busca será inicializado com a cor referente ao seu vizinho
                            if not parents: 
                                parents.append((root_node, neighbor_color)) 
                            else:
                                parents.append((root_node, curr_color))      
                        
                        # checando situação de baldeação de linha
                        transfer = self.change_line(parents, neighbor_color)
                        cost[border_node] = cost_aux + transfer
                        
                        f = cost[border_node] + self.heuristic(int(border_node[1:]) - 1, int(goal[1:]) - 1) # f(n)
                        heapq.heappush(pq, (f, border_node, neighbor_color))
                    
        return parents, cost


# Grafo com as distâncias multiplicadas por 2 para obter o tempo
realDistances = {
    'E1' : [('E2', 20, 'azul')],
    'E2' : [('E3', 17, 'azul'), ('E1', 20, 'azul'), ('E9', 20, 'amarela'), ('E10', 7, 'amarela')],
    'E3' : [('E2', 17, 'azul'), ('E4', 12.6, 'azul'), ('E9', 18.8, 'vermelha'), ('E13', 37.4, 'vermelha')],
    'E4' : [('E3', 12.6, 'azul'), ('E5', 26, 'azul'), ('E8', 30.6, 'verde'), ('E13', 25.6, 'verde')],
    'E5' : [('E4', 26, 'azul'), ('E6', 6, 'azul'), ('E7', 4.8, 'amarela'), ('E8', 60, 'amarela')],
    'E6' : [('E5', 6, 'azul')],
    'E7' : [('E5', 4.8, 'amarela')],
    'E8' : [('E5', 60, 'amarela'), ('E4', 30.6, 'verde'), ('E9', 19.2, 'amarela'), ('E12', 12.8, 'verde')],
    'E9' : [('E8', 19.2, 'amarela'), ('E2', 20, 'amarela'), ('E3', 18.8, 'vermelha'), ('E11', 24.4, 'vermelha')],
    'E10': [('E2', 7, 'amarela')],
    'E11': [('E9', 24.4, 'vermelha')],
    'E12': [('E8', 12.8, 'verde')],
    'E13': [('E3', 37.4, 'vermelha'), ('E4', 25.6, 'verde'), ('E14', 10.2, 'verde')],
    'E14': [('E13', 10.2, 'verde')]
}


straightDistances = [
        [0,    10,   18.5, 24.8, 36.4, 38.8, 35.8, 25.4, 17.6, 9.1,  16.7, 27.3, 27.6, 29.8], 
        [10,   0,    8.5,  14.8, 26.6, 29.1, 26.1, 17.3, 10,   3.5,  15.5, 20.9, 19.1, 21.8], 
        [18.5, 8.5,  0,    6.3,  18.2, 20.6, 17.6, 13.6, 9.4,  10.3, 19.5, 19.1, 12.1, 16.6], 
        [24.8, 14.8, 6.3,  0,    12,   14.4, 11.5, 12.4, 12.6, 16.7, 23.6, 18.6, 10.6, 15.4], 
        [36.4, 26.6, 18.2, 12,   0,    3,    2.4,  19.4, 23.3, 28.2, 34.2, 24.8, 14.5, 17.9], 
        [38.8, 29.1, 20.6, 14.4, 3,    0,    3.3,  22.3, 25.7, 30.3, 36.7, 27.6, 15.2, 18.2], 
        [35.8, 26.1, 17.6, 11.5, 2.4,  3.3,  0,    20,   23,   27.3, 34.2, 25.7, 12.4, 15.6], 
        [25.4, 17.3, 13.6, 12.4, 19.4, 22.3, 20,   0,    8.2,  20.3, 16.1, 6.4,  22.7, 27.6], 
        [17.6, 10,   9.4,  12.6, 23.3, 25.7, 23,   8.2,  0,    13.5, 11.2, 10.9, 21.2, 26.6],
        [9.1,  3.5,  10.3, 16.7, 28.2, 30.3, 27.3, 20.3, 13.5, 0,    17.6, 24.2, 18.7, 21.2], 
        [16.7, 15.5, 19.5, 23.6, 34.2, 36.7, 34.2, 16.1, 11.2, 17.6, 0,    14.2, 31.5, 35.5], 
        [27.3, 20.9, 19.1, 18.6, 24.8, 27.6, 25.7, 6.4,  10.9, 24.2, 14.2, 0,    28.8, 33.6], 
        [27.6, 19.1, 12.1, 10.6, 14.5, 15.2, 12.4, 22.7, 21.2, 18.7, 31.5, 28.8, 0,    5.1],  
        [29.8, 21.8, 16.6, 15.4, 17.9, 18.2, 15.6, 27.6, 26.6, 21.2, 35.5, 33.6, 5.1,  0]     
    ]


if __name__ == '__main__':
    multiply = lambda arr : [[i*2 for i in linha]for linha in arr]
    aStar = AStar(multiply(straightDistances), realDistances)

    start, target = cli()
    way, cost = aStar.build_way(start, target)

    print('*'*44)
    print('O caminho encontrado foi:')
    for x in range(len(way)):
        if x == len(way)-1:
            print(f'({way[x][0]}, {way[x][1]})')
        else:
            print(f'({way[x][0]}, {way[x][1]}) ->', end=' ')
    print(f'\nCustos: {cost}')
    print(f'O tempo estimado de ({start}) -> ({target}) é de: {cost[target]}min')
    print('*'*44)