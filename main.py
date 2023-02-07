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
    def __init__(self, _straightDist, _realDist) -> None:
        self.matrix = _straightDist
        self.graph = _realDist
    
    def children(self, node):
        return self.graph[node]

    def heuristic(self, currNode, no_dest):
        return self.matrix[currNode][no_dest]
        
    def build_way(self, start, goal):
        visited = []
        parents = []
        cost = {start: 0}

        pq = [(0, start)]
        while pq:
            (_, currNode) = heapq.heappop(pq)
            if currNode == goal:
                parents.append(currNode)
                break

            visited.append(currNode)
            children_aux = self.children(currNode)
            
            for subCurrNode, dist, _ in children_aux:    
                if subCurrNode in visited:
                    if len(children_aux) == 1:
                        print(f'No way out in {subCurrNode} -> {currNode}! Try another way')
                    continue
                
                cost_aux = cost[currNode] + dist # G(n)
                if subCurrNode not in cost or cost_aux < cost[subCurrNode]:
                    cost[subCurrNode] = cost_aux
                    f = cost_aux + self.heuristic(int(subCurrNode[1:]) - 1, int(goal[1:]) - 1) # f(n)

                    heapq.heappush(pq, (f, subCurrNode))

                    if currNode not in parents:
                        parents.append(currNode)
        
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
            print(f'E{way[x]}')
        else:
            print(f'E{way[x]} ->', end=' ')
    print(f'O tempo estimado de {start} -> {target} é de: {cost[target]} min')
    print('*'*44)