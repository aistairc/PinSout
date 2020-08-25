# class Graph:
#
#     def __init__(self, edges, N):
#         self.adjList = [[] for _ in range(N)]
#
#         for (src, dest) in edges:
#             self.adjList[src].append(dest)
#             self.adjList[dest].append(src)
#
# def DFS(graph, v, discovered, parent):
#
#     discovered[v] = 1
#
#     for w in graph.adjList[v]:
#
#         if discovered[w] == 0:
#
#             if DFS(graph, w, discovered, v):
#                 print w
#                 return 1
#         elif w != parent:
#             print w
#             return 1
#     return 0
# if __name__ == '__main__':
#     #
#     # edges = [[0, 1], [0, 2], [2, 3], [3, 4], [3, 5], [5, 6], [5, 7], [5, 8], [8, 9], [8, 10], [10, 11], [11, 12],
#     #          [11, 4], [11, 13], [13, 14], [14, 15], [14, 16], [14, 17], [17, 18], [18, 19], [19, 20], [20, 21],
#     #          [20, 22], [22, 23], [23, 21], [23, 24], [24, 25], [24, 26], [24, 27], [26, 15], [26, 28], [28, 29],
#     #          [25, 30], [25, 21], [30, 31], [30, 32], [30, 33], [33, 34], [34, 35], [32, 36], [32, 16], [36, 37],
#     #          [36, 38], [38, 15], [38, 39], [38, 40], [40, 41], [39, 42], [39, 31], [42, 43], [42, 16], [43, 44],
#     #          [44, 45], [45, 46], [46, 6], [37, 47], [47, 1], [31, 48], [15, 49], [49, 50], [50, 4], [12, 1], [12, 51],
#     #          [12, 52], [52, 53], [53, 51], [53, 4]]

#     # Set number of vertices in the graph
#     N = len(edges)
#
#     # create a graph from edges
#     graph = Graph(edges, N)
#
#     # stores vertex is discovered or not
#     discovered = [0] * N
#     k = []
    # Do DFS traversal from first vertex
#     # print DFS(graph, 0, discovered, -1)
#     count = 0
#     start = 0
#
#     while True:
#         count = count + 1
#         if DFS(graph, start, discovered, -1):
#             print "Graph contains cycle"
#         else:
#             print "Graph doesn't contain any cycle"
#
#         print discovered
#         if discovered.count(0) == 0:
#             break
#         else:
#             i = discovered.index(0)
#             parent = 3
#         start = i
#         if count == 2:
#             break
from functools import reduce
import operator
import math
coords = [[1.21091214887766, 5.75380525819936], [6.98157308188523, 5.65679927725593], [3.9444040363752, 5.70785470504333]]
center = tuple(map(operator.truediv, reduce(lambda x, y: map(operator.add, x, y), coords), [len(coords)] * 2))
result = (sorted(coords, key=lambda coord: (-135 - math.degrees(math.atan2(*tuple(map(operator.sub, coord, center))[::-1]))) % 360))
print result