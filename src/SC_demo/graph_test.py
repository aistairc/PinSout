import networkx as nx
import matplotlib.pyplot as plt
import math
import Point_sort as rs
import Point_sort2 as rs2
#
# edges = [
#     [1, 2], [2, 3], [3,4], [4, 5], [5, 6],
#     [6, 7],
#     [7, 8], [8, 9], [9, 10],
#     [10, 11], [11, 12], [12, 1],
#     [11, 16], [16, 15],
#     [12, 13], [13, 14],
#     [9, 16], [16, 13],
#     [8, 15], [15, 14],
#     [23, 22], [22, 20], [20, 18],
#     [24, 21], [21, 19], [19, 17],
#     [2, 23], [23, 24],
#     [3, 22], [22, 21],
#     [4, 20], [20, 19],
#     [5, 18], [18, 17]
# ]
#
# gl = nx.Graph()
#
# gl.add_edges_from(edges)
# # nx.draw(gl, with_labels = True, font_weigth = "bold")
# # plt.show()
# # print nx.info(gl)
# cycles_list = nx.minimum_cycle_basis(gl)
# print len(cycles_list)
# temp_delete = []
# delete_edges = []
#
# for i in cycles_list:
#     print i
# for each_cycle in cycles_list:
#     for each_cycle_i in range(len(each_cycle) - 1):
#         for wall_node in edges:
#             if each_cycle[each_cycle_i] in wall_node:
#                 index_node = wall_node.index(each_cycle[each_cycle_i])
#                 for each_cycle_j in range(each_cycle_i + 1, len(each_cycle)):
#                     if each_cycle[each_cycle_j] in wall_node:
#                         if index_node == 0:
#                             temp_edge = [each_cycle[each_cycle_i], each_cycle[each_cycle_j]]
#                         else:
#                             temp_edge = [each_cycle[each_cycle_j], each_cycle[each_cycle_i]]
#
#                         temp_delete.append(temp_edge)
# delete_dup_edges = list(set(map(tuple, temp_delete)))
# for delete_edge in delete_dup_edges:
#     if temp_delete.count([delete_edge[0], delete_edge[1]]) > 1:
#         delete_edges.append([delete_edge[0], delete_edge[1]])
#
#     # print wall_graph
# print "delete_edges", delete_edges
#
# gl.remove_edges_from(delete_edges)
# new_cycles = nx.minimum_cycle_basis(gl)
# nx.draw(gl, with_labels = True, font_weigth = "bold")
# plt.show()


class MakingGraph2:

    def __init__(self, surface_list):

        self.new_line_info = []
        self.new_line_info2 = []
        self.surface_info = surface_list
        self.atomic_lines = []
        self.atomic_lines2 = []
        self.first_edges = []
    def make_line_info(self):

        for each_room in self.surface_info:

            for each_wall in each_room:
                merge_line_info = []
                merge_line_info2 = []
                each_line_info = []
                each_line_info2 = []
                if len(each_wall) != 0:
                    for wall_point in each_wall:

                        # if wall_point[:2] not in merge_line_info:
                        #     merge_line_info.append(wall_point[:2])
                        if wall_point[0][:2] not in merge_line_info:
                            merge_line_info.append(wall_point[0][:2])
                            merge_line_info2.append(wall_point)

                    if len(merge_line_info) != 0:

                        # center = tuple(
                        #     map(operator.truediv, reduce(lambda x, y: map(operator.add, x, y), merge_line_info), [len(merge_line_info)] * 2))
                        # sorted_new_line_info = (sorted(merge_line_info, key=lambda coord: (-135 - math.degrees(
                        #     math.atan2(*tuple(map(operator.sub, coord, center))[::-1]))) % 360))
                        sorted_new_line_info, sorted_index = self.sorted2Dpoints(merge_line_info)
                        # if len(sorted_new_line_info) == 3:
                        #     sorted_new_line_info = self.remove_index(sorted_new_line_info)
                        self.new_line_info.append(sorted_new_line_info)
                        temp_merge2 = []
                        for sorted_i in sorted_index:
                            temp_merge2.append(merge_line_info2[sorted_i])
                        self.new_line_info2.append(temp_merge2)


                        # for sorted_i in range(len(sorted_new_line_info) - 1):
                        #     temp_list = []
                        #     temp_list2 = []
                        #     n = sorted_new_line_info[sorted_i]
                        #     n_1 = sorted_new_line_info[sorted_i + 1]
                        #     t = merge_line_info2[sorted_index[sorted_i]]
                        #     t_1 = merge_line_info2[sorted_index[sorted_i + 1]]
                        #     if n != n_1:
                        #         temp_list.append(n)
                        #         temp_list.append(n_1)
                        #         temp_list2.append(t)
                        #         temp_list2.append(t_1)
                        #
                        #         each_line_info.append(temp_list)
                        #         each_line_info2.append(temp_list2)
                        #
                        # # self.new_line_info.extend(each_line_info)
                        # # self.new_line_info2.extend(each_line_info2)
                        # self.new_line_info.append(each_line_info)
                        # self.new_line_info2.append(each_line_info2)
    def sorted2Dpoints(self, point_list):
        max = -1.0
        index = []
        sorted_list = []
        sorted_index = []
        index_list = [i for i in range(len(point_list))]


        for i in range(len(point_list) - 1):
            for j in range(1, len(point_list)):
                distance = math.sqrt(
                    ((point_list[i][0] - point_list[j][0]) ** 2) + ((point_list[i][1] - point_list[j][1]) ** 2))
                if distance > max:
                    max = distance
                    index = []
                    index.append(i)
                    index.append(j)

        value0 = point_list[index[0]]
        value1 = point_list[index[1]]
        if index[0] < index[1]:
            point_list.pop(index[0])
            point_list.pop(index[1] - 1)

            index_list.pop(index[0])
            index_list.pop(index[1] - 1)
        else:
            point_list.pop(index[1])
            point_list.pop(index[0])
            index_list.pop(index[1])
            index_list.pop(index[0])

        sorted_list.append(value0)
        sorted_list.append(value1)

        sorted_index.append(index[0])
        sorted_index.append(index[1])


        # print point_list
        while True:
            if len(point_list) == 0:
                break
            min = float("inf")
            index = -1
            for i in range(len(point_list)):
                distance = math.sqrt(
                    ((sorted_list[0][0] - point_list[i][0]) ** 2) + ((sorted_list[0][1] - point_list[i][1]) ** 2))
                if distance < min:
                    min = distance
                    index = i
                # print min, index
            sorted_list.insert(len(sorted_list) - 1, point_list[index])
            sorted_index.insert(len(sorted_index) - 1, index_list[index])
            index_list.pop(index)
            point_list.pop(index)


        # print "list: ", sorted_list
        # print "index: ", sorted_index

        return sorted_list, sorted_index

    def make_edge_info(self):

        all_lines = reduce(lambda x, y: x+y,self.new_line_info)
        all_lines2 = reduce(lambda x, y: x+y,self.new_line_info2)
        # self.atomic_lines = list(set(map(tuple, all_lines)))

        # for each_line in all_lines:
        #     if each_line not in self.atomic_lines:
        #         self.atomic_lines.append(each_line)
        for each_l in range(len(all_lines)):
            if all_lines[each_l] not in self.atomic_lines:
                self.atomic_lines.append(all_lines[each_l])
                self.atomic_lines2.append(all_lines2[each_l])


        used_l = []
        wall_graph = []
        for atomic_i in range(len(self.atomic_lines)):
            atomic_v = [self.atomic_lines[atomic_i][0], self.atomic_lines[atomic_i][1]]

            for l in range(len(self.new_line_info)):
                if l not in used_l:
                    if atomic_v in self.new_line_info[l]:
                        index = self.new_line_info[l].index(atomic_v)

                        if index == 0:
                            v_i = self.atomic_lines.index(self.new_line_info[l][1])
                            # v_i = self.atomic_lines.index(tuple(self.new_line_info[l][1]))
                            wall_graph.append([atomic_i, v_i])
                        else:
                            v_i = self.atomic_lines.index(self.new_line_info[l][0])
                            # v_i = self.atomic_lines.index(tuple(self.new_line_info[l][0]))
                            wall_graph.append([v_i, atomic_i])
                        # print atomic_i,  v_i, self.new_line_info[l]
                        used_l.append(l)

        # self.r_s.visual_graph2(self.atomic_lines)
        return used_l, wall_graph

    def make_edge_info2(self):

        new_list = [[] for i in range(len(self.new_line_info))]
        all_lines = reduce(lambda x, y: x + y, self.new_line_info)
        all_lines2 = reduce(lambda x, y: x + y, self.new_line_info2)

        for each_l in range(len(all_lines)):
            if all_lines[each_l] not in self.atomic_lines:
                self.atomic_lines.append(all_lines[each_l])
                self.atomic_lines2.append(all_lines2[each_l])


        for i in range(len(self.new_line_info)):

            for j in range(len(self.new_line_info[i])):
                new_list[i].append([])

        for i in range(len(self.atomic_lines)):

            for j in range(len(self.new_line_info)):

                if self.atomic_lines[i] in self.new_line_info[j]:

                    index_i = self.new_line_info[j].index(self.atomic_lines[i])
                    print j, index_i, new_list[j][0]
                    if len(new_list[j][index_i]) == 0:
                        new_list[j][index_i].append(i)


        new_edges = []
        new_edges2 = []
        for i in new_list:
            for j in range(len(i) - 1):
                new_edges.append([i[j][0], i[j+1][0]])
            new_edges2.append(reduce(lambda x, y: x + y, i))
        self.first_edges = new_edges2
        return new_edges, new_edges2




    def make_graph2(self):
        self.make_line_info()
        wall_graph, pointcloud_i = self.make_edge_info2()
        G = nx.Graph()
        G.add_edges_from(wall_graph)

        cycles_list = nx.minimum_cycle_basis(G)
        cycles_list2 = nx.cycle_basis(G)
        for cl_2 in cycles_list2:
            print cl_2
        print "minimum_cycle_basis"
        # max_len = 0
        # max_i = -1
        # for i in range(len(cycles_list)):
        #     print cycles_list[i]
        #     if len(cycles_list[i]) > max_len:
        #         max_len = cycles_list[i]
        #         max_i = i
        # cycles_list.pop(max_i)
        if len(cycles_list) > 1:
            delete_edges = []
            temp_delete = []

            for each_cycle in cycles_list:
                for each_cycle_i in range(len(each_cycle) - 1):
                    for wall_node in wall_graph:
                        if each_cycle[each_cycle_i] in wall_node:
                            index_node = wall_node.index(each_cycle[each_cycle_i])
                            for each_cycle_j in range(each_cycle_i + 1, len(each_cycle)):
                                if each_cycle[each_cycle_j] in wall_node:
                                    if index_node == 0:
                                        temp_edge = [each_cycle[each_cycle_i], each_cycle[each_cycle_j]]
                                    else:
                                        temp_edge = [each_cycle[each_cycle_j], each_cycle[each_cycle_i]]

                                    temp_delete.append(temp_edge)
            delete_dup_edges = list(set(map(tuple, temp_delete)))

            for delete_edge in delete_dup_edges:
                if temp_delete.count([delete_edge[0], delete_edge[1]]) > 1:
                    delete_edges.append([delete_edge[0], delete_edge[1]])

            # print wall_graph
            print "delete_edges", delete_edges
            checked_list = []
            for delete_value in delete_edges:
                for p_i in range(len(pointcloud_i)):
                    check_count = 0
                    if delete_value[0] in pointcloud_i[p_i]:
                        check_count = check_count+1
                    if delete_value[1] in pointcloud_i[p_i]:
                        check_count = check_count+1
                    if check_count == 2:
                        checked_value = []
                        checked_value.append(p_i)
                        checked_value.append(self.get_lengthRate(pointcloud_i[p_i], delete_value))
                        checked_value.append(self.atomic_lines2[delete_value[0]])
                        checked_value.append(self.atomic_lines2[delete_value[1]])
                        checked_value.append(delete_value)
                        checked_list.append(checked_value)
            # for ii in checked_list:
            #     print ii
            return checked_list, G
    def get_newGraph(self, G, delete_value):

        for i_i in self.first_edges:
            print i_i
        for d_i in delete_value:

            print d_i

        G.remove_edges_from(delete_value)
        new_cycles = nx.minimum_cycle_basis(G)
        nx.draw(G)
        plt.show()
        print G.edges
        a = rs.Point_sort()
        print "make", len(new_cycles)
        wall_list2 = []
        wall_list3 = []
        ceiling_list2 = []
        for cycle_l in new_cycles:
            if len(cycle_l) > 1:

                temp_list_l = []
                for cycle_node in cycle_l:

                    for test_i in nx.edges(G, cycle_node):
                        temp_list_l.append([test_i[0], test_i[1]])

                used_index = []
                for t_i in range(len(temp_list_l) - 1):
                    for t_j in range(1 + t_i, len(temp_list_l)):
                        if temp_list_l[t_i][0] in temp_list_l[t_j]:
                            if temp_list_l[t_i][1] in temp_list_l[t_j]:
                                used_index.append(t_j)
                # print used_index
                new_list = []
                for t_k in range(len(temp_list_l)):
                    if t_k not in used_index:
                        if temp_list_l[t_k][0] in cycle_l:
                            if temp_list_l[t_k][1] in cycle_l:
                                new_list.append(temp_list_l[t_k])

                start_node = new_list[0]
                new_list.pop(0)
                # first_check = True
                # while True:

                right = start_node[1]
                left = start_node[0]
                while True:
                    used_index = []
                    for i in new_list:
                        if right in i:
                            if i.index(right) == 0:
                                right = i[1]
                            else:
                                right = i[0]
                            start_node.append(right)
                            used_index.append(new_list.index(i))
                            continue
                        if left in i:
                            if i.index(left) == 0:
                                left = i[1]
                            else:
                                left = i[0]
                            start_node.insert(0, left)
                            used_index.append(new_list.index(i))
                            continue
                    new_list2 = []
                    if len(used_index) == len(new_list):
                        break
                    for j in range(len(new_list)):
                        if j not in used_index:
                            new_list2.append(new_list[j])
                    new_list = new_list2

                result_point_list = []
                ceiling_list = []
                floor_list = []
                wall_list = []

                floor = []
                ceiling = []
                print start_node
                for each_i in range(len(start_node) - 1):
                    print each_i
                    wall = []
                    now_value = start_node[each_i]
                    next_value = start_node[each_i + 1]
                    wall.append(self.atomic_lines2[now_value][0])
                    wall.append(self.atomic_lines2[next_value][0])
                    wall.append(self.atomic_lines2[now_value][1])
                    wall.append(self.atomic_lines2[next_value][1])

                    wall_list.append(rs2.CalculateCentroid(wall))
                    wall_list2.append(rs2.CalculateCentroid(wall))
                    wall_list3.extend(rs2.CalculateCentroid(wall))

                    if self.atomic_lines2[now_value][1] not in ceiling:
                        ceiling.append(self.atomic_lines2[now_value][1])
                    if self.atomic_lines2[now_value][0] not in floor:
                        floor.append(self.atomic_lines2[now_value][0])

                    if each_i + 1 == len(start_node) - 1:
                        ceiling.append(self.atomic_lines2[next_value][1])
                        floor.append(self.atomic_lines2[next_value][0])

                ceiling_list2.append(ceiling)
                a.visual_graph(ceiling)


        a.visual_graph3(ceiling_list2)

    def get_lengthRate(self, p_index, delete_value):


        m_p1 = self.atomic_lines[p_index[0]]
        m_p2 = self.atomic_lines[p_index[-1]]

        s_p1 = self.atomic_lines[delete_value[0]]
        s_p2 = self.atomic_lines[delete_value[1]]

        m_length = math.fabs(m_p1[0] - m_p2[0]) + math.fabs(m_p1[1] - m_p2[1])
        s_length = math.fabs(s_p1[0] - s_p2[0]) + math.fabs(s_p1[1] - s_p2[1])

        rate = s_length / m_length
        return rate




# test_data = [[[[[4.69854685802589, 3.10151648035394, 0.1383921056985855], [4.69898191219853, 3.10958355974974, 2.9708728790283203]], [[-4.69465665226413, 3.10398086955731, 0.1118827760219574], [-4.70062380572536, 3.11214009719768, 2.9761099815368652]], [[2.99017285842551, 3.10193782673220, 0.12413974851369858], [2.99012450869413, 3.11003465572297, 2.967020034790039]], [[2.85540792304944, 3.10196782928267, 0.12187949568033218], [2.85528386582496, 3.11008741926494, 2.9727442264556885]], [[1.17219602331160, 3.10231435168829, 0.08374449610710144], [1.17223855326683, 3.11050433268750, 2.9593400955200195]], [[1.03741109239587, 3.10232000767662, 0.07293400168418884], [1.03732531466617, 3.11054700494348, 2.9615142345428467]]], [[[4.69853791261411, -3.11214530639999, 0.1383921056985855], [4.69897295526738, -3.11207981056381, 2.9708728790283203]], [[-4.69444402063365, -3.11231134419257, 0.1118827760219574], [-4.70041089726566, -3.11224522711288, 2.9761099815368652]], [[-0.0238351378024052, -3.11222895144253, 0.11777067184448242], [-0.0230678311454295, -3.11216263718301, 2.98539662361145]], [[-0.215554603783639, -3.11223304530248, 0.08674249798059464], [-0.215549185321421, -3.11216673332830, 2.9548499584198]]], [[[4.69854685802589, 3.10151648035394, 0.1383921056985855], [4.69898191219853, 3.10958355974974, 2.9708728790283203]], [[4.69853791261411, -3.11214530639999, 0.1383921056985855], [4.69897295526738, -3.11207981056381, 2.9708728790283203]]], [[[-4.69465665226413, 3.10398086955731, 0.1118827760219574], [-4.70062380572536, 3.11214009719768, 2.9761099815368652]], [[-4.69444402063365, -3.11231134419257, 0.1118827760219574], [-4.70041089726566, -3.11224522711288, 2.9761099815368652]], [[-4.69443219202276, -0.793317465108162, 0.0681283250451088], [-4.70062455781574, -0.793303843199870, 3.0405960083007812]], [[-4.69457174532324, -0.947812407869469, 0.13765360414981842], [-4.70060417840854, -0.947599103061783, 3.0333468914031982]]], [[[-4.69443219202276, -0.793317465108162, 0.0681283250451088], [-4.70062455781574, -0.793303843199870, 3.0405960083007812]], [[-0.0768509059854013, -0.793330388367523, 0.11777067184448242], [-0.0760823832272323, -0.793317266118729, 2.98539662361145]], [[-0.215569985664283, -0.793330135316455, 0.08674249798059464], [-0.215564566849273, -0.793317008690659, 2.9548499584198]]], [[[2.99017285842551, 3.10193782673220, 0.12413974851369858], [2.99012450869413, 3.11003465572297, 2.967020034790039]], [[2.99021039174232, -0.792066934604599, 0.0672641471028328], [2.99016019817793, -0.792809812568359, 3.02361798286438]], [[2.99020831932797, -0.658530435614269, 0.11546099931001663], [2.99015882983623, -0.658547353902615, 3.029949903488159]]], [[[2.85540792304944, 3.10196782928267, 0.12187949568033218], [2.85528386582496, 3.11008741926494, 2.9727442264556885]], [[2.85547731324113, -0.792161839406786, 0.0672641471028328], [2.85534882328056, -0.792904772521843, 3.02361798286438]], [[2.85547291846695, -0.658525668156383, 0.11546099931001663], [2.85534623573955, -0.658542583713335, 3.029949903488159]]], [[[2.99021039174232, -0.792066934604599, 0.0672641471028328], [2.99016019817793, -0.792809812568359, 3.02361798286438]], [[2.85547731324113, -0.792161839406786, 0.0672641471028328], [2.85534882328056, -0.792904772521843, 3.02361798286438]], [[1.17213605872868, -0.793351711279736, 0.08374449610710144], [1.17217845149698, -0.794074231915932, 2.9593400955200195]], [[1.03751796044006, -0.793443818739471, 0.07293400168418884], [1.03743242830425, -0.794169692131849, 2.9615142345428467]]], [[[1.17219602331160, 3.10231435168829, 0.08374449610710144], [1.17223855326683, 3.11050433268750, 2.9593400955200195]], [[1.17213605872868, -0.793351711279736, 0.08374449610710144], [1.17217845149698, -0.794074231915932, 2.9593400955200195]], [[1.17213860266967, -0.658466105299048, 0.11546099931001663], [1.17218157982450, -0.658483026859221, 3.029949903488159]]], [[[1.03741109239587, 3.10232000767662, 0.07293400168418884], [1.03732531466617, 3.11054700494348, 2.9615142345428467]], [[1.03751796044006, -0.793443818739471, 0.07293400168418884], [1.03743242830425, -0.794169692131849, 2.9615142345428467]], [[1.03751299808344, -0.658461341726177, 0.11546099931001663], [1.03742667915385, -0.658478258711356, 3.029949903488159]]], [[[2.99020831932797, -0.658530435614269, 0.11546099931001663], [2.99015882983623, -0.658547353902615, 3.029949903488159]], [[2.85547291846695, -0.658525668156383, 0.11546099931001663], [2.85534623573955, -0.658542583713335, 3.029949903488159]], [[1.17213860266967, -0.658466105299048, 0.11546099931001663], [1.17218157982450, -0.658483026859221, 3.029949903488159]], [[1.03751299808344, -0.658461341726177, 0.11546099931001663], [1.03742667915385, -0.658478258711356, 3.029949903488159]]], [[[-4.69457174532324, -0.947812407869469, 0.13765360414981842], [-4.70060417840854, -0.947599103061783, 3.0333468914031982]], [[-0.0733125882786270, -0.948095661552296, 0.11777067184448242], [-0.0725485855279601, -0.947884835124891, 2.98539662361145]], [[-0.215568959107065, -0.948089268832954, 0.08674249798059464], [-0.215563541603990, -0.947878360741024, 2.9548499584198]]], [[[-0.0238351378024052, -3.11222895144253, 0.11777067184448242], [-0.0230678311454295, -3.11216263718301, 2.98539662361145]], [[-0.0768509059854013, -0.793330388367523, 0.11777067184448242], [-0.0760823832272323, -0.793317266118729, 2.98539662361145]], [[-0.0733125882786270, -0.948095661552296, 0.11777067184448242], [-0.0725485855279601, -0.947884835124891, 2.98539662361145]]], [[[-0.215554603783639, -3.11223304530248, 0.08674249798059464], [-0.215549185321421, -3.11216673332830, 2.9548499584198]], [[-0.215569985664283, -0.793330135316455, 0.08674249798059464], [-0.215564566849273, -0.793317008690659, 2.9548499584198]], [[-0.215568959107065, -0.948089268832954, 0.08674249798059464], [-0.215563541603990, -0.947878360741024, 2.9548499584198]]]]]
# test_graph = MakingGraph2(test_data)
# test_graph.make_graph2()
# minimum_cycle_basis
# delete_edges [[7, 8], [10, 11], [3, 20], [8, 9], [22, 23], [20, 21], [1, 2], [11, 22], [20, 18], [2, 16], [11, 9], [2, 3], [16, 17], [14, 16], [22, 12], [8, 22], [3, 4], [16, 20]]
# [1, [[-0.0238351378024052, -3.11222895144253, 0.11777067184448242], [-0.0230678311454295, -3.11216263718301, 2.98539662361145]], [[-0.215554603783639, -3.11223304530248, 0.08674249798059464], [-0.215549185321421, -3.1121667333283, 2.9548499584198]], [7, 8]]
# [3, [[-4.69443219202276, -0.793317465108162, 0.0681283250451088], [-4.70062455781574, -0.79330384319987, 3.0405960083007812]], [[-4.69457174532324, -0.947812407869469, 0.13765360414981842], [-4.70060417840854, -0.947599103061783, 3.0333468914031982]], [10, 11]]
# [8, [[1.1721960233116, 3.10231435168829, 0.08374449610710144], [1.17223855326683, 3.1105043326875, 2.9593400955200195]], [[1.17213860266967, -0.658466105299048, 0.11546099931001663], [1.1721815798245, -0.658483026859221, 3.029949903488159]], [3, 20]]
# [1, [[-0.215554603783639, -3.11223304530248, 0.08674249798059464], [-0.215549185321421, -3.1121667333283, 2.9548499584198]], [[-4.69444402063365, -3.11231134419257, 0.1118827760219574], [-4.70041089726566, -3.11224522711288, 2.9761099815368652]], [8, 9]]
# [11, [[-0.215568959107065, -0.948089268832954, 0.08674249798059464], [-0.21556354160399, -0.947878360741024, 2.9548499584198]], [[-0.073312588278627, -0.948095661552296, 0.11777067184448242], [-0.0725485855279601, -0.947884835124891, 2.98539662361145]], [22, 23]]
# [10, [[1.17213860266967, -0.658466105299048, 0.11546099931001663], [1.1721815798245, -0.658483026859221, 3.029949903488159]], [[1.03751299808344, -0.658461341726177, 0.11546099931001663], [1.03742667915385, -0.658478258711356, 3.029949903488159]], [20, 21]]
# [0, [[2.99017285842551, 3.1019378267322, 0.12413974851369858], [2.99012450869413, 3.11003465572297, 2.967020034790039]], [[2.85540792304944, 3.10196782928267, 0.12187949568033218], [2.85528386582496, 3.11008741926494, 2.9727442264556885]], [1, 2]]
# [11, [[-4.69457174532324, -0.947812407869469, 0.13765360414981842], [-4.70060417840854, -0.947599103061783, 3.0333468914031982]], [[-0.215568959107065, -0.948089268832954, 0.08674249798059464], [-0.21556354160399, -0.947878360741024, 2.9548499584198]], [11, 22]]
# [8, [[1.17213860266967, -0.658466105299048, 0.11546099931001663], [1.1721815798245, -0.658483026859221, 3.029949903488159]], [[1.17213605872868, -0.793351711279736, 0.08374449610710144], [1.17217845149698, -0.794074231915932, 2.9593400955200195]], [20, 18]]
# [6, [[2.85540792304944, 3.10196782928267, 0.12187949568033218], [2.85528386582496, 3.11008741926494, 2.9727442264556885]], [[2.85547291846695, -0.658525668156383, 0.11546099931001663], [2.85534623573955, -0.658542583713335, 3.029949903488159]], [2, 16]]
# [3, [[-4.69457174532324, -0.947812407869469, 0.13765360414981842], [-4.70060417840854, -0.947599103061783, 3.0333468914031982]], [[-4.69444402063365, -3.11231134419257, 0.1118827760219574], [-4.70041089726566, -3.11224522711288, 2.9761099815368652]], [11, 9]]
# [0, [[2.85540792304944, 3.10196782928267, 0.12187949568033218], [2.85528386582496, 3.11008741926494, 2.9727442264556885]], [[1.1721960233116, 3.10231435168829, 0.08374449610710144], [1.17223855326683, 3.1105043326875, 2.9593400955200195]], [2, 3]]
# [6, [[2.85547291846695, -0.658525668156383, 0.11546099931001663], [2.85534623573955, -0.658542583713335, 3.029949903488159]], [[2.85547731324113, -0.792161839406786, 0.0672641471028328], [2.85534882328056, -0.792904772521843, 3.02361798286438]], [16, 17]]
# [10, [[2.99020831932797, -0.658530435614269, 0.11546099931001663], [2.99015882983623, -0.658547353902615, 3.029949903488159]], [[2.85547291846695, -0.658525668156383, 0.11546099931001663], [2.85534623573955, -0.658542583713335, 3.029949903488159]], [14, 16]]
# [13, [[-0.215568959107065, -0.948089268832954, 0.08674249798059464], [-0.21556354160399, -0.947878360741024, 2.9548499584198]], [[-0.215569985664283, -0.793330135316455, 0.08674249798059464], [-0.215564566849273, -0.793317008690659, 2.9548499584198]], [22, 12]]
# [13, [[-0.215554603783639, -3.11223304530248, 0.08674249798059464], [-0.215549185321421, -3.1121667333283, 2.9548499584198]], [[-0.215568959107065, -0.948089268832954, 0.08674249798059464], [-0.21556354160399, -0.947878360741024, 2.9548499584198]], [8, 22]]
# [0, [[1.1721960233116, 3.10231435168829, 0.08374449610710144], [1.17223855326683, 3.1105043326875, 2.9593400955200195]], [[1.03741109239587, 3.10232000767662, 0.07293400168418884], [1.03732531466617, 3.11054700494348, 2.9615142345428467]], [3, 4]]
# [10, [[2.85547291846695, -0.658525668156383, 0.11546099931001663], [2.85534623573955, -0.658542583713335, 3.029949903488159]], [[1.17213860266967, -0.658466105299048, 0.11546099931001663], [1.1721815798245, -0.658483026859221, 3.029949903488159]], [16, 20]]
