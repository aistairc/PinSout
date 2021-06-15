import networkx as nx
import math
import Point_sort as rs
import sys
import logging
import time
handler = logging.StreamHandler(sys.stdout)
handler.setFormatter(logging.Formatter('%(asctime)s %(funcName)s [%(levelname)s]: %(message)s'))
logger = logging.getLogger(__name__)
logger.setLevel(logging.INFO)
logger.addHandler(handler)
class MakingGraph2:

    def __init__(self, surface_list):

        self.new_line_info = []
        self.new_line_info2 = []
        self.surface_info = surface_list
        self.atomic_lines = []
        self.atomic_lines2 = []
        self.first_edges = []
    def make_line_info(self):
        """Making the line information from each wall surface

            Align the intersections of each face and use the floor points to organize the data.

            Args:
                self.surface_info: Intersection point list on each side

            Returns:
                self.new_line_info: List of aligned floor points
                self.new_line_info2: List of aligned intersections points
        """
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

                    if len(merge_line_info) > 1:

                        sorted_new_line_info, sorted_index = self.sorted2Dpoints(merge_line_info)

                        self.new_line_info.append(sorted_new_line_info)
                        temp_merge2 = []
                        for sorted_i in sorted_index:
                            temp_merge2.append(merge_line_info2[sorted_i])
                        self.new_line_info2.append(temp_merge2)

    def sorted2Dpoints(self, point_list):
        """Function to align intersection point

            Since coordinates are created in one plane, align using the distance between points

            Args:
                point_list: Intersection point list on each side

            Returns:
                sorted_list: List of aligned points
                sorted_index: The current position list of indices moved from the existing array.
       """
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

            sorted_list.insert(len(sorted_list) - 1, point_list[index])
            sorted_index.insert(len(sorted_index) - 1, index_list[index])
            index_list.pop(index)
            point_list.pop(index)


        return sorted_list, sorted_index


    def make_edge_info(self):
        """Generate edge information to create a graph

            Using the intersection of each face, create edge information that makes up the face

            Args:
                self.new_line_info: List of aligned floor points
                self.new_line_info2: List of aligned intersections points
            Returns:
                new_edges: List of edge information
                pointcloud_i: Index information of PointCloud including each surface
        """
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

                    if len(new_list[j][index_i]) == 0:
                        new_list[j][index_i].append(i)


        new_edges = []
        pointcloud_i = []
        for i in new_list:
            for j in range(len(i) - 1):
                new_edges.append([i[j][0], i[j+1][0]])
            pointcloud_i.append(reduce(lambda x, y: x + y, i))
        self.first_edges = pointcloud_i
        return new_edges, pointcloud_i

    def make_graph(self):
        """Generate edge information to create a graph

             Using the intersection of each face, create edge information that makes up the face

             Args:
                 self.new_line_info: List of aligned floor points
                 self.new_line_info2: List of aligned intersections points
             Returns:
                 new_edges: List of edge information
                 pointcloud_i: Index information of PointCloud including each surface
         """
        self.make_line_info()
        wall_graph, pointcloud_i = self.make_edge_info()
        G = nx.Graph()
        G.add_edges_from(wall_graph)
        cycles_list = nx.minimum_cycle_basis(G)

        if len(cycles_list) >= 1:
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

            checked_list = []
            if len(delete_dup_edges) > 0:
                for delete_edge in delete_dup_edges:
                    if temp_delete.count([delete_edge[0], delete_edge[1]]) > 1:
                        delete_edges.append([delete_edge[0], delete_edge[1]])

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

            return checked_list, G

    def get_newGraph(self, G, delete_value):



        if len(delete_value) != 0:
            G.remove_edges_from(delete_value)
        new_cycles = nx.minimum_cycle_basis(G)


        a = rs.Point_sort()

        wall_list2 = []
        wall_list3 = []
        result_ceiling = []
        result_floor = []
        result_wall = []
        roomCount = 0
        print len(new_cycles), new_cycles
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


                reverseCheck = 0
                for each_j in range(len(start_node) - 1):
                    now_value_l = start_node[each_j]
                    now_value_r = start_node[each_j + 1]
                    nextV = self.atomic_lines2[now_value_r][0]
                    currV = self.atomic_lines2[now_value_l][0]
                    x1 = nextV[0] - currV[0]
                    y1 = nextV[1] + currV[1]
                    reverseCheck += (x1*y1)
                    if each_j + 1 == len(start_node) - 1:
                        fNode = start_node[0]
                        x1 = self.atomic_lines2[fNode][0][0] - nextV[0]
                        y1 = self.atomic_lines2[fNode][0][1] + nextV[1]
                        reverseCheck += (x1 * y1)
                if reverseCheck > 0:
                    start_node.reverse()

                for each_i in range(len(start_node) - 1):
                    wall = []
                    now_value_l = start_node[each_i]
                    now_value_r = start_node[each_i + 1]
                    wall.append(self.atomic_lines2[now_value_l][0])
                    wall.append(self.atomic_lines2[now_value_r][0])
                    wall.append(self.atomic_lines2[now_value_r][1])
                    wall.append(self.atomic_lines2[now_value_l][1])
                    # wall.append(self.atomic_lines2[now_value_l][0])
                    # sorted_wall = rs2.CalculateCentroid(wall)
                    wall_list.append(wall)
                    wall_list2.append(wall)
                    wall_list3.extend(wall)

                    if self.atomic_lines2[now_value_l][1] not in ceiling:
                        ceiling.append(self.atomic_lines2[now_value_l][1])
                    if self.atomic_lines2[now_value_l][0] not in floor:
                        floor.append(self.atomic_lines2[now_value_l][0])

                    if each_i + 1 == len(start_node) - 1:
                        ceiling.append(self.atomic_lines2[now_value_r][1])
                        floor.append(self.atomic_lines2[now_value_r][0])

                result_ceiling.append(ceiling)
                result_floor.append(floor)
                result_wall.append(wall_list)

                # make_gml_file2 = gml.PointCloudToCityGML([ceiling], [floor], wall_list,
                #                                          [], [])
                # make_gml_file2.MakeRoomObject()
                a.visual_graph(ceiling)



        # logger.info("Making the ROOM information from the Cluster data: "+str(len(result_wall)))
        a.visual_graph3(result_ceiling)
        return result_ceiling, result_floor, result_wall



    def get_lengthRate(self, p_index, delete_value):


        m_p1 = self.atomic_lines[p_index[0]]
        m_p2 = self.atomic_lines[p_index[-1]]

        s_p1 = self.atomic_lines[delete_value[0]]
        s_p2 = self.atomic_lines[delete_value[1]]

        m_length = math.fabs(m_p1[0] - m_p2[0]) + math.fabs(m_p1[1] - m_p2[1])
        s_length = math.fabs(s_p1[0] - s_p2[0]) + math.fabs(s_p1[1] - s_p2[1])

        rate = s_length / m_length
        return rate

