import networkx as nx
import sys
import logging
import PointCloudUtils
import src.citygml.PointCloud_To_CityGML as gml
handler = logging.StreamHandler(sys.stdout)
handler.setFormatter(logging.Formatter('%(asctime)s %(funcName)s [%(levelname)s]: %(message)s'))
logger = logging.getLogger(__name__)
logger.setLevel(logging.INFO)
logger.addHandler(handler)
class MakingGraph:

    def __init__(self, surface_list):


        self.surface_info = surface_list
        self.atomic_list = [] # Store the point [x, y, min_z]
        self.atomic_list2 = [] # Store the point [[x, y, min_z], [x, y, max_z]]
        self.index_value = [] # Store the point's index value
    def make_line_info(self):
        """Making the line information from each wall surface

            Align the intersections of each face and use the floor points to organize the data.

            Args:
                self.surface_info: Intersection point list on each side

            Returns:
                self.new_line_info: List of aligned floor points
                self.new_line_info2: List of aligned intersections points
        """

        self.each_wall_index = [[] for i in range(len(self.surface_info))]

        for each_i in range(len(self.surface_info)):
            each_wall = self.surface_info[each_i]


            for each_line in each_wall:
                # Setting the start and end point (x, y) of line segment
                start_p = each_line[0][0][:2]
                end_p = each_line[1][0][:2]

                # Add the start and end point to the list if points are not included

                if start_p not in self.atomic_list:
                    self.atomic_list.append(start_p)
                    self.atomic_list2.append(each_line[0])
                    start_i = len(self.atomic_list) - 1
                else:
                    start_i = self.atomic_list.index(start_p)

                if end_p not in self.atomic_list:
                    self.atomic_list.append(end_p)
                    self.atomic_list2.append(each_line[1])
                    end_i = len(self.atomic_list) - 1
                else:
                    end_i = self.atomic_list.index(end_p)
                # start_i and end_i are unique value
                # Add the index number of start and end point from the list
                self.index_value.append([start_i, end_i])
                self.each_wall_index[each_i].append([start_i, end_i])

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
        ceiling, floor, wall = [], [], []
        self.make_line_info()
        if len(self.index_value) > 0:
            G = nx.Graph()
            # Making the graph using index_value list
            G.add_edges_from(self.index_value)
            # Finding the minimum cycle from the Graph
            cycles_list = nx.minimum_cycle_basis(G)


            if len(cycles_list) >= 1:
                delete_edges = []
                temp_delete = []

                # Finding the overlapping edges between generated cycles

                for each_cycle in cycles_list:
                    # First making original edge information the each cycles node and self.index_value
                    # eahc_cycle is not sorted
                    for each_cycle_i in range(len(each_cycle) - 1):
                        for each_edge in self.index_value:
                            if each_cycle[each_cycle_i] in each_edge:
                                index_node = each_edge.index(each_cycle[each_cycle_i])
                                for each_cycle_j in range(each_cycle_i + 1, len(each_cycle)):
                                    if each_cycle[each_cycle_j] in each_edge:
                                        if index_node == 0:
                                            temp_edge = [each_cycle[each_cycle_i], each_cycle[each_cycle_j]]
                                        else:
                                            temp_edge = [each_cycle[each_cycle_j], each_cycle[each_cycle_i]]

                                        temp_delete.append(temp_edge)

                delete_dup_edges = list(set(map(tuple, temp_delete)))

                if len(delete_dup_edges) > 0:
                    for delete_edge in delete_dup_edges:
                        # Second checking the duplicate edges
                        if temp_delete.count([delete_edge[0], delete_edge[1]]) > 1:
                            delete_edges.append([delete_edge[0], delete_edge[1]])

                # After removing the duplicate edges, finding all cycles included from the Graph
                if len(delete_edges) > 0:
                    G.remove_edges_from(delete_edges)
                    new_cycles = nx.minimum_cycle_basis(G)
                    ceiling, floor, wall = self.get_newGraph(G, new_cycles)
                else:
                    ceiling, floor, wall = self.get_newGraph(G, cycles_list)
        return ceiling, floor, wall
    def get_newGraph(self, G, cycles_list):


        wall_list2 = []
        wall_list3 = []
        result_ceiling = []
        result_floor = []
        result_wall = []
        roomCount = 0

        for cycle_l in cycles_list:

            if len(cycle_l) >= 1:
                # Not aligned clockwise or counterclockwise
                # Arrange the nodes included in the created cycle
                # Using the edge information to find the connection between each nodes
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


                wall_list = []
                floor = []
                ceiling = []

                reverseCheck = 0
                new_node_list = []
                new_index_list =[]
                preV_index = -1

                # Connect the edges included in same plane to create a single line segment
                # self.each_wall_index has index lists included in each plane
                # Merge the node as one line segment if they are included in same plane

                test_index = []
                for each_node_i in range(len(start_node) - 1):
                    currV_index = -1
                    search_index = [start_node[each_node_i],start_node[each_node_i + 1]]
                    search_index2 = [start_node[each_node_i + 1], start_node[each_node_i]]
                    for each_wall_i in range(len(self.each_wall_index)):
                        if search_index in self.each_wall_index[each_wall_i]:
                            currV_index = each_wall_i
                        if search_index2 in self.each_wall_index[each_wall_i]:
                            currV_index = each_wall_i
                        if currV_index != -1:
                            if each_node_i == 0:
                                preV_index = currV_index
                                new_node_list.append(start_node[each_node_i])
                                new_index_list.append(each_wall_i)
                            else:
                                if preV_index == currV_index:
                                    continue
                                else:
                                    new_node_list.append(start_node[each_node_i])
                                    new_index_list.append(each_wall_i)
                                    preV_index = currV_index

                            break
                    if len(test_index) == 0:
                        test_index.append(currV_index)
                    else:
                        if len(test_index) > 0 and test_index[-1] != currV_index:
                            test_index.append(currV_index)

                    if each_node_i == len(start_node) - 2:
                        new_node_list.append(start_node[each_node_i + 1])


                # Align the generated line segment information counterclockwise
                for each_j in range(len(new_node_list) - 1):
                    now_value_l = new_node_list[each_j]
                    now_value_r = new_node_list[each_j + 1]

                    currV = self.atomic_list2[now_value_l][0]
                    nextV = self.atomic_list2[now_value_r][0]
                    x1 = nextV[0] - currV[0]
                    y1 = nextV[1] + currV[1]
                    reverseCheck += (x1*y1)
                    if each_j + 1 == len(new_node_list) - 1:
                        fNode = new_node_list[0]
                        x1 = self.atomic_list2[fNode][0][0] - nextV[0]
                        y1 = self.atomic_list2[fNode][0][1] + nextV[1]
                        reverseCheck += (x1 * y1)

                if reverseCheck > 0:

                    new_node_list.reverse()
                    test_index.reverse()


                for each_i in range(len(new_node_list)-1):

                    line_segment_index_info = sum(self.each_wall_index[test_index[each_i]], [])
                    line_segment_point_info = sum(self.surface_info[test_index[each_i]], [])
                    now_value_l = line_segment_point_info[line_segment_index_info.index(new_node_list[each_i])]
                    now_value_r = line_segment_point_info[line_segment_index_info.index(new_node_list[each_i+1])]

                    # Generate wall, ceiling, and floor information by using the point information that matches the node information

                    wall = []
                    wall.append(now_value_l[0])
                    wall.append(now_value_r[0])
                    wall.append(now_value_r[1])
                    wall.append(now_value_l[1])
                    wall.append(now_value_l[0])

                    wall_list.append(wall)
                    wall_list2.append(wall)
                    wall_list3.extend(wall)
                    # ceiling.append(now_value_l[1])
                    # ceiling.append(now_value_r[1])
                    # floor.append(now_value_l[0])
                    # floor.append(now_value_r[0])
                    if now_value_l[1] not in ceiling:
                        ceiling.append(now_value_l[1])
                    if now_value_r[1] not in ceiling:
                        ceiling.append(now_value_r[1])
                    if now_value_l[0] not in floor:
                        floor.append(now_value_l[0])
                    if now_value_r[0] not in floor:
                        floor.append(now_value_r[0])

                    if each_i == len(new_node_list) - 2:
                        ceiling.append(ceiling[0])
                        floor.append(floor[0])
                        # ceiling.append(now_value_r[1])
                        # floor.append(now_value_r[0])

                result_ceiling.append(ceiling)
                result_floor.append(floor)
                result_wall.append(wall_list)
                # PointCloudUtils.visual_graph3(result_ceiling)
                # Save one created room(cycle) information in 3DCityDB
                # There is currently no process for processing door or window information
                # If door and window processes are added, result_wall, result_ceiling, result_floor information is returned and the process is executed
                make_gml_file2 = gml.PointCloudToCityGML([ceiling], [floor], wall_list, [], [])
                make_gml_file2.MakeRoomObject()
        PointCloudUtils.visual_graph3(result_ceiling)
        return result_ceiling, result_floor, result_wall




