import polygon3dmodule
import copy
import itertools
import numpy as np
import ring_sort as rs
from collada import *

"""
1. get vertices (list)
2. triangulation (make face)
3. make normal_vector
4. v, vt, f
"""
class Ply2Obj:

    def __init__(self, polygon_list):
        self.polygon_list = polygon_list
        self.output = {}
        self.local_vertices = {}
        self.vertices_output = {}
        self.nvertices_output = {}
        self.face_output = {}
        self.normal_vertics = {}
        self.vertices = {}
        self.output['All'] = []
        self.local_vertices['All'] = []
        self.vertices_output['All'] = []
        self.nvertices_output['All'] = []
        self.face_output['All'] = []
        self.normal_vertics['All'] = []
        self.vertices['All'] = []

    def remove_reccuring(self, list_vertices):

        list_vertices_without_last = list_vertices[:-1]
        found = set()
        for item in list_vertices_without_last:
            if str(item) not in found:
                yield item
                found.add(str(item))

    def get_index(self, point, list_vertices, shift=0):
        """Index the vertices.
        The third option is for incorporating a local index (building-level) to the global one (dataset-level)."""

        """Unique identifier and indexer of vertices."""
        # print point, list_vertices
        if point in list_vertices:
            return list_vertices.index(point) + 1 + shift, list_vertices
        else:
            list_vertices.append(point)
            return list_vertices.index(point) + 1 + shift, list_vertices

    def get_index2(self, point, list_vertices, shift=0):
        """Index the vertices.
        The third option is for incorporating a local index (building-level) to the global one (dataset-level)."""

        """Unique identifier and indexer of vertices."""

        if point in list_vertices:
            return list_vertices.index(point) + 1 + shift, list_vertices
        else:
            list_vertices.append(point)
            return list_vertices.index(point) + 1 + shift, list_vertices

    def write_vertices(self, list_vertices, cla):
        """Write the vertices in the OBJ format."""

        for each in list_vertices:
            self.vertices_output[cla].append("v" + " " + str(each[0]) + " " + str(each[1]) + " " + str(each[2]) + "\n")

    def write_normal_vertices(self, list_normal_vertices, cla):
        for each in list_normal_vertices:
            self.nvertices_output[cla].append("vn" + " " + str(each[0]) + " " + str(each[1]) + " " + str(each[2]) + "\n")

    def poly_2_obj(self, cl):
        for list_vertices in self.polygon_list:
            last_ep = list_vertices[-1]
            epoints_clean = list(self.remove_reccuring(list_vertices))
            epoints_clean.append(last_ep)

            vaild = polygon3dmodule.isPolyValid(epoints_clean, True)

            if vaild:
                t = polygon3dmodule.triangulation(epoints_clean)
                for tri in t:

                    f = "f "
                    # self.normal_vertics[cl].append(polygon3dmodule.getNormal(tri))

                    for ep in range(len(tri)):

                        v, self.local_vertices[cl] = self.get_index(tri[ep], self.local_vertices[cl], len(self.vertices[cl]))
                        # vn, self.local_vertices[cl] = self.get_index(tri[ep], self.local_vertices[cl], len(self.vertices[cl]))
                        self.normal_vertics[cl].append(polygon3dmodule.getNormal(tri))
                        tri.append(tri.pop(0))
                        f += str(v) + "//" + str(len(self.normal_vertics[cl])) + " "

                    self.face_output[cl].append(f + "\n")

        for cl in self.local_vertices:
            for vertex in self.local_vertices[cl]:
                self.vertices[cl].append(vertex)

    def output_obj(self):

        for cl in self.output:
            if len(self.vertices[cl]) > 0:
                self.write_vertices(self.vertices[cl], cl)
                self.write_normal_vertices(self.normal_vertics[cl], cl)
                self.output[cl].append("\n" + ''.join(self.vertices_output[cl]))
                print self.vertices_output[cl]
                self.output[cl].append("\n" + ''.join(self.nvertices_output[cl]))
                self.output[cl].append("\n" + ''.join(self.face_output[cl]))
                if cl == 'All':
                    adj_suffix = ""
                else:
                    adj_suffix = "-" + str(cl)


                with open("/Users/dprt/PycharmProjects/MakeToObj/test" + str(adj_suffix) + ".obj", "w") as obj_file:
                    obj_file.write(''.join(self.output[cl]))


#
#
# """
# Simple script to convert ply to obj models
# """
# import os
# from argparse import ArgumentParser
#
# from plyfile import PlyData
#
#
# def parse_args():
#     parser = ArgumentParser()
#     parser.add_argument('ply_path')
#     parser.add_argument('--obj_path', default=None, required=False)
#
#     args = parser.parse_args()
#     return args.ply_path, args.obj_path
#
#
# def ply_path_to_obj_path(ply_path):
#     """
#     Replaces the .ply extension with .obj extension
#     """
#     return os.path.splitext(ply_path)[0] + '.obj'
#
#
# def convert(ply_path, obj_path=None):
#     """
#     Converts the given .ply file to an .obj file
#     """
#     obj_path = obj_path or ply_path_to_obj_path(ply_path)
#     ply = PlyData.read(ply_path)
#
#     with open(obj_path, 'w') as f:
#         f.write("# OBJ file\n")
#
#         verteces = ply['vertex']
#
#         for v in verteces:
#             p = [v['x'], v['y'], v['z']]
#             if 'red' in v and 'green' in v and 'blue' in v:
#                 c = [v['red'] / 256, v['green'] / 256, v['blue'] / 256]
#             else:
#                 c = [0, 0, 0]
#             a = p + c
#             f.write("v %.6f %.6f %.6f %.6f %.6f %.6f \n" % tuple(a))
#
#         for v in verteces:
#             if 'nx' in v and 'ny' in v and 'nz' in v:
#                 n = (v['nx'], v['ny'], v['nz'])
#                 f.write("vn %.6f %.6f %.6f\n" % n)
#
#         for v in verteces:
#             if 's' in v and 't' in v:
#                 t = (v['s'], v['t'])
#                 f.write("vt %.6f %.6f\n" % t)
#
#         if 'face' in ply:
#             for i in ply['face']['vertex_indices']:
#                 f.write("f")
#                 for j in range(i.size):
#                     # ii = [ i[j]+1 ]
#                     ii = [i[j] + 1, i[j] + 1, i[j] + 1]
#                     # f.write(" %d" % tuple(ii) )
#                     f.write(" %d/%d/%d" % tuple(ii))
#                 f.write("\n")
#
#
# def main():
#     ply_path, obj_path = parse_args()
#     obj_path = ply_path_to_obj_path(ply_path)
#     print("Converting {ply_path} to .obj...")
#     convert(ply_path, obj_path)
#     print("Conversion finished successfully. Output path: {obj_path}")
#
#
# if __name__ == '__main__':
#     main()
#
#
#







# #! /usr/bin/env python3
# '''
# VOLUME CALCULATION STL binary MODELS
# Author: Mar Canet (mar.canet@gmail.com) - september 2012
# Description: useful to calculate cost in a 3D printing ABS or PLA usage
# Modified by:
# Author: Saijin_Naib (Synper311@aol.com)
# Date: 2016-06-26 03:55:13.879187
# Description: Added input call for print material (ABS or PLA), added print of object mass, made Python3 compatible, changed tabs for spaces
# Material Mass Source: https://www.toybuilderlabs.com/blogs/news/13053117-filament-volume-and-length
# '''
#
# import struct
# import sys
# print('Choose desired print material of STL file below:')
# material = input('1 = ABS or 2 = PLA or 3 = 3k CFRP or 4 = Plexiglass : ')
#
#
# class STLUtils:
#     def resetVariables(self):
#         self.normals = []
#         self.points = []
#         self.triangles = []
#         self.bytecount = []
#         self.fb = []  # debug list
#
#     # Calculate volume for the 3D mesh using Tetrahedron volume
#     # based on: http://stackoverflow.com/questions/1406029/how-to-calculate-the-volume-of-a-3d-mesh-object-the-surface-of-which-is-made-up
#     def signedVolumeOfTriangle(self, p1, p2, p3):
#         v321 = p3[0] * p2[1] * p1[2]
#         v231 = p2[0] * p3[1] * p1[2]
#         v312 = p3[0] * p1[1] * p2[2]
#         v132 = p1[0] * p3[1] * p2[2]
#         v213 = p2[0] * p1[1] * p3[2]
#         v123 = p1[0] * p2[1] * p3[2]
#         return (1.0 / 6.0) * (-v321 + v231 + v312 - v132 - v213 + v123)
#
#     def unpack(self, sig, l):
#         s = self.f.read(l)
#         self.fb.append(s)
#         return struct.unpack(sig, s)
#
#     def read_triangle(self):
#         n = self.unpack("<3f", 12)
#         p1 = self.unpack("<3f", 12)
#         p2 = self.unpack("<3f", 12)
#         p3 = self.unpack("<3f", 12)
#         b = self.unpack("<h", 2)
#
#         self.normals.append(n)
#         l = len(self.points)
#         self.points.append(p1)
#         self.points.append(p2)
#         self.points.append(p3)
#         self.triangles.append((l, l + 1, l + 2))
#         self.bytecount.append(b[0])
#         return self.signedVolumeOfTriangle(p1, p2, p3)
#
#     def read_length(self):
#         length = struct.unpack("@i", self.f.read(4))
#         return length[0]
#
#     def read_header(self):
#         self.f.seek(self.f.tell() + 80)
#
#     def cm3_To_inch3Transform(self, v):
#         return v * 0.0610237441
#
#     def calculateMassCM3(self, totalVolume):
#         totalMass = 0
#         if material in {1, 'ABS'}:
#             totalMass = (totalVolume * 1.04)
#         elif material in {2, 'PLA'}:
#             totalMass = (totalVolume * 1.25)
#         elif material in {3, 'CFRP'}:
#             totalMass = (totalVolume * 1.79)
#         elif material in {4, 'Plexiglass'}:
#             totalMass = (totalVolume * 1.18)
#         elif material in {5, 'Alumide'}:
#             totalMass = (totalVolume * 1.36)
#         elif material in {6, 'Aluminum'}:
#             totalMass = (totalVolume * 2.68)
#         elif material in {7, 'Brass'}:
#             totalMass = (totalVolume * 8.6)
#         elif material in {8, 'Bronze'}:
#             totalMass = (totalVolume * 9.0)
#         elif material in {9, 'Copper'}:
#             totalMass = (totalVolume * 9.0)
#         elif material in {10, 'Gold_14K'}:
#             totalMass = (totalVolume * 13.6)
#         elif material in {11, 'Gold_18K'}:
#             totalMass = (totalVolume * 15.6)
#         elif material in {12, 'Polyamide_MJF'}:
#             totalMass = (totalVolume * 1.01)
#         elif material in {13, 'Polyamide_SLS'}:
#             totalMass = (totalVolume * 0.95)
#         elif material in {14, 'Rubber'}:
#             totalMass = (totalVolume * 1.2)
#         elif material in {15, 'Silver'}:
#             totalMass = (totalVolume * 10.26)
#         elif material in {16, 'Steel'}:
#             totalMass = (totalVolume * 7.86)
#         elif material in {17, 'Titanium'}:
#             totalMass = (totalVolume * 4.41)
#         elif material in {18, 'Resin'}:
#             totalMass = (totalVolume * 1.2)
#         elif material in {19, 'Resin'}:
#             totalMass = totalVolume
#         return totalMass
#
#     def calculateVolume(self, infilename, unit):
#         print(infilename)
#         self.resetVariables()
#         totalVolume = 0
#         totalMass = 0
#         try:
#             self.f = open(infilename, "rb")
#             self.read_header()
#             l = self.read_length()
#             print("total triangles:", l)
#             try:
#                 while True:
#                     totalVolume += self.read_triangle()
#             except Exception as e:
#                 print("End calculate triangles volume")
#             totalVolume = (totalVolume / 1000)
#             totalMass = self.calculateMassCM3(totalVolume)
#
#             if totalMass <= 0:
#                 print('Total mass could not be calculated')
#             else:
#                 print('Total mass:', totalMass, 'g')
#
#                 if unit == "cm":
#                     print("Total volume:", totalVolume, "cm^3")
#                 else:
#                     totalVolume = self.cm3_To_inch3Transform(totalVolume)
#                     print("Total volume:", totalVolume, "inch^3")
#         except Exception as e:
#             print(e)
#         return totalVolume
#
#
# if __name__ == '__main__':
#     if len(sys.argv) == 1:
#         print("Define model to calculate volume ej: python measure_volume.py torus.stl")
#     else:
#         mySTLUtils = STLUtils()
#         if(len(sys.argv) > 2 and sys.argv[2] == "inch"):
#             mySTLUtils.calculateVolume(sys.argv[1], "inch")
#         else:
#             mySTLUtils.calculateVolume(sys.argv[1], "cm")