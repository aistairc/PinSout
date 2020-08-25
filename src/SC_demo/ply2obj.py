import polygon3dmodule
import numpy as np
from collada import *

class Ply2Obj:

    def __init__(self, polygon_list):
        self.polygon_list = polygon_list
        self.output = {}
        self.local_vertices = {}
        self.local_nvertices = {}
        self.face_output = {}
        self.nface_output = {}
        self.normal_vertics = {}
        self.vertices = {}
        self.output['All'] = []
        self.local_vertices['All'] = []
        self.local_nvertices['All'] = []
        self.face_output['All'] = []
        self.nface_output['All'] = []
        self.normal_vertics['All'] = []
        self.vertices['All'] = []

    def get_index(self, point, list_vertices, shift=0):
        """Index the vertices.
        The third option is for incorporating a local index (building-level) to the global one (dataset-level)."""

        """Unique identifier and indexer of vertices."""
        if point in list_vertices:
            return list_vertices.index(point) + shift, list_vertices
        else:
            list_vertices.append(point)
            return list_vertices.index(point) + shift, list_vertices

    # def generateNormals(self):
    #     """If :attr:`normals` is `None` or you wish for normals to be
    #     recomputed, call this method to recompute them."""
    #     print self.local_vertices['All'].shape
    #     norms = np.zeros(self._vertex.shape, dtype=self._vertex.dtype)
    #
    #     tris = self._vertex[self._vertex_index]
    #     print tris
    #     n = np.cross(tris[::, 1] - tris[::, 0], tris[::, 2] - tris[::, 0])
    #     print n
    #     print self._vertex_index[:, 0]
    #     self.normalize_v3(n)
    #     norms[self._vertex_index[:, 0]] += n
    #     norms[self._vertex_index[:, 1]] += n
    #     norms[self._vertex_index[:, 2]] += n
    #     self.normalize_v3(norms)
    #     self._normal = norms
    #     self._normal_index = self._vertex_index
    def normalize_v3(self, arr):

        lens = np.sqrt(arr[:, 0] ** 2 + arr[:, 1] ** 2 + arr[:, 2] ** 2)
        lens[np.equal(lens, 0)] = 1
        arr[:, 0] /= lens
        arr[:, 1] /= lens
        arr[:, 2] /= lens

        return arr

    def poly_2_obj(self, cl):

        temp = list()

        for list_vertices in self.polygon_list:


            vaild = polygon3dmodule.isPolyValid(list_vertices, True)

            if vaild:

                t = polygon3dmodule.triangulation(list_vertices)
                for tri in t:

                    n = np.cross(np.array(tri[1]) - np.array(tri[0]), np.array(tri[2]) - np.array(tri[0]))
                    temp.append(n)
                    temp_v = list()
                    for ep in range(len(tri)):
                        v, self.local_vertices[cl] = self.get_index(tri[ep], self.local_vertices[cl],
                                                                    len(self.vertices[cl]))
                        temp_v.append(v)
                        self.face_output[cl].append(v)


        for cl in self.local_vertices:
            for vertex in self.local_vertices[cl]:
                self.vertices[cl].append(vertex[0])
                self.vertices[cl].append(vertex[1])
                self.vertices[cl].append(vertex[2])


    def output_dae(self, dae_filename):

        indices = list()

        for i in range(len(self.face_output['All'])):
            indices.append(self.face_output['All'][i])


        mesh = Collada()
        effect = material.Effect("effect0", [], "phong", diffuse=(0, 0, 0), specular=(0, 0, 0))
        mat = material.Material("material0", "mymaterial", effect)
        mesh.effects.append(effect)
        mesh.materials.append(mat)

        vert_src = source.FloatSource("test-array", np.array(self.vertices['All']), ('X', 'Y', 'Z'))
        geom = geometry.Geometry(mesh, "geometry0", "mycube", [vert_src])

        input_list = source.InputList()
        input_list.addInput(0, 'VERTEX', "#test-array")

        triset = geom.createTriangleSet(np.array(indices), input_list, "materialref")

        geom.primitives.append(triset)
        mesh.geometries.append(geom)

        matnode = scene.MaterialNode("materialref", mat, inputs=[])
        geomnode = scene.GeometryNode(geom, [matnode])
        node = scene.Node("node0", children=[geomnode])

        myscene = scene.Scene("myscene", [node])
        mesh.scenes.append(myscene)
        mesh.scene = myscene


        mesh.write(dae_filename)
#
# if __name__ == "__main__":
#     a = list()
#     b = list()
#     c = list()
#     d = list()
#     a.append([32.5970243380032, 36.9873963483887, 3.09508482])
#     a.append([32.5970243380032, 36.9873963483887, 0.0])
#     a.append([37.5840095015936, 44.4522287484350, 0.0])
#     a.append([37.5840095015936, 44.4522287484350, 3.339632034301758])
#     a.append([32.5970243380032, 36.9873963483887, 3.09508482])
#     c.append([2, 0, 0])
#     c.append([2, 1, 0])
#     c.append([2, 1, 1])
#     c.append([2, 0, 1])
#     c.append([2, 0, 0])
#     d.append([2, 1, 0])
#     d.append([0, 1, 0])
#     d.append([0, 1, 1])
#     d.append([2, 1, 1])
#     d.append([2, 1, 0])

    # c = [[29.4000257775920, 49.9888111354351, 3.53930401802063], [29.4000257775920, 49.9888111354351, 0.0], [37.5841859698525, 44.4525089839345, 0.0], [37.5841859698525, 44.4525089839345, 3.53930401802063], [29.4000257775920, 49.9888111354351, 3.53930401802063]]
    # d = [[20.0157376796853, 35.6988615849451, 0.0], [20.0255873667074, 35.8046250829720, 0.0], [28.2361357452717, 30.4180775684758, 0.0], [28.2361357452717, 30.4180775684758, 3.4791018962860107], [20.0255873667074, 35.8046250829720, 3.4791018962860107], [20.0157376796853, 35.6988615849451, 3.4791018962860107], [20.0157376796853, 35.6988615849451, 0.0]]
    # e = [[26.4448109810408, 45.5493265130182, 3.25236034], [26.4448109810408, 45.5493265130182, 0.0], [29.4002157006027, 49.9883932556753, 0.0], [29.4002157006027, 49.9883932556753, 3.3963212966918945], [26.4448109810408, 45.5493265130182, 3.25236034]]
    # f = [[20.8195304666681, 37.3263819171933, 3.503504514694214], [20.7836321120197, 36.9819773201132, 3.503504514694214], [20.7836321120197, 36.9819773201132, 0.0], [20.8195304666681, 37.3263819171933, 0.0], [20.8195304666681, 37.3263819171933, 3.503504514694214]]
    # g = [[20.7825621564999, 36.9892560189464, 0.0], [20.7765028864053, 37.2230683233320, 0.0], [20.7765028864053, 37.2230683233320, 5.308474063873291], [20.7825621564999, 36.9892560189464, 5.308474063873291], [20.9942790172178, 35.8838756730033, 5.308474063873291], [20.9942790172178, 35.8838756730033, 0.0], [20.7825621564999, 36.9892560189464, 0.0]]
    # h = [[19.8345142013313, 36.8785743064563, 4.5693888664245605], [20.0257380745464, 35.8084428741886, 4.5693888664245605], [20.0257380745464, 35.8084428741886, 0.0], [19.8345142013313, 36.8785743064563, 0.0], [19.8345142013313, 36.8785743064563, 4.5693888664245605]]
    # i = [[20.7763956573824, 37.2233417301987, 0.0], [20.8101703234998, 37.3202510967757, 0.0], [20.8101703234998, 37.3202510967757, 5.126150608062744], [20.7763956573824, 37.2233417301987, 5.126150608062744], [19.8351605879877, 36.8778537375964, 5.126150608062744], [19.8351605879877, 36.8778537375964, 0.0], [20.7763956573824, 37.2233417301987, 0.0]]
    # j = [[20.9946660619463, 35.8816852380590, 5.008011341094971], [20.0087275987706, 35.7089960405232, 5.008011341094971], [20.0087275987706, 35.7089960405232, 0.0], [20.9946660619463, 35.8816852380590, 0.0], [20.9946660619463, 35.8816852380590, 5.008011341094971]]
    # s = list()
    # s.append(a)
    # # s.append(c)
    # # s.append(d)
    # poly2obj = Ply2Obj(s)
    # poly2obj.poly_2_obj('All')
    # poly2obj.output_dae()
