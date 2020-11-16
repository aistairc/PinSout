import math
import numpy as np
import psycopg2
from decimal import *
import matplotlib.pyplot as plt
import sys
import logging
handler = logging.StreamHandler(sys.stdout)
handler.setFormatter(logging.Formatter('%(asctime)s %(funcName)s [%(levelname)s]: %(message)s'))
logger = logging.getLogger(__name__)
logger.setLevel(logging.INFO)
logger.addHandler(handler)
"""  db connect  """

user = 'postgres'
password = 'dprt'
host_product = 'localhost'
dbname = 'CityModelX'
port = '5432'
srid = 'SRID=4326;'
product_connection_string = "dbname={dbname} user={user} host={host} password={password} port={port}" \
    .format(dbname=dbname,
            user=user,
            host=host_product,
            password=password,
            port=port)
try:
    product = psycopg2.connect(product_connection_string)
except:
    print("I am unable to connect to the database")

product = psycopg2.connect(product_connection_string)
cursor = product.cursor()
''' relation root'''
centerPoint = []
rootSurfaceID = 0
rootObjectID = 0
minLon = 0
minLat = 0
minH = 0
maxLon = 0
maxLat = 0
maxH = 0
envelopCounter = True
parentID = 0
CityObjectRoot = 0
CityObjectRoom = 0
buildingParent = 0
bottomID = -1
upperID = -1
sidesInfo = ""
buildingMinH = 0
buildingMaxH = 0
roofChecker = False
CityObjectGmlID = ""
CityObjectBuildingPartRoot = 0
RootSurfaceRoom = 0
buildingCounter = 0
buildingMaxArray = []
buildingMinArray = []
check_first_time = True
roomCount = 0
''' END '''
class PointCloudToCityGML:

    # def __init__(self, ceiling_point, floor_point, wall_point, door_point):
    #     self.ceiling_list = ceiling_point
    #     self.wall_list = wall_point
    #     self.floor_list = floor_point
    #     self.door_list = door_point
    def __init__(self, ceiling_point, floor_point, wall_point, door_point, window_point):
        self.ceiling_list = ceiling_point
        self.floor_list = floor_point
        self.wall_list = wall_point

        if len(door_point) != 0:
            self.door_list = door_point
        else:
            self.door_list = []
        # self.door_list = [] if len(door_point) - 1 != 0 else door_point

        self.window_list = [] if len(window_point) - 1 != 0 else window_point

    def makePolygonz (self, sideinfo):
        # srid = "SRID=25833;"
        global srid
        # srid = "SRID=4326;"
        polygonSet = "POLYGONZ(())"
        side = self.deleteString(sideinfo, len(sideinfo)-1)
        polygonz = (srid+self.insertString(polygonSet, 10, side))

        return polygonz
    def makePolygonz2 (self, sideinfo):
        # srid = "SRID=25833;"

        polygonSet = ", ()"
        side = self.deleteString(sideinfo, len(sideinfo)-1)
        polygonz = self.insertString(polygonSet, 3, side)

        return polygonz
    def insertString (self, myString, position, insertStr ):
        resultString = myString[:position] + insertStr + myString[position:]

        return resultString


    def deleteString (self, myString, position):
        resultString = myString[:position]

        return resultString

    def getCityObjectID (self):

        cursor.execute("SELECT id from citydb.cityobject order by id")
        temp = cursor.fetchall()

        if len(temp) == 0:
            return 0
        else:
            cityObjectID = temp[len(temp) - 1][0]
            lastCityID = cityObjectID

            return lastCityID


    def getSurfaceGeometryID (self):

        cursor.execute("SELECT id from citydb.surface_geometry order by id")
        temp = cursor.fetchall()

        if len(temp) == 0:
            return 0
        else:
            surfaceID = temp[len(temp) - 1][0]
            lastSurfaceID = surfaceID

            return lastSurfaceID
    def getSurfaceGeometry (self, surface_id, surface):
        global srid
        # cursor.execute("SELECT geometry from citydb.surface_geometry where id = '%s'" % surface_id)
        cursor.execute("SELECT ST_AsText(geometry) from citydb.surface_geometry where id = '%s'" % surface_id)
        update_geomerty = cursor.fetchone()[0]
        polygonz = self.insertString(update_geomerty, len(update_geomerty)-1, surface)
        # print polygonz
        # srid = "SRID=4326;"
        update_geomerty2 = srid + polygonz
        return update_geomerty2

    def getBuildingID (self):

        cursor.execute("SELECT id from citydb.building order by id")
        temp = cursor.fetchall()

        # print(surFaceID)
        if len(temp) == 0:
            return 0
        else:
            buildingID = temp[len(temp) - 1][0]
            lastBuilding = buildingID

            return lastBuilding
    def getOpeningID(self):

        cursor.execute("SELECT id from citydb.opening order by id")
        temp = cursor.fetchall()

        # print(surFaceID)
        if len(temp) == 0:
            return 0
        else:
            openingID = temp[len(temp) - 1][0]
            lastOpeningID = openingID

            return lastOpeningID
    def getThematicSurfaceID (self):
        cursor.execute("SELECT id from citydb.thematic_surface order by id")
        temp = cursor.fetchall()

        # print(surFaceID)
        if len(temp) == 0:
            return 0
        else:
            thematicID = temp[len(temp) - 1][0]
            lastTheamticID = thematicID

            return lastTheamticID

    def getSurfaceGmlID (self):
        global rootSurfaceID
        cursor.execute("SELECT gmlid from citydb.surface_geometry where id = '%s'" % rootSurfaceID)
        surfaceGmlid = cursor.fetchone()[0]
        return surfaceGmlid

    def getRoomID (self):
        cursor.execute("SELECT id from room order by id")
        temp = cursor.fetchall()

        # print(surFaceID)
        if len(temp) == 0:
            return 0
        else:
            roomID = temp[len(temp) - 1][0]
            lastRoom = roomID

            return lastRoom

    def makeOpening(self, id, objectClassID, rootSurfaceID):
        cursor.execute(
            "INSERT INTO citydb.opening (id, objectclass_id, lod4_multi_surface_id) values (%s, %s, %s)",
            (id, objectClassID, rootSurfaceID))
        product.commit()

    def makeOpeningToThemSurface (self, openingID, thematicID):
        cursor.execute(
            "INSERT INTO citydb.opening_to_them_surface (opening_id, thematic_surface_id) values (%s, %s)",
            (openingID, thematicID))
        product.commit()

    def makeThematicSurfaceRoom (self, id, objectClassID, CityObjectRoom, rootSurfaceID):

        cursor.execute("INSERT INTO citydb.thematic_surface (id, objectclass_id, room_id, lod4_multi_surface_id) values (%s, %s, %s, %s)",
                       (id, objectClassID, CityObjectRoom, rootSurfaceID))
        product.commit()

    def makeThematicSurface (self, id, objectClassID, CityObjectRoot, rootSurfaceID):

        cursor.execute("INSERT INTO citydb.thematic_surface (id, objectclass_id, building_id, lod2_multi_surface_id) values (%s, %s, %s, %s)",
                       (id, objectClassID, CityObjectRoot, rootSurfaceID))

    def makeCityObject (self, objectClassID):
        global CityObjectRoot
        global CityObjectRoom
        global rootSurfaceID
        global CityObjBuilding
        global CityObjectBuildingPartRoot
        global buildingCounter
        global RootSurfaceRoom

        if objectClassID is 26:
            # CityObjBuilding = "building" + str(getCityObjectID()+1)
            buildingCounter += 1
            print("**************")
            print(buildingCounter)
            print("**************")
            CityObjBuilding = "building" + str(buildingCounter)
            cursor.execute("INSERT INTO citydb.cityobject(id, objectclass_id, gmlid) values (%s, %s, %s)",
                           (int(self.getCityObjectID() + 1), objectClassID, CityObjBuilding))
            product.commit()
            CityObjectRoot = self.getCityObjectID()
            CityObjectBuildingPartRoot = CityObjectRoot
            cursor.execute(
                "INSERT INTO citydb.building (id, objectclass_id, building_root_id) values (%s, %s, %s)",
                (int(self.getCityObjectID()), objectClassID, int(CityObjectRoot)))
            product.commit()
        elif objectClassID is 25:
            CityObjBuildingPart = CityObjBuilding+"_building:part" + str(self.getCityObjectID()+1)
            cursor.execute("INSERT INTO citydb.cityobject(id, objectclass_id, gmlid) values (%s, %s, %s)",
                           (int(self.getCityObjectID() + 1), objectClassID, CityObjBuildingPart))
            product.commit()
            cursor.execute(
                "INSERT INTO citydb.building (id, objectclass_id, building_parent_id, building_root_id) values (%s, %s, %s, %s)",
                (int(self.getCityObjectID()), objectClassID, int(CityObjectBuildingPartRoot), int(CityObjectBuildingPartRoot)))
            product.commit()
            CityObjectRoot = self.getCityObjectID()

        elif objectClassID is 34:
            ''' Wall '''
            CityObjWallSurface = CityObjBuilding + "_building:WallSurface" + str(self.getCityObjectID() + 1)
            cursor.execute("INSERT INTO citydb.cityobject(id, objectclass_id, gmlid) values (%s, %s, %s)",
                           (int(self.getCityObjectID() + 1), objectClassID, CityObjWallSurface))
            product.commit()
            WallSurfaceGmlid = CityObjWallSurface + "_root" + str(self.getSurfaceGeometryID() + 1)
            cursor.execute(
                "INSERT INTO citydb.surface_geometry (id, gmlid, root_id, cityobject_id) values (%s, %s, %s, %s)",
                (int(self.getSurfaceGeometryID() + 1), WallSurfaceGmlid, int(self.getSurfaceGeometryID() + 1), int(self.getCityObjectID())))
            product.commit()
            rootSurfaceID = self.getSurfaceGeometryID()
            self.makeThematicSurface(self.getCityObjectID(), objectClassID, CityObjectRoot, rootSurfaceID)
        elif objectClassID is 33:
            ''' Roof '''
            CityObjRoofSurface = CityObjBuilding + "_building:RoofSurface" + str(self.getCityObjectID() + 1)
            cursor.execute("INSERT INTO citydb.cityobject(id, objectclass_id, gmlid) values (%s, %s, %s)",
                           (int(self.getCityObjectID() + 1), objectClassID, CityObjRoofSurface))
            product.commit()
            RoofSurfaceGmlid = CityObjRoofSurface + "_root" + str(self.getSurfaceGeometryID() + 1)
            cursor.execute(
                "INSERT INTO citydb.surface_geometry (id, gmlid, root_id, cityobject_id) values (%s, %s, %s, %s)",
                (
                    int(self.getSurfaceGeometryID() + 1), RoofSurfaceGmlid, int(self.getSurfaceGeometryID() + 1),
                    int(self.getCityObjectID())))
            product.commit()
            rootSurfaceID = self.getSurfaceGeometryID()
            self.makeThematicSurface(self.getCityObjectID(), objectClassID, CityObjectRoot, rootSurfaceID)

        elif objectClassID is 32:
            ''' Foot '''
            CityObjFootSurface = CityObjBuilding + "_building:FootSurface" + str(self.getCityObjectID() + 1)
            cursor.execute("INSERT INTO citydb.cityobject(id, objectclass_id, gmlid) values (%s, %s, %s)",
                           (int(self.getCityObjectID() + 1), objectClassID, CityObjFootSurface))
            product.commit()
            FootSurfaceGmlid = CityObjFootSurface + "_root" + str(self.getSurfaceGeometryID() + 1)
            cursor.execute(
                "INSERT INTO citydb.surface_geometry (id, gmlid, root_id, cityobject_id) values (%s, %s, %s, %s)",
                (
                    int(self.getSurfaceGeometryID() + 1), FootSurfaceGmlid, int(self.getSurfaceGeometryID() + 1),
                    int(self.getCityObjectID())))
            product.commit()
            rootSurfaceID = self.getSurfaceGeometryID()
            self.makeThematicSurface(self.getCityObjectID(), objectClassID, CityObjectRoot, rootSurfaceID)

        elif objectClassID is 39:
            ''' Foot '''
            CityObjDoorSurface = CityObjBuilding + "_building:DoorSurface" + str(self.getCityObjectID() + 1)
            cursor.execute("INSERT INTO citydb.cityobject(id, objectclass_id, gmlid) values (%s, %s, %s)",
                           (int(self.getCityObjectID() + 1), objectClassID, CityObjDoorSurface))
            product.commit()
            DoorSurfaceGmlid = CityObjDoorSurface + "_root" + str(self.getSurfaceGeometryID() + 1)
            cursor.execute(
                "INSERT INTO citydb.surface_geometry (id, gmlid, root_id, cityobject_id) values (%s, %s, %s, %s)",
                (
                    int(self.getSurfaceGeometryID() + 1), DoorSurfaceGmlid, int(self.getSurfaceGeometryID() + 1),
                    int(self.getCityObjectID())))
            product.commit()
            rootSurfaceID = self.getSurfaceGeometryID()
            self.makeThematicSurface(self.getCityObjectID(), objectClassID, CityObjectRoot, rootSurfaceID)

            # CityObjectRoot = getCityObjectID()
    def makeRoomObject(self, objectClassID):
        global CityObjectRoom
        global rootSurfaceID
        global RootSurfaceRoom
        global roomCount
        # if objectClassID is 41:
        #     CityObjRoom = CityObjBuilding+"_room"+str(getCityObjectID() + 1)
        #     cursor.execute("INSERT INTO citydb.cityobject(id, objectclass_id, gmlid) values (%s, %s, %s)",
        #                    (int(getCityObjectID() + 1), objectClassID, CityObjRoom))
        #     product.commit()
        #     CityObjectRoom = getCityObjectID()
        #
        #     makeRoom()
        if objectClassID is 41:
            # CityObjRoom = CityObjBuilding+"_room"+str(self.getCityObjectID() + 1)
            CityObjRoom = CityObjBuilding + "_room" + str(roomCount)
            cursor.execute("INSERT INTO citydb.cityobject(id, objectclass_id, gmlid) values (%s, %s, %s)",
                           (int(self.getCityObjectID() + 1), objectClassID, CityObjRoom))
            product.commit()
            CityObjectRoom = self.getCityObjectID()
            cursor.execute(
                "INSERT INTO citydb.surface_geometry (id, gmlid, root_id, cityobject_id) values (%s, %s, %s, %s)",
                (
                    int(self.getSurfaceGeometryID() + 1), CityObjRoom, int(self.getSurfaceGeometryID() + 1),
                    int(self.getCityObjectID())))
            product.commit()
            RootSurfaceRoom = self.getSurfaceGeometryID()
            self.makeRoom()
        elif objectClassID is 31:

            ''' Wall '''
            CityObjInteriorWallSurface = CityObjBuilding+"_building:InteriorWallSurface"+str(self.getCityObjectID()+1)
            cursor.execute("INSERT INTO citydb.cityobject(id, objectclass_id, gmlid) values (%s, %s, %s)",
                           (int(self.getCityObjectID() + 1), objectClassID, CityObjInteriorWallSurface))
            product.commit()
            InteriorWallSurfaceGmlid = CityObjInteriorWallSurface + "_root"+str(self.getSurfaceGeometryID()+1)
            cursor.execute(
                "INSERT INTO citydb.surface_geometry (id, gmlid, root_id, cityobject_id) values (%s, %s, %s, %s)",
                (int(self.getSurfaceGeometryID()+1), InteriorWallSurfaceGmlid, int(self.getSurfaceGeometryID() + 1), int(self.getCityObjectID())))
            product.commit()
            rootSurfaceID = self.getSurfaceGeometryID()
            self.makeThematicSurfaceRoom(self.getCityObjectID(), objectClassID, CityObjectRoom, rootSurfaceID)

            # print(31)
        elif objectClassID is 30:
            ''' Roof '''
            CityObjCeilingSurface = CityObjBuilding+"_building:CeilingSurface"+str(self.getCityObjectID()+1)
            cursor.execute("INSERT INTO citydb.cityobject(id, objectclass_id, gmlid) values (%s, %s, %s)",
                           (int(self.getCityObjectID() + 1), objectClassID, CityObjCeilingSurface))
            product.commit()
            CeilingSurfaceGmlid = CityObjCeilingSurface + "_root"+str(self.getSurfaceGeometryID() + 1)
            cursor.execute(
                "INSERT INTO citydb.surface_geometry (id, gmlid, root_id, cityobject_id) values (%s, %s, %s, %s)",
                (
                int(self.getSurfaceGeometryID() + 1), CeilingSurfaceGmlid, int(self.getSurfaceGeometryID() + 1), int(self.getCityObjectID())))
            product.commit()
            rootSurfaceID = self.getSurfaceGeometryID()
            self.makeThematicSurfaceRoom(self.getCityObjectID(), objectClassID, CityObjectRoom, rootSurfaceID)
            # print(30)
        elif objectClassID is 32:
            ''' Foot '''
            CityObjFloorSurface = CityObjBuilding + "_building:FloorSurface" + str(self.getCityObjectID() + 1)
            cursor.execute("INSERT INTO citydb.cityobject(id, objectclass_id, gmlid) values (%s, %s, %s)",
                           (int(self.getCityObjectID() + 1), objectClassID, CityObjFloorSurface))
            product.commit()
            FloorSurfaceGmlid = CityObjFloorSurface + "_root" + str(self.getSurfaceGeometryID() + 1)
            cursor.execute(
                "INSERT INTO citydb.surface_geometry (id, gmlid, root_id, cityobject_id) values (%s, %s, %s, %s)",
                (
                    int(self.getSurfaceGeometryID() + 1), FloorSurfaceGmlid, int(self.getSurfaceGeometryID() + 1),
                    int(self.getCityObjectID())))
            product.commit()
            rootSurfaceID = self.getSurfaceGeometryID()
            self.makeThematicSurfaceRoom(self.getCityObjectID(), objectClassID, CityObjectRoom, rootSurfaceID)
            # print(32)

        elif objectClassID is 39:
            ''' Door '''
            CityObjIndoorSurface = CityObjBuilding + "_building:DoorSurface" + str(self.getCityObjectID() + 1)
            cursor.execute("INSERT INTO citydb.cityobject(id, objectclass_id, gmlid) values (%s, %s, %s)",
                           (int(self.getCityObjectID() + 1), objectClassID, CityObjIndoorSurface))
            product.commit()
            DoorSurfaceGmlid = CityObjIndoorSurface + "_root" + str(self.getSurfaceGeometryID() + 1)
            cursor.execute(
                "INSERT INTO citydb.surface_geometry (id, gmlid, root_id, cityobject_id) values (%s, %s, %s, %s)",
                (
                    int(self.getSurfaceGeometryID() + 1), DoorSurfaceGmlid, int(self.getSurfaceGeometryID() + 1),
                    int(self.getCityObjectID())))
            product.commit()
            rootSurfaceID = self.getSurfaceGeometryID()
            self.makeOpening(self.getCityObjectID(), objectClassID, rootSurfaceID)
            self.makeOpeningToThemSurface(self.getOpeningID(), self.getThematicSurfaceID())

        elif objectClassID is 38:
            ''' Window '''
            CityObjIndoorSurface = CityObjBuilding + "_building:WindowSurface" + str(self.getCityObjectID() + 1)
            cursor.execute("INSERT INTO citydb.cityobject(id, objectclass_id, gmlid) values (%s, %s, %s)",
                           (int(self.getCityObjectID() + 1), objectClassID, CityObjIndoorSurface))
            product.commit()
            WindowSurfaceGmlid = CityObjIndoorSurface + "_root" + str(self.getSurfaceGeometryID() + 1)
            cursor.execute(
                "INSERT INTO citydb.surface_geometry (id, gmlid, root_id, cityobject_id) values (%s, %s, %s, %s)",
                (
                    int(self.getSurfaceGeometryID() + 1), WindowSurfaceGmlid, int(self.getSurfaceGeometryID() + 1),
                    int(self.getCityObjectID())))
            product.commit()
            rootSurfaceID = self.getSurfaceGeometryID()
            self.makeOpening(self.getCityObjectID(), objectClassID, rootSurfaceID)
            self.makeOpeningToThemSurface(self.getOpeningID(), self.getThematicSurfaceID())

    def makeRoom (self):
        global CityObjectRoot
        global CityObjectRoom
        global rootSurfaceID

        global RootSurfaceRoom

        cursor.execute(
            "INSERT INTO citydb.room(id, objectclass_id, building_id, lod4_multi_surface_id) values (%s, %s, %s, %s)",
            (CityObjectRoom, 41, CityObjectRoot, RootSurfaceRoom)
        )
        product.commit()

    def getEnvelopEachSurface (self, array, components):
        global envelopCounter
        defaultStart = 0
        defaultLength = len(array)
        minArray = []
        maxArray = []
        if envelopCounter is True:
            for i in range(components):
                buildingMaxArray.append(float("inf"))
                buildingMinArray.append(-float("inf"))
            envelopCounter = False
        for i in range(components):
            minArray.append(float("inf"))
            maxArray.append(-float("inf"))
        count = int(defaultLength / components)
        for i in range(count):
            for j in range(components):
                index = defaultStart + (i * components) + j
                value = float(array[index])
                minArray[j] = min(minArray[j], value)
                maxArray[j] = max(maxArray[j], value)
                buildingMaxArray[j] = min(buildingMaxArray[j], value)
                buildingMinArray[j] = max(buildingMinArray[j], value)

        return [minArray, maxArray]


    def updateBuildingEnvelop(self):
        global CityObjectRoot
        global buildingMaxArray
        global buildingMinArray
        surfaceMinLat = buildingMinArray[0]
        surfaceMinLon = buildingMinArray[1]
        surfaceMinH = buildingMinArray[2]
        surfaceMaxLat = buildingMaxArray[0]
        surfaceMaxLon = buildingMaxArray[1]
        surfaceMaxH = buildingMaxArray[2]
        polygonEnvelop = "{west} {north} {minHeight}," \
                         "{east} {south} {minHeight}," \
                         "{east} {south} {maxHeight}," \
                         "{west} {north} {maxHeight}," \
                         "{west} {north} {minHeight}," \
            .format(north=surfaceMaxLat, west=surfaceMinLon, south=surfaceMinLat, east=surfaceMaxLon, minHeight=surfaceMinH,
                    maxHeight=surfaceMaxH)
        envelop = self.makePolygonz(polygonEnvelop)
        cursor.execute(
            "UPDATE citydb.cityobject SET envelope = %s WHERE id = %s", (envelop, CityObjectRoot)
        )
        product.commit()
    def updateEnvelop(self, surfaceData, objectID):

        surfaceData = surfaceData.replace(",", " ")
        surfaceData = self.deleteString(surfaceData, len(surfaceData) - 1)
        surfaceData = surfaceData.split(" ")

        tempEnvelop = self.getEnvelopEachSurface(surfaceData, 3)

        surfaceMinLat = tempEnvelop[0][0]
        surfaceMinLon = tempEnvelop[0][1]
        surfaceMinH = tempEnvelop[0][2]
        surfaceMaxLat = tempEnvelop[1][0]
        surfaceMaxLon = tempEnvelop[1][1]
        surfaceMaxH = tempEnvelop[1][2]
        polygonEnvelop = "{west} {north} {minHeight}," \
                         "{east} {south} {minHeight}," \
                         "{east} {south} {maxHeight}," \
                         "{west} {north} {maxHeight}," \
                         "{west} {north} {minHeight}," \
            .format(north=surfaceMaxLat, west=surfaceMinLon, south= surfaceMinLat, east=surfaceMaxLon, minHeight=surfaceMinH, maxHeight=surfaceMaxH)
        envelop = self.makePolygonz(polygonEnvelop)
        cursor.execute(
            "UPDATE citydb.cityobject SET envelope = %s WHERE id = %s", (envelop, objectID)
        )
        product.commit()
    def makeNodeList(self, array):
        defaultStart = 0
        defaultLength = len(array)
        nodeList = []

        count = int(defaultLength / 3)

        for i in range(count):
            value = []
            for j in range(3):
                index = defaultStart + (i * 3) + j
                value.append(float(array[index]))

            nodeList.append(value)

        return nodeList

    def polygonArea(self, nodeList):
        area = 0
        i = 0
        for node in nodeList:
            j = ((i + 1) % len(nodeList))
            # area += (round(Decimal(nodeList[i][0]), 6) * round(Decimal(nodeList[j][1]), 6))
            # area -= (round(Decimal(nodeList[j][0]), 6) * round(Decimal(nodeList[i][1]), 6))
            area += (float(nodeList[i][0]) * float(nodeList[j][1]))
            area -= (float(nodeList[j][0]) * float(nodeList[i][1]))
            i += 1
        return area / 2



    '''new pcl version'''
    '''
    1. 26 - building object
    2. 41 - room
    3. 31 - interior wallsurface
    4. 32 - floor
    5. 30 - ceilling
    '''
    def makeSurfaceGeometry(self):

        self.make_room_ceiling()
        self.make_room_floor()
        self.make_room_wall()

    def make_room_ceiling(self):
        global rootSurfaceID
        global RootSurfaceRoom

        if len(self.ceiling_list) != 0:
            for ceiling_index in self.ceiling_list:

                CeilingSurfaceInfo = ""
                for each_point in ceiling_index:
                    CeilingSurfaceInfo += str(each_point[0]) + " " + str(each_point[1]) + " " + str(each_point[2]) + ","

                self.makeRoomObject(30)
                CeilingSurfaceGmlid = self.getSurfaceGmlID() + "_CeilingSurface_" + str(self.getSurfaceGeometryID() + 1)
                surface = self.makePolygonz(CeilingSurfaceInfo)
                # print surface
                cursor.execute(
                    "INSERT INTO citydb.surface_geometry (id, gmlid, parent_id, root_id, geometry, cityobject_id) values (%s, %s, %s, %s, %s, %s)",
                    (int(self.getSurfaceGeometryID() + 1), CeilingSurfaceGmlid, rootSurfaceID, rootSurfaceID, surface,
                     int(self.getCityObjectID())))
                product.commit()
                self.updateEnvelop(CeilingSurfaceInfo, int(self.getCityObjectID()))
                cursor.execute(
                    "INSERT INTO citydb.surface_geometry (id, parent_id, root_id, is_reverse, geometry, cityobject_id) values (%s, %s, %s, %s, %s, %s)",
                    (int(self.getSurfaceGeometryID() + 1), RootSurfaceRoom, RootSurfaceRoom, 1, surface,
                     int(self.getRoomID())))
                product.commit()


    def make_room_floor(self):
        global rootSurfaceID
        global RootSurfaceRoom

        if len(self.floor_list) != 0:
            for floor_index in self.floor_list:
                FloorSurfaceInfo = ""
                for each_point in floor_index:
                    FloorSurfaceInfo += str(each_point[0]) + " " + str(each_point[1]) + " " + str(each_point[2]) + ","
                self.makeRoomObject(32)
                FloorSurfaceGmlid = self.getSurfaceGmlID() + "_FloorSurface_" + str(self.getSurfaceGeometryID() + 1)
                surface = self.makePolygonz(FloorSurfaceInfo)
                cursor.execute(
                    "INSERT INTO citydb.surface_geometry (id, gmlid, parent_id, root_id, geometry, cityobject_id) values (%s, %s, %s, %s, %s, %s)",
                    (int(self.getSurfaceGeometryID() + 1), FloorSurfaceGmlid, rootSurfaceID, rootSurfaceID, surface,
                     int(self.getCityObjectID())))
                product.commit()
                self.updateEnvelop(FloorSurfaceInfo, int(self.getCityObjectID()))
                cursor.execute(
                    "INSERT INTO citydb.surface_geometry (id, parent_id, root_id, is_reverse, geometry, cityobject_id) values (%s, %s, %s, %s, %s, %s)",
                    (int(self.getSurfaceGeometryID() + 1), RootSurfaceRoom, RootSurfaceRoom, 1, surface,
                     int(self.getRoomID())))
                product.commit()

    def make_room_wall(self):
        global rootSurfaceID
        global RootSurfaceRoom

        if len(self.wall_list) != 0:
            count = 0
            for wall_index in self.wall_list:

                self.makeRoomObject(31)
                InteriorWallSurfaceGmlid = self.getSurfaceGmlID() + "_InteriorWallSurface_" + str(self.getSurfaceGeometryID() + 1)
                cursor.execute(
                    "INSERT INTO citydb.surface_geometry (id, gmlid, parent_id, root_id, is_composite, cityobject_id) values (%s, %s, %s, %s, %s, %s)",
                    (int(self.getSurfaceGeometryID() + 1), InteriorWallSurfaceGmlid, rootSurfaceID, rootSurfaceID, 1, int(self.getCityObjectID())))
                product.commit()
                parent_id = int(self.getSurfaceGeometryID())
                InteriorWallSurfaceGmlid = self.getSurfaceGmlID() + "_InteriorWallSurface_" + str(self.getSurfaceGeometryID() + 1)
                InteriorWallSurfaceInfo = ""
                for each_point in wall_index:

                    InteriorWallSurfaceInfo += str(each_point[0]) + " " + str(each_point[1]) + " " + str(each_point[2]) + ","
                surface = self.makePolygonz(InteriorWallSurfaceInfo)
                cursor.execute(
                    "INSERT INTO citydb.surface_geometry (id, gmlid, parent_id, root_id, geometry, cityobject_id) values (%s, %s, %s, %s, %s, %s)",
                    (int(self.getSurfaceGeometryID() + 1), InteriorWallSurfaceGmlid, parent_id, rootSurfaceID, surface, int(self.getCityObjectID())))
                product.commit()
                wall_data_id = int(self.getSurfaceGeometryID())
                cursor.execute(
                    "INSERT INTO citydb.surface_geometry (id, parent_id, root_id, is_reverse, geometry, cityobject_id) values (%s, %s, %s, %s, %s, %s)",
                    (int(self.getSurfaceGeometryID() + 1), RootSurfaceRoom, RootSurfaceRoom, 1, surface,
                     int(self.getRoomID())))
                product.commit()
                self.updateEnvelop(InteriorWallSurfaceInfo, int(self.getCityObjectID()))
                for index in self.door_list:

                    i = index[len(index) - 1]
                    if i == count:

                        door_temp_list = index[:len(index) - 1]
                        door_temp_list_2 = index[:len(index) - 1]
                        door_temp_list_2.reverse()
                        # print door_temp_list
                        # print door_temp_list_2
                        # self.visual_graph(door_temp_list)
                        # self.visual_graph(door_temp_list_2)
                        InteriorWallSurfaceInfo = ""
                        for index in door_temp_list_2:
                            InteriorWallSurfaceInfo += str(index[0]) + " " + str(index[1]) + " " + str(index[2]) + ","
                        surface = self.makePolygonz2(InteriorWallSurfaceInfo)

                        update_geometry = self.getSurfaceGeometry(wall_data_id, surface)

                        cursor.execute(
                            "UPDATE citydb.surface_geometry SET geometry = %s WHERE id = %s", (update_geometry, wall_data_id)
                        )
                        product.commit()

                        DoorSurfaceInfo = ""
                        for index in door_temp_list:
                            DoorSurfaceInfo += str(index[0]) + " " + str(index[1]) + " " + str(index[2]) + ","
                        surface = self.makePolygonz(DoorSurfaceInfo)
                        self.makeRoomObject(39)
                        # print surface
                        DoorSurfaceInfoGmlid = self.getSurfaceGmlID() + "_DoorSurface_"
                        cursor.execute(
                            "INSERT INTO citydb.surface_geometry (id, gmlid, parent_id, root_id, geometry, cityobject_id) values (%s, %s, %s, %s, %s, %s)",
                            (int(self.getSurfaceGeometryID() + 1), DoorSurfaceInfoGmlid, rootSurfaceID, rootSurfaceID, surface, int(self.getCityObjectID())))
                        product.commit()
                        self.updateEnvelop(DoorSurfaceInfo, int(self.getCityObjectID()))
                for index in self.window_list:
                    i = index[len(index) - 1]
                    if i == count:
                        window_temp_list = index[:len(index) - 1]
                        window_temp_list_2 = index[:len(index) - 1]
                        window_temp_list_2.reverse()
                        # print window_temp_list
                        # print window_temp_list_2

                        InteriorWallSurfaceInfo = ""
                        for index in window_temp_list_2:
                            InteriorWallSurfaceInfo += str(index[0]) + " " + str(index[1]) + " " + str(index[2]) + ","
                        surface = self.makePolygonz2(InteriorWallSurfaceInfo)

                        update_geometry = self.getSurfaceGeometry(wall_data_id, surface)

                        cursor.execute(
                            "UPDATE citydb.surface_geometry SET geometry = %s WHERE id = %s",
                            (update_geometry, wall_data_id)
                        )
                        product.commit()


                        WindowSurfaceInfo = ""
                        for index in window_temp_list:
                            WindowSurfaceInfo += str(index[0]) + " " + str(index[1]) + " " + str(index[2]) + ","
                        surface = self.makePolygonz(WindowSurfaceInfo)
                        self.makeRoomObject(38)
                        # print surface
                        WindowSurfaceInfoGmlid = self.getSurfaceGmlID() + "_WindowSurface_"
                        cursor.execute(
                            "INSERT INTO citydb.surface_geometry (id, gmlid, parent_id, root_id, geometry, cityobject_id) values (%s, %s, %s, %s, %s, %s)",
                            (int(self.getSurfaceGeometryID() + 1), WindowSurfaceInfoGmlid, rootSurfaceID,
                             rootSurfaceID, surface, int(self.getCityObjectID())))
                        product.commit()
                        self.updateEnvelop(WindowSurfaceInfo, int(self.getCityObjectID()))
                        # self.updateEnvelop(DoorSurfaceInfo, int(self.getCityObjectID()))


                count += 1
    # def make_room_window(self):


    def makeBuildingSurfaceGeometry(self):
        global rootSurfaceID
        global roofChecker

        RoofSurfaceInfo = ""
        FootSurfaceInfo = ""
        idx = 0
        # print(nodeList)
        if len(self.ceiling_list) == len(self.floor_list):
            size_check = True
        if size_check:
            for i in range(len(self.ceiling_list) - 1):
                nextidx = i + 1

                # point_one = (str(self.ceiling_list[idx][0]) + " " + str(self.ceiling_list[idx][1]) + " " + str(self.ceiling_list[idx][2]) + ",")
                # point_two = (str(self.ceiling_list[nextidx][0]) + " " + str(self.ceiling_list[nextidx][1]) + " " + str(self.ceiling_list[nextidx][2]) + ",")
                # point_three = (str(self.floor_list[nextidx][0]) + " " + str(self.floor_list[nextidx][1]) + " " + str(self.floor_list[nextidx][2]) + ",")
                # point_four = (str(self.floor_list[idx][0]) + " " + str(self.floor_list[idx][1]) + " " + str(self.floor_list[idx][2]) + ",")
                # point_zero = (str(self.ceiling_list[idx][0]) + " " + str(self.ceiling_list[idx][1]) + " " + str(self.ceiling_list[idx][2]) + ",")
                point_one = (str(self.ceiling_list[idx][0]) + " " + str(self.ceiling_list[idx][1]) + " " + str(self.ceiling_list[idx][2]) + ",")
                point_two = (str(self.floor_list[idx][0]) + " " + str(self.floor_list[idx][1]) + " " + str(self.floor_list[idx][2]) + ",")
                point_three = (str(self.floor_list[nextidx][0]) + " " + str(self.floor_list[nextidx][1]) + " " + str(self.floor_list[nextidx][2]) + ",")
                point_four = (str(self.ceiling_list[nextidx][0]) + " " + str(self.ceiling_list[nextidx][1]) + " " + str(self.ceiling_list[nextidx][2]) + ",")
                point_zero = (str(self.ceiling_list[idx][0]) + " " + str(self.ceiling_list[idx][1]) + " " + str(self.ceiling_list[idx][2]) + ",")
                RoofNode = (str(self.ceiling_list[idx][0]) + " " + str(self.ceiling_list[idx][1]) + " " + str(self.ceiling_list[idx][2]) + ",")
                FootNode = (str(self.floor_list[idx][0]) + " " + str(self.floor_list[idx][1]) + " " + str(self.floor_list[idx][2]) + ",")

                WallSurfaceInfo = "{one}{two}{three}{four}{default}".format(one=point_one, two=point_two, three=point_three, four=point_four, default=point_zero)
                RoofSurfaceInfo += "{RoofNode}".format(RoofNode=RoofNode)
                FootSurfaceInfo += "{FootNode}".format(FootNode=FootNode)
                surface = self.makePolygonz(WallSurfaceInfo)
                self.makeCityObject(34)
                WallSurfaceGmlid = self.getSurfaceGmlID() + "_wallSurface_" + str(nextidx)

                cursor.execute(
                    "INSERT INTO citydb.surface_geometry (id, gmlid, parent_id, root_id, geometry, cityobject_id) values (%s, %s, %s, %s, %s, %s)",
                    (int(self.getSurfaceGeometryID() + 1), WallSurfaceGmlid, rootSurfaceID, rootSurfaceID, surface,
                     int(self.getCityObjectID())))
                product.commit()
                self.updateEnvelop(WallSurfaceInfo, int(self.getCityObjectID()))
                idx += 1
                if idx == len(self.ceiling_list) - 1:

                    defaultRoofSurface = "{RoofFirstSurface}".format(RoofFirstSurface=(str(self.ceiling_list[0][0]) + " " + str(self.ceiling_list[0][1]) + " " + str(self.ceiling_list[0][2]) + ","))
                    defaultFootSurface = "{FootFirstSurface}".format(FootFirstSurface=(str(self.floor_list[0][0]) + " " + str(self.floor_list[0][1]) + " " + str(self.floor_list[0][2]) + ","))
                    RoofSurfaceInfo += defaultRoofSurface
                    FootSurfaceInfo += defaultFootSurface
                    surface = self.makePolygonz(RoofSurfaceInfo)
                    self.makeCityObject(33)
                    RoofSurfaceGmlid = self.getSurfaceGmlID() + "_roofSurface_" + str(self.getSurfaceGeometryID() + 1)
                    cursor.execute(
                        "INSERT INTO citydb.surface_geometry (id, gmlid, parent_id, root_id, geometry, cityobject_id) values (%s, %s, %s, %s, %s, %s)",
                        (int(self.getSurfaceGeometryID() + 1), RoofSurfaceGmlid, rootSurfaceID, rootSurfaceID, surface,
                         int(self.getCityObjectID())))
                    product.commit()
                    self.updateEnvelop(RoofSurfaceInfo, int(self.getCityObjectID()))
                    self.makeCityObject(32)
                    FootSurfaceGmlid = self.getSurfaceGmlID() + "_footSurface_" + str(self.getSurfaceGeometryID() + 1)
                    surface = self.makePolygonz(FootSurfaceInfo)
                    cursor.execute(
                        "INSERT INTO citydb.surface_geometry (id, gmlid, parent_id, root_id, geometry, cityobject_id) values (%s, %s, %s, %s, %s, %s)",
                        (int(self.getSurfaceGeometryID() + 1), FootSurfaceGmlid, rootSurfaceID, rootSurfaceID, surface,
                         int(self.getCityObjectID())))
                    product.commit()
                    self.updateEnvelop(FootSurfaceInfo, int(self.getCityObjectID()))
                    # makeTexture()
                    break
    def MakeRoomObject (self):

        '''
        1. 26 - building object
        2. 41 - room
        3. 31 - interior wallsurface
        4. 32 - floor
        5. 30 - ceilling
        '''
        # '''roof, wall, ground'''

        # objectid = {"ceiling": 30,
        #             "wall": 31,
        #             "floor": 32
        #             }
        global check_first_time
        global roomCount
        if check_first_time:
            self.makeCityObject(26)
            check_first_time = False
        # self.makeBuildingSurfaceGeometry()
        roomCount += 1
        self.makeRoomObject(41)
        self.makeSurfaceGeometry()
        self.updateBuildingEnvelop()

        logger.info("Finish to make Room Object : " + str(roomCount))


    def visual_graph(self, point_list):



        x = []
        y = []
        # print len(e)
        for index in point_list:
            x.append(index[0])
            y.append(index[2])
        plt.scatter(x, y, label="stars", color="green",
                    marker="*", s=50)
        plt.plot(x, y)
        plt.legend()
        plt.show()


