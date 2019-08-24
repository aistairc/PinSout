![pinsout](https://user-images.githubusercontent.com/17999237/63635418-58978180-c69d-11e9-9e7e-89065b3c54d7.png)

# PinSout
Accelerating 3D Indoor Space Construction from Point Clouds with Deep Learning

# Introduction
In general, the procedure for creating a 3D model has five steps that are Data Segmentation, Componenets separation, Surface generation, Components assembling and Alignment. In the process of creating a 3D model manually, high time-consuming and labor-intensice tasks occur. We think "How can make the process quickly and easily?". So, we are starting a project to automatically generate 3D model from raw point data.

In this project, we create the three steps that are Semantic Segmentation, Polygonization and Featurization. These three steps automaticaly chnged the process of maually building 3D model.

The three steps produce the following result:
1. Semantic Segmentation - Classify sematntics from point
2. Polygonization - Construct polygons from point
3. Featurizaiotn - Mapping between semantic faeatures and surfaces

We uesd the following open sources in each step.
1. [PointNet] - Deep Learning on Point Sets for 3D Classification and Segmentation
2. [PCL] - Small python binding to the pointcloud library
3. [3DCityDB] - Free 3D geo database to store, represent, and manage virtual 3D city models

This is PinSout result.
<div>
  <img width="800" src=https://user-images.githubusercontent.com/17999237/63635662-c8a70700-c69f-11e9-8fda-99881e6be107.png>
</div>
  
# System Requirements
This release has been tested on Linux Ubuntu 16.04 with
> Anaconda - Python 2.7.6  
> PostgreSQL DBMS >= 9.3 with PostGIS extension >= 2.0

# Installation


   [PointNet]: <https://github.com/charlesq34/pointnet>
   [PCL]: <https://anaconda.org/sirokujira/python-pcl>
   [3DCityDB]: <https://www.3dcitydb.org/3dcitydb/d3ddatabase/>
