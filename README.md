![pinsout](https://user-images.githubusercontent.com/17999237/63635418-58978180-c69d-11e9-9e7e-89065b3c54d7.png)

# PinSout
Accelerating 3D Indoor Space Construction from Point Clouds with Deep Learning

# Introduction
In general, the procedure for creating a 3D model has five steps that are Data Segmentation, Componenets separation, Surface generation, Components assembling and Alignment. Actually, it is performed using man-power, all tasks involve very high time-consuming and labor intensive. We think how can improve the processes quickly and easily?

So, we are starting a project to automatically generate 3d models from raw point data. Before we start, we need two pieces of background.

  * ### PointCloud
    Point cloud is a set of data point which is reflected a real-world by 3D scanning. The important thing is point cloud can include several features, including geometry information, color, intensity and so on.
    specially, we can use geometry information of point cloud data for constructing 3D model
  * ### CityGML
    gggg
  
In this project, we create the three steps that are Semantic Segmentation, Polygonization and Featurization. These three steps automatically changed the process of maually building 3D model. 

The three steps produce the following result:
1. Semantic Segmentation - Classify sematntics from point
2. Polygonization - Construct polygons from point
3. Featurizaiotn - Mapping between semantic faeatures and surfaces

We uesd the following open sources in each step.
1. [PointNet] - Deep Learning on Point Sets for 3D Classification and Segmentation
2. [PCL] - Small python binding to the pointcloud library
3. [3DCityDB] - Free 3D geo database to store, represent, and manage virtual 3D city models

  * ### Deep Learning (PointNet)
  * ### PCL
  * ### 3DCityDB


The 3D model is expressed in **CityGML**, which is an **open standardized data model and exchange format to store digital 3D model proposed by OGC**.

This is PinSout result.
<div>
  <img width="800" src=https://user-images.githubusercontent.com/17999237/63635662-c8a70700-c69f-11e9-8fda-99881e6be107.png>
</div>
  
# System Requirements
This release has been tested on Linux Ubuntu 16.04 with
> Anaconda - Python 2.7.6  
> PostgreSQL DBMS >= 9.3 with PostGIS extension >= 2.0

# Installation
**1. PointNet**  
* PointNet github address : https://github.com/charlesq34/pointnet
  
**2. Python-PCL**  
* Python-PCL github address : https://github.com/strawlab/python-pcl

**3. 3DCityDB**  
* 3DCityDB github address : https://github.com/3dcitydb/3dcitydb
* 3DCityDB download page : https://www.3dcitydb.org/3dcitydb/d3ddatabase/

**4. 3DCityDB importer&exporter**  
* 3DCityDB importer&exporter download page : https://www.3dcitydb.org/3dcitydb/d3dimpexp/

**5. FZK Viewer**  
* FZK Viewer download page : https://www.iai.kit.edu/1302.php

**6. CloudCompare**
* CloudCompare download page : https://www.danielgm.net/cc/

# Usage
Atfer installation
1. Run pointnet and go to the **sem_seg** folder.
* Download prepared HDF5 data for training:
**Only hdf5 train file**
```sh
$ sh download_data.sh
```
* Download 3D indoor parsing dataset (S3DIS Dataset) for testing and visualization. Version 1.2 of the dataset is used in this work.
```sh
$ python collect_indoor3d_data.py
$ python gen_indoor3d_h5.py
```
* To prepare your own HDF5 data, you need to firstly download 3D indoor parsing dataset and then use
Training if no model has been learnt
2. Trianning  
    * Once you have downloaded prepared HDF5 files or prepared them by yourself, to start training:
```sh
$ python train.py --log_dir log6 --test_area 6
```
In default a simple model based on vanilla PointNet is used for training. Area 6 is used for test set.

3. Test  
    * Testing requires download of 3D indoor parsing data and preprocessing with collect_indoor3d_data.py

After training, use batch_inference.py command to segment rooms in test set. In our work we use 6-fold training that trains 6 models. For model 1, area2-6 are used as train set, area1 is used as test set. For model2, area1, 3-6 are used as train set and area2 is used as test set... Note that S3DIS dataset paper uses a different 3-fold training, which was not publicly announced at the time of our work.
For example, to test model6, use command:
```sh
$ python batch_inference.py --model_path log_5cls/model.ckpt --dump_dir log_5cls/dump --output_filelist log_5cls/output_filelist.txt --room_data_filelist meta/area6_data_label.txt --visu
```
--model_path : Path where model.ckpt file is stored  
--dump_dir : Folder where forecasted results are stored  
--output_filelist : Set file path / name where the path of the prediction result is stored  
--room_data_filelist : .npy file path to test  
--visu : Use when visualizing  

4. Check the result  
    * Check the result with CloudCompare.

5. Run Postgresql pgadmin  

6 . Testing PinSout  
    * Change the data to be trained to area1.  
    * Change the class name to five(ceiling, floor, wall, window, door).  
    * Start Trainning  
    * Modify the contents of area_data_label to "data/your result folder/Area_1_office_1.npy" and run it. 
    * Add the PinSout's files in **sem_seg**

7. Check the result  
    * Check the result using pgadmin or 3DCityDB importer&exporter.

8. Export the CityGML  
    * Export the CityGML file using 3DCityDB importer&exporter.

9. Check the CityGML File  
    * Run the FZK Viewer to visualize the CityGML file.
    * Select and execute the "Unknown SRS" at the bottom of the Spatial Reference System.
    
# Reference
1. Make_CityGML_Data
```
make_gml_data = MakeCityGMLData(pred_cloud, ceiling_cloud, floor_cloud, wall_cloud, door_cloud, window_cloud)
```
|  <center>Name</center> |  <center>Type</center> |  <center>Default</center> |  <center>Description</center> |
|:--------:|:--------:|:--------:|:--------:|
| pred_cloud, ceiling_cloud, floor_cloud, wall_cloud, door_cloud, window_cloud | pcl.PointCloud() |  | pcl.PointCloud() of pcl library |

2. PointCloud_To_CityGML
```
make_gml_file = PointCloudToCityGML(ceiling_point, floor_point, wall_point, door_point, window_point)
```
|  <center>Name</center> |  <center>Type</center> |  <center>Default</center> |  <center>Description</center> |
|:--------:|:--------:|:--------:|:--------:|
| ceiling_point, floor_point, wall_point, door_point, window_point | List |  | each surface's points |

3. Point_Sort
```
point_sort = Point_sort()
sort_result = point_sort.SortPointsClockwise2(point_list, True)
```
|  <center>Name</center> |  <center>Type</center> |  <center>Default</center> |  <center>Description</center> |
|:--------:|:--------:|:--------:|:--------:|
| point_list | List | True | CounterClockWiseSort |



# More Information
OGC CityGML is an open data model and XML-based format for the storage and exchange of semantic 3D city models. It is an application schema for the Geography Markup Language version 3.1.1 (GML3), the extendible international standard for spatial data exchange issued by the Open Geospatial Consortium (OGC) and the ISO TC211. The aim of the development of CityGML is to reach a common definition of the basic entities, attributes, and relations of a 3D city model.

**CityGML** is an international **OGC** standard and can be used **free of charge**.

   [PointNet]: <https://github.com/charlesq34/pointnet>
   [PCL]: <https://anaconda.org/sirokujira/python-pcl>
   [3DCityDB]: <https://www.3dcitydb.org/3dcitydb/d3ddatabase/>
