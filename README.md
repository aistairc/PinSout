![pinsout](https://user-images.githubusercontent.com/17999237/63635418-58978180-c69d-11e9-9e7e-89065b3c54d7.png)

# PinSout
Accelerating 3D Indoor Space Construction from Point Clouds with Deep Learning

## Introduction
In general, the procedure for creating a 3D model has five steps that are Data Segmentation, Components separation, Surface generation, Components assembling and Alignment. Actually, it is performed using man-power, all tasks involve very high time-consuming and labor intensive. We think how can improve the processes quickly and easily?

So, we are starting a project to automatically generate 3d models from raw point data. Before we start, we need two pieces of background.

  * ### PointCloud
    Point cloud is a set of data point which is reflected a real-world by 3D scanning. The important thing is point cloud can include several features, including geometry information, color, intensity and so on.
    specially, we can use geometry information of point cloud data for constructing 3D model
  * ### [CityGML]( https://www.opengeospatial.org/standards/citygml )
    OGC CityGML is an open data model and XML-based format for the storage and exchange of semantic 3D city models. It is an application schema for the Geography Markup Language version 3.1.1 (GML3), the extendible international standard for spatial data exchange issued by the Open Geospatial Consortium (OGC) and the ISO TC211. The aim of the development of CityGML is to reach a common definition of the basic entities, attributes, and relations of a 3D city model.
    **CityGML** is an international **OGC** standard and can be used **free of charge**.

  
In this project, we create the three steps that are Semantic Segmentation, Polygonization and Featurization. These three steps automatically changed the process of maually building 3D model. 

The three steps produce the following result:
1. Semantic Segmentation - Classify semantics from point
2. Polygonization - Construct polygons from point
3. Featurizaiotn - Mapping between semantic features and surfaces

The 3D model is expressed in **CityGML**, which is an **open standardized data model and exchange format to store digital 3D model proposed by OGC**.

### "This is PinSout result."
<div>
  <img width="800" src=https://user-images.githubusercontent.com/17999237/63635662-c8a70700-c69f-11e9-8fda-99881e6be107.png>
</div>

## Dependencies
We used the following open sources in each step.
1. [PointNet] - Deep Learning on Point Sets for 3D Classification and Segmentation
2. [Python-PCL] - Small python binding to the point cloud library
3. [3DCityDB] - Free 3D geo database to store, represent, and manage virtual 3D city models

  * #### PointNet
      Deep learning can help to analyze massive point cloud data. In terms of classification, clustering, segmentation, annotation and so on. PointNet is popular open deep neural network for analyzing point cloud data. We can segment the indoor components such as walls, ceilings, floor, windows and door. Using the Stanford data to do the test

  * #### Point Cloud Library (PCL)
      PCL is the open source project for 2D/3D image and point cloud processing. it Contains numerous state-of-the art algorithms and create surfaces from point clouds and visualize them. Usually the indoor space is a closed space consisting of planar surface, so we are using the RANSAC to extract the plane

  * #### 3DCityDB
      Using the 3DCityDB to deal with the CityGML. We are mapping the geometry and semantic using the table defined in 3DCityDB. 
      3DCityDB is the free geo database to store, represent, and manage virtual 3D city models on top of a standard spatial relational database. Database schema implement the CityGML standard.

## System Requirements
This release has been tested on Linux Ubuntu 16.04 with
> Anaconda - Python 2.7.6  
> PostgreSQL DBMS >= 9.3 with PostGIS extension >= 2.0

## Installation
**PostgreSQL with PostGIS**
* PostgreSQL download page: https://www.postgresql.org/download/

**Anaconda**
* Anaconda download page: https://www.anaconda.com/distribution/

**PointNet**  
* PointNet github address: https://github.com/charlesq34/pointnet
  
**Python-PCL**  
* Python-PCL github address: https://github.com/strawlab/python-pcl

**3DCityDB**  
* 3DCityDB github address: https://github.com/3dcitydb/3dcitydb
* 3DCityDB download page: https://www.3dcitydb.org/3dcitydb/d3ddatabase/

**3DCityDB importer&exporter**  
* 3DCityDB importer&exporter download page: https://www.3dcitydb.org/3dcitydb/d3dimpexp/

**FZK Viewer**  
* FZK Viewer download page: https://www.iai.kit.edu/1302.php

**CloudCompare**
* CloudCompare download page: https://www.danielgm.net/cc/

## Quick Start with Sample Data

### 0. Data information
* [Stanford 2D-3D-Semantics Dataset](http://buildingparser.stanford.edu/dataset.html)
* Office type rooms, except complex types ( Convert all offices in area_1)

### 1. Semantic Segmentation
1. Run [PointNet] and go to the **sem_seg** folder.
    * Download prepared HDF5 data for training: **Only hdf5 train file**
    ```sh
    $ sh download_data.sh
    ```    
    * Download 3D indoor parsing dataset (S3DIS Dataset) for testing and visualization. Version 1.2 of the dataset is used in this work.
    ```sh
    $ python collect_indoor3d_data.py
    $ python gen_indoor3d_h5.py
    ```  
    * To prepare your own HDF5 data, you need to firstly download 3D indoor parsing dataset and then use training if no model has been learnt

2. Training  
    * Once you have downloaded prepared HDF5 files or prepared them by yourself, to start training:
    ```sh
    $ python train.py --log_dir log6 --test_area 6
    ```
    In default a simple model based on vanilla PointNet is used for training. Area 6 is used for test set.

3. Test  
    * Testing requires a download of 3D indoor parsing data and preprocessing with collect_indoor3d_data.py

    After training, use batch_inference.py command to segment rooms in test set. In our work we use 6-fold training that trains 6 models. 
        
        For model_1, area_2 ~ _6 are used as train set, area_1 is used as test set. 
        For model_2, area_1, _3 ~ _6 are used as train set and area_2 is used as test set
        ...
         
    Note that S3DIS dataset paper uses a different 3-fold training, which was not publicly announced at the time of our work.
    For example, to test model_6, use command:
    ```sh
        $ python batch_inference.py --model_path log_5cls/model.ckpt --dump_dir log_5cls/dump --output_filelist log_5cls/output_filelist.txt --room_data_filelist meta/area6_data_label.txt --visu
    ```
    --model_path : The path where model.ckpt file is stored  
    --dump_dir : The folder where forecasted results are stored  
    --output_filelist : Set file path/name where the path of the prediction result is stored
    --room_data_filelist : .npy file path to test  
    --visu : Use when visualizing  

4. Check the result  
    * Check the result with CloudCompare.
### 2. Polygonization & Featurizaiotn
1. Run **Postgresql pgadmin**  

2. Run **PinSout**  
    * Change the data to be trained to area_1.  
    * Change the class name to five (ceiling, floor, wall, window, door).  
    * Start Training  
    * Modify the contents of area_data_label to "data/your result folder/Area_1_office_1.npy" and run it. 
    * Add the PinSout's files in **sem_seg**

3. Check the result  
    * Check the result using **pgadmin** or **3DCityDB importer&exporter**.

4. Export to CityGML file 
    * Export the CityGML file using **3DCityDB importer&exporter**.

5. Check the CityGML file  
    * Run the FZK Viewer to visualize the CityGML file.
    * Select and execute the "Local CRS" at the bottom of the Spatial Reference System.
    
## Usage
* [Make a CityGML data from point cloud](https://github.com/aistairc/PinSout/wiki/Make_CityGML_Data)
    * [Convert a PointCloud to CityGML](https://github.com/aistairc/PinSout/wiki/PointCloud_To_CityGML)
    * [Point sorting](https://github.com/aistairc/PinSout/wiki/Point_Sort)

## License
**PinSout** licensed under the [MIT](https://opensource.org/licenses/MIT)

## Contact
Should you have any questions, comments, or suggestions, please contact me at: cho-wijae@aist.go.jp

**_Wijae Cho_**

https://www.airc.aist.go.jp/en/dprt/

[PointNet]: <https://github.com/charlesq34/pointnet>
[Python-PCL]: <https://github.com/strawlab/python-pcl>
[3DCityDB]: <https://github.com/3dcitydb/3dcitydb>
