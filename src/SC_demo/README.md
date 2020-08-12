![pinsout](https://user-images.githubusercontent.com/17999237/63635418-58978180-c69d-11e9-9e7e-89065b3c54d7.png)

# *PinSout* (*P*oint *IN* *S*pace *OUT*)
Accelerating 3D Indoor Space Construction from Point Clouds with Deep Learning

## Introduction
In general, the procedure for creating a 3D model has five steps that are Data Segmentation, Components separation, Surface generation, Components assembling and Alignment. 
Actually, it is performed using man-power, all tasks involve very high time-consuming and labor-intensive. 
We think about how can improve the processes quickly and easily.

So, we start a project to generate 3d models from raw point data automatically.
Before we start, we need two pieces of background.

  * ### PointCloud
    **Point cloud** is a set of data point which is reflected a real-world by 3D scanning. 
    The important thing is point cloud can include several features, including geometry information, color, intensity and so on.
    Specially, we can use geometry information of point cloud data for constructing a 3D model
  * ### [CityGML]( https://www.opengeospatial.org/standards/citygml )
    **OGC CityGML** is an open data model and XML-based format for the storage and exchange of semantic 3D city models. 
    It is an application schema for the Geography Markup Language version 3.1.1 (GML3), the extendible **international standard** for spatial data exchange issued by the **Open Geospatial Consortium (OGC)** and the **ISO TC211**. 
    The aim of the development of CityGML is to reach a common definition of the basic entities, attributes, and relations of a 3D city model.

From the process of manually building the 3D model(**CityGML**) from the point cloud data, we derived the three steps: Semantic Segmentation, Polygonization, and Featurization. 

1. ***Semantic Segmentation*** - Classify semantics from point
2. ***Polygonization*** - Construct polygons from point
3. ***Featurizaiotn*** - Mapping between semantic features and surfaces

So, in this project, our purpose is making each step results automatically as below figure.
<div>
  <img width="800" src=https://user-images.githubusercontent.com/10336074/63823756-e7ebb000-c98f-11e9-97a0-8454cf0be5e1.png>
</div>

## Dependencies
We used the following open sources in each step (*Semantic segmentation, Polygonization, and Featurizaiotn*).
1. [PointNet] - Deep Learning on Point Sets for 3D Classification and Segmentation
2. [Python-PCL] - Small python binding to the point cloud library
3. [3DCityDB] - Free 3D geodatabase to store, represent, and manage virtual 3D city models

  * #### PointNet
      Deep learning can help to analyze massive point cloud data. 
      In terms of classification, clustering, segmentation, annotation and so on. 
      PointNet is a popular open deep neural network for analyzing point cloud data. 
      We can segment the indoor components such as walls, ceilings, floor, windows, and door. 
      Using the Stanford data to do the test.

  * #### Point Cloud Library (PCL)
      PCL is the open-source project for 2D/3D image and point cloud processing. 
      it contains numerous state-of-the-art algorithms and create surfaces from point clouds and visualize them. 
      Usually, the indoor space is a closed space consisting of a planar surface, so we are using the RANSAC to extract the plane.

  * #### 3DCityDB
      Using the 3DCityDB to deal with the CityGML. 
      We are mapping the geometry and semantic using the table defined in 3DCityDB. 
      3DCityDB is the free geodatabase to store, represent, and manage virtual 3D city models on top of a standard spatial relational database. 
      Database schema implements the CityGML standard.

## System Requirements
This release has been tested on Linux Ubuntu 16.04 with
> Anaconda - Python 2.7.6  
> PostgreSQL DBMS >= 9.3 with PostGIS extension >= 2.0

## Installation
* [PostgreSQL with PostGIS](https://www.postgresql.org/download/)

* [Anaconda](https://www.anaconda.com/distribution/)

* [PointNet]  
   
* [Python-PCL]  

* [3DCityDB](https://www.3dcitydb.org/3dcitydb/d3ddatabase/)  

* [3DCityDB importer&exporter]

* [FZK Viewer]

* [CloudCompare]

## How to use
You can browse the [document](https://breeze0512.github.io/PinSout/) for more information about function.

## Process of SC-Demo

### -) Data information
* Collected PointCloud data


### 1) Preparing to use **PointNet** 

1. Run [PointNet] and go to the **sem_seg** folder.
    * Download 3D indoor parsing dataset (S3DIS Dataset) for testing and visualization. 
    Dataset version 1.2 is used in this work.
    * Before we start collect_indoor3d_data.py, we change the entry in the **"meta/class_names.txt"** to ceiling, floor, wall, door and window
    * Change the value of **"g_class2color and g_easy_view_labels"** in the 
    ```sh
    $ python collect_indoor3d_data.py
    ```
    ```sh
    $ python gen_indoor3d_h5.py
    ```  
    * To prepare your HDF5 data, you need to firstly download 3D indoor parsing dataset and then use training if no model has been learned

2. Training  
    * Change the value of **"NUM_CLASSES"** and directory path in "train.py"
    ```python
    """ NUM_CLASSES = 13 """
    NUM_CLASSES = 5
    ALL_FILES = provider.getDataFiles('Your indoor3d_sem_seg_hdf5_data path/all_files.txt')
    room_filelist = [line.rstrip() for line in open('Your indoor3d_sem_seg_hdf5_data path/room_filelist.txt')]
    ```
    * Once you have downloaded prepared HDF5 files or prepared them by yourself, to start training:
    ```sh
    $ python train.py --log_dir log_5cls --test_area 6
    ```
    In default, a simple model based on vanilla PointNet is used for training. 
    Area_6 is used for the test set.

3. Test  
    * Testing requires a download of 3D indoor parsing data and preprocessing with collect_indoor3d_data.py

    After training, use batch_inference.py command to segment rooms in the test set. 
    In this work, we use 6-fold training that trains six models. 
        
        For model_1, area_2 ~ _6 are used as the training data set, and area_1 is used as the test data set. 
        For model_2, area_1, _3 ~ _6 are used as the training data set and area_2 is used as the test data set
        ...
         
    Note that S3DIS dataset paper uses a different 3-fold training, which was not publicly announced at the time of our work.
    * Chage the value of **"NUM_CLASSES"** in the "batch_inference.py" 
    ```python
    """ NUM_CLASSES = 13 """
    NUM_CLASSES = 5
    ```
    For example, to test model_6, using the below command:
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

### 2) Running the **PlyTo3Dmodel**
1. Run PlyTo3Dmodel.py
```sh
$ python PlyTo3Dmodel.py path_of_your_pointcloud_data
```
2. Dividing the entire PointCloud data into a certain size
    * default_x, defulat_y = 10.0
3. Move data of each area to the origin
4. Convert data from all areas to .npy format
5. Perform semantic segmentation of [PointNet] using the converted data
```python
"""
Run PinSout
    model_path: Path with the trained model needed to do Semantic segmentation.
    out_filename: The path where the 3D model will be saved
    npy_list: List of split PointCloud information
    min_list: Minimum value of each PointCloud data
"""
batch_inference.evaluate(model_path, out_filename, npy_list, min_list)

```

### 3) Generate CityGML data from point cloud 
1. Run **PostgreSQL pgadmin**

2. Run **PinSout**  
    1. ***Semantic Segmentation*** - Classify semantics from PointCloud
    ```python
    """
    sem_seg/bactch_inference.py
    1) Classify A data using trained models
        - Basic model : log_6cls_test16 (ceiling, floor, wall, chair, desk)
        - If the model does not exist or is newly created
            1) Preparing to use **PointNet** 
    2) The classified results are stored in each classification item array
    3) Use PCL-RANSAC function to extract the surface of each item
    Wall 
    """
    if len(wall_list) != 0:
        temp_value_3 = np.asarray(wall_list)
        temp_value_3 += min_list[each_i]
        wall_cloud = pcl.PointCloud()
        wall_cloud.from_list(temp_value_3.tolist())
        pcl.save(wall_cloud, os.path.join(DUMP_DIR, os.path.basename(out_filename)) + '_wall' + "_" + str(each_i)+".pcd")
        all_wall += temp_value_3.tolist()
    wall_cloud = pcl.PointCloud()
    wall_cloud.from_list(all_wall)
    ```
    2. ***Polygonization*** - Construct polygons from PointCloud using PCL-RANSAC

    ```python
    """
    plane_ransac_test.py
    1) Clustering the the semantic segmented result data
    2) Finding the plane from each clustered PointCloud data
    3) Finding the intersection points between planes
    """
    wall_surface_list = make_wall_info(wall_cloud)
    ```
    3. ***3DModel*** - Making the 3D model using each surface of clustered data
    ```python
    poly2obj = po.Ply2Obj(wall_surface_list)
    poly2obj.poly_2_obj('All')
    poly2obj.output_dae(dae_filename)
    ```
    3. ***Featurizaiotn*** - Mapping between semantic features and surfaces
    ```python
    make_gml_file2 = gml2.PointCloudToCityGML(ceiling_surface, floor_surface, wall_surface, door_surface, window_surface)
    make_gml_file2.MakeRoomObject()
    ```
    
    
3. Export to CityGML file 
    * Check the result using **pgadmin** or **3DCityDB importer&exporter**.
    * Export the CityGML file using **3DCityDB importer&exporter**.

4. Check the CityGML file  
    * Run the FZK Viewer to visualize the CityGML file.
    * Select and execute the "Local CRS" at the bottom of the Spatial Reference System.
    
## Usage
For detail information about usage, please see the [User Guide](https://github.com/aistairc/PinSout/wiki)
* [Polygonization](https://github.com/aistairc/PinSout/wiki/Make_CityGML_Data)
* [Featurizaiotn](https://github.com/aistairc/PinSout/wiki/PointCloud_To_CityGML)
* [Point list sorting](https://github.com/aistairc/PinSout/wiki/Point_Sort)

## License
This project is licensed under the [MIT](https://opensource.org/licenses/MIT) licenses. - see the [LICENSE](https://github.com/aistairc/PinSout/blob/master/LICENSE) file for details

## Contact
Should you have any questions, comments, or suggestions, please contact me at cho-wijae@aist.go.jp

**_Wijae Cho_**

https://www.airc.aist.go.jp/en/dprt/

[PostgreSQL with PostGIS]: <https://www.postgresql.org/download/>  
[Anaconda]: <https://www.anaconda.com/distribution/>  
[PointNet]: <https://github.com/charlesq34/pointnet>  
[Python-PCL]: <https://github.com/strawlab/python-pcl>  
[3DCityDB]: <https://github.com/3dcitydb/3dcitydb>  
[3DCityDB importer&exporter]: <https://www.3dcitydb.org/3dcitydb/d3dimpexp/>  
[FZK Viewer]: <https://www.iai.kit.edu/1302.php>  
[CloudCompare]: <https://www.danielgm.net/cc/>  

