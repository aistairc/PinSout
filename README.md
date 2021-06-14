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
This release has been tested on Linux Ubuntu 18.04 with
> Anaconda3 - Python 2.7.18 
> PostgreSQL DBMS >= 9.3 with PostGIS extension >= 2.0

## Installation

* [PointNet]  
   
* [Anaconda](https://www.anaconda.com/distribution/)

* [Python-PCL]  

* [PostgreSQL with PostGIS](https://www.postgresql.org/download/)

* [3DCityDB](https://www.3dcitydb.org/3dcitydb/d3ddatabase/)  

* [3DCityDB importer&exporter]

* [FZK Viewer]

* [CloudCompare]

## Quick Start with Example

### -) Data information
* [Stanford 2D-3D-Semantics Dataset](http://buildingparser.stanford.edu/dataset.html)
* Office type rooms, except complex types ( Convert all offices in area_1)

### 1) Preparing to use **PinSout**
1. Installing Graphic drivers.  
    1-1) Checking and selecting the Graphic driver
    ```shell script
    $ sudo lshw -C display
    ```
    or 
    ```shell script
    $ ubuntu-drivers devices
    == /sys/devices/pci0000:00/0000:00:01.0/0000:01:00.0 ==
    modalias : pci:v000010DEd00001BE1sv00001028sd0000088Bbc03sc00i00
    vendor   : NVIDIA Corporation
    model    : GP104M [GeForce GTX 1070 Mobile]
    driver   : nvidia-driver-415 - third-party free
    driver   : nvidia-driver-450-server - distro non-free
    driver   : nvidia-driver-435 - distro non-free
    driver   : nvidia-driver-455 - third-party free recommended
    driver   : nvidia-driver-418-server - distro non-free
    driver   : nvidia-driver-410 - third-party free
    driver   : nvidia-driver-450 - distro non-free
    driver   : nvidia-driver-390 - distro non-free
    driver   : nvidia-driver-440-server - distro non-free
    driver   : xserver-xorg-video-nouveau - distro free builtin
    ```
    1-2) Installing the nvidia-driver-455
    First add the following repository:
    ```shell script
    $ sudo add-apt-repository ppa:graphics-drivers/ppa
    $ sudo apt update
    ```
    ```shell script
    $ apt-cache search nvidia | grep nvidia-driver-455
    nvidia-driver-455 - NVIDIA driver metapackage
    $ sudo apt-get install nvidia-driver-455
    $ sudo reboot
    ```
    If a problem occurs during or after the graphics card is installed, delete it and then reinstall it.
    ```shell script
    $ sudo apt --purge autoremove nvidia*
    ```
    1-3) Checking Nvidia-Driver Information
    ```shell script
    $ nvidia-settings
    or
    $ nvidia-smi
    ```
2. Installing CUDA, cuDNN, Tensorflow
    - Checking the version of CUDA, cuDNN and Tensorflow using [CUDA Tookit Version] and [Tensorflow Version]
    - PinSout is testing on CUDA-10.0, cuDNN-7.4.1 and Tesorflow-1.14.0  
    
    2-1) Set up CUDA
    - Downloading the cuda runfile and install from [CUDA Download]
    ```shell script
    $ sudo sh ~/Downloads/cuda_10.0.130_410.48_linux.run
    ```
    - Add cuda path to /etc/environment
    ```shell script
    $ sudo nano /etc/environment
    Append ':/usr/local/cuda/bin' at the end of the PATH
    $ sudo reboot
    ```
    - Testing the CUDA using CUDA samples
    ```shell script
    $ cd /usr/local/cuda-10.0/samples
    $ sudo make
    $ cd /usr/local/cuda-10.0/samples/bin/x86_64/linux/release
    $ ./deviceQuery
    # result:
    deviceQuery, CUDA Driver = CUDART, CUDA Driver Version = 11.1, CUDA Runtime Version = 10.0, NumDevs = 1 Result = PASS
    ```  
    2-2) Set up cuDNN
    - Downloading the cuDNN files from [cuDNN Download] after login
        - cuDNN v7.4.1 Library for Linux  
        - cuDNN v7.4.1 Runtime Library for Ubuntu 18.04(Deb)  
        - cuDNN v7.4.1 Developer Library for Ubuntu 18.04(Deb)  
        - cuDNN v7.4.1 Code Samples and User Guide for Ubuntu 18.04(Deb)  
   
    - Install cuDNN v7.4.1 Library for Linux
    ```shell script
    $ cd Downloads/
    $ tar -xavf cudnn-10.0-linux-x64-v7.4.1.5.tgz
    $ sudo cp cuda/include/cudnn.h /use/local/cuda/include
    $ sudo cp cuda/lib64/libcudnn* /usr/local/cuda/lib64
    $ sudo chmod a+r /usr/local/cuda/include/cudnn.h /usr/local/cuda/lib64/libcudnn*
    ``` 
    - Add the path of CUDA and cuDNN
    ```shell script
    $ nano ~/.bashrc
    $ export LD_LIBRARY_PATH="$LD_LIBRARY_PATH:/usr/local/cuda/lib64"
    $ source ~/.bashrc
    ```
    - Install cuDDN v7.4.1 Runtime Library, Developer Library, Code Samples and User Guide
    ```shell script
    $ sudo dpkg -i libcudnn7_7.4.1.5-1+cuda10.0_amd64.deb
    $ sudo dpkg -i libcudnn7-dev_7.4.1.5-1+cuda10.0_amd64.deb
    $ sudo dpkg -i libcudnn7-doc_7.4.1.5-1+cuda10.0_amd64.deb
    ```
    - Testing the CUDA and cuDNN
    ```shell script
    $ cp –r /usr/src/cudnn_samples_v7/ ~/
    $ cd ~/cudnn_samples_v7/mnistCUDNN
    $ sudo make clean && make
    # or sudo make
    $ ./mnistCUDNN
    ```
     - If there were installed correctly you should see __"Test passed!"__ at the end of the output  
      
    2-3) Set up Tensorflow
    - Installing Anaconda3 on Ubuntu 18.04
    - Download the installation script file from [Anaconda Homepage]
    ```shell script
    $ sudo apt install python-pip
    $ bash ~/Downloads/Anaconda3-2020.07-Linux-x86_64.sh
    # after finish install the Aanaconda3
    $ source ~/.bashrc
    ```
    - Creating a virtual environment in Anaconda3 and install Tensorflow and python-library
    ```shell script
    $ conda create --name "your virtual environment name" python=2.7.18
    $ conda activate "your virtual environment name"
    $ pip install tensorflow-gpu==1.14.0
    $ pip isntall numpy
    $ sudo apt-get install libhdf5-dev
    $ pip isntall h5py
    ```
3. Install python-pcl from [Anaconda Cloud]  
    3-1) Download and install python-pcl
    ```shell script
    $ source activate "your virtual environment name"
    $ conda install -c sirokujira python-pcl --channel conda-forge
    ```
    3-2) Create new link files
    ```shell script
    $ cd ~/anaconda3/envs/"your virtual environment name"/lib
    $ ln -s libboost_system.so.1.64.0 libboost_system.so.1.54.0
    $ ln -s libboost_filesystem.so.1.64.0 libboost_filesystem.so.1.54.0
    $ ln -s libboost_thread.so.1.64.0 libboost_thread.so.1.54.0
    $ ln -s libboost_iostreams.so.1.64.0 libboost_iostreams.so.1.54.0
    ```
4. Install PostgreSQL with PostGIS and PgAdmin4
    
5. Install 3DCityDB and 3DCityDB importer&exporter

    5-1) Downloading the 3DCityDB and 3DCityDB importer&exporter
    - Downloading the 3DCityDB from [here](https://www.3dcitydb.org/3dcitydb/d3ddatabase/)
    - Downloading the 3DCityDB importer&exporter from [here](https://www.3dcitydb.org/3dcitydb/d3dimpexp/)
    
    5-2) Installing the 3DCityDB and 3DCityDB importer&exporter following the document
    - [Chapter 3 Implementation and Installation](https://www.3dcitydb.org/3dcitydb/fileadmin/downloaddata/3DCityDB_Documentation_v4.2.pdf) for installing 3DCityDB and 3DCityDB importer&exporter

### 2) Starting the PinSout
1. Clone this project  
    ```shell script
    git clone https://github.com/aistairc/PinSout.git
    ```  

2. Add the new information of created 3DCityDB

    ```shell script
    cd PinSout/src/citygml
   
    # Add the new information in PointCloud_To_CityGML.py
    user = 'Your id for login to Postgresql'
    password = 'Your password for login to Postgresql'
    host_product = 'IP address of Postgresql'
    dbname = 'Name of 3DCityDB installed in Postgresql'
    port = 'Port number'
    srid = 'The coordinate system number entered when installing 3DCityDB'
    
    # Example
    user = 'postgres'
    password = 'dprt'
    host_product = 'localhost'
    dbname = 'CityModelX'
    port = '5432'
    srid = 'SRID=4326;'
    ```

3. Running the PinSout.sh with three parameters

    3-1) Add the path of CUDA and anaconda virtual environment in PinSout.sh
    ```shell script
    cd PinSout/src
    #add the path to PinSout.sh
    export LD_LIBRARY_PATH=/usr/local/cuda/lib64:$LD_LIBRARY_PATH
    export PATH="/root/anaconda3/envs/"anaconda3 virtual environment name"/bin:$PATH"
    ```
   
    3-2) Running the PinSout.sh
    - First argument : Path of PointCloud data
    - Second argument : Value of distance threshold for Plane RANSAC ( default=0.05 )
    - Third argument : Value of Epsilon for Intersection Function ( default=0.5 )
    ```shell script
    sh PinSout.sh ../data/1000_168/npy_data/dump/samplling_in_d2_wall_.pcd 0.04 0.5
    ```
4. Export the CityGML using 3DCityDB Importer&Exporter
    - Exporting the CityGML following the 3DCityDB Importer&Exporter document
        - [Chapter 5 Importer / Exporter](https://www.3dcitydb.org/3dcitydb/fileadmin/downloaddata/3DCityDB_Documentation_v4.2.pdf)

   
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
[CUDA Tookit Version]: <https://docs.nvidia.com/cuda/cuda-toolkit-release-notes/index.html#cuda-major-component-versions__table-cuda-toolkit-driver-versions>
[CUDA Download]: <https://developer.nvidia.com/cuda-10.0-download-archive>
[cuDNN Download]: <https://developer.nvidia.com/rdp/cudnn-archive>
[Tensorflow Version]: <https://www.tensorflow.org/install/source?hl=en#gpu>
[Anaconda Homepage]: <https://www.anaconda.com/products/individual>
[Anaconda Cloud]: <https://anaconda.org/sirokujira/python-pcl>
