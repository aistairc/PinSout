# PinSout
Accelerating 3D Indoor Space Construction from Point Clouds with Deep Learning

# Introduction

* [PointNet] - Deep Learning on Point Sets for 3D Classification and Segmentation
* [PCL] - Small python binding to the pointcloud library
* [3DCityDB] - Free 3D geo database to store, represent, and manage virtual 3D city models
  
# System Requirements
This release has been tested on Linux Ubuntu 16.04 with
> Anaconda - Python 2.7.6  
> PostgreSQL DBMS >= 9.3 with PostGIS extension >= 2.0

# Installation
 - Install CUDA 9.0 cuDNN 7.05
 1. install Nvidia drivers
 Checking the GPU drivers (https://www.nvidia.com/Download/index.aspx?) 
 
 * Add nivida drivers ppa and update repos
  ```sh
  $ sudo add-apt-repository ppa:graphics-drivers/ppa
  $ sudo apt-get update
  ```
Re-run apt-get update as root if issues arise (sudo -i)

** install from apt-get, Minium driver version for CUDA 9.0 is 384
```sh
$ sudo apt-get install nivida-384
```
For current install latest driver was 396 which was used

* Restart Verify everything is working by running
```sh
$ nvidia-smi
```
All graphics cards should be detected and using the nividia driver installed
  
  ddd


#



   [PointNet]: <https://github.com/charlesq34/pointnet>
   [PCL]: <https://anaconda.org/sirokujira/python-pcl>
   [3DCityDB]: <https://www.3dcitydb.org/3dcitydb/d3ddatabase/>
