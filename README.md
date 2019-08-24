![pinsout](https://user-images.githubusercontent.com/17999237/63635418-58978180-c69d-11e9-9e7e-89065b3c54d7.png)

# PinSout
Accelerating 3D Indoor Space Construction from Point Clouds with Deep Learning

# Introduction
In general, the procedure for creating a 3D model has five steps that are Data Segmentation, Componenets separation, Surface generation, Components assembling and Alignment. In the process of creating a 3D model manually, high time-consuming and labor-intensice tasks occur. We think "How can make the process quickly and easily?". So, We are starting a project to automatically generate 3D model from raw point data.

In this project, we create the three steps that are Semantic Segmentation, Polygonization and Featurization. These three steps automaticaly chnged the process of maually building 3D model. We uesd the following open sources in each step.

* [PointNet] - Deep Learning on Point Sets for 3D Classification and Segmentation
* [PCL] - Small python binding to the pointcloud library
* [3DCityDB] - Free 3D geo database to store, represent, and manage virtual 3D city models

<div>
  <img width="800" src=https://user-images.githubusercontent.com/17999237/63635591-14a57c00-c69f-11e9-9a1e-0fa4aa6dc93e.png>
</div>
  
# System Requirements
This release has been tested on Linux Ubuntu 16.04 with
> Anaconda - Python 2.7.6  
> PostgreSQL DBMS >= 9.3 with PostGIS extension >= 2.0

# Installation
> Install CUDA 9.0 cuDNN 7.05
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
2. Install CUDA 9.0 (run file)
```sh
$ sudo apt-get install build-essential
$ sudo apt-get update
$ sudo apt-get upgrade
$ cd ~/Downloads
```
https://developer.nvidia.com/cuda-90-download-archive CUDA Toolkit Download
```sh
$ sudo sh cuda_9.0.176_384.81_linux.run
```
The CUDA runfile installer can be downloaded from https://developer.nvidia.com/cuda-90-download-archive, or using wget in case you can’t find it easily on NVIDIA:
```sh
$ cd
$ wget https://developer.nvidia.com/compute/cuda/9.0/Prod/local_installers/cuda_9.0.176_384.81_linux_run 
$ chmod +x cuda_9.0.196_384.81_linux_run
$ ./cuda_9.0.176_384.81_linux_run --extract=$HOME
```
What you download is a package the following three components:

1) NVIDIA-Linux-x86_64-384.81.run -> NVIDIA driver, which we do NOT need, we already installed the latest one in the prev step. just remove it to be safe
```sh
$ rm NVIDIA-Linux-x86_64-384.81.run
```
2) cuda-linux.9.0.176-########.run -> CUDA 9.0 driver, which we will install
3) cuda-samples.9.0.176-#######.run CUDA 9.0 Samples, which we also want
Install CUDA 9.0
```sh
$ sudo ./cuda-linux.9.0.176-22781540.run
```
Accept the license by scrolling down (press d) and enter ‘accept’ Accept all the defaults (press enter)
Install CUDA samples to verify the install
```sh
$ sudo ./cuda-samples.9.0.176-22781540-linux.run
```
Same as with CUDA Accept the license by scrolling down (press d) and enter ‘accept’ Accept all the defaults (press enter)
Configure the runtime library
```sh
$ sudo bash –c “echo /usr/local/cuda/lib64/”
> /etc/ld.so.conf.d/cuda.conf
$ sudo ldconfig
```
Add cuda path to /etc/environment
```sh
$ sudo nano /etc/environment
```
Append ‘:/usr/local/cuda/bin’ at the end of the PATH
reboot and test CUDA using CUDA samples - takes long time and lots of WARN do not worry
```sh
$ cd /usr/local/cuda-9.0/samples
$ sudo make
$ cd /usr/local/cuda-9.0/samples/bin/x86_64/linux/release
$ ./deviceQuery
```
It should output something like this
deviceQuery, CUDA Driver = CUDART, CUDA Driver Version = 9.2, CUDA Runtime Version = 9.0, NumDevs = 3 Result = PASS

3. Install cuDNN 7.3 for CUDA 9.0
Go to the cuDNN download page (need registration) and select the latest cuDNN 7.0.* version made for CUDA 9.0. Download all 3 .deb files: the runtime library, the developer library, and the code samples library for Ubuntu 16.04. In your download folder, install them in the same order: Go to the cuDNN download page (need registration) and select the latest cuDNN 7.3.* version made for CUDA 9.0. In the current install we are using cuDNN 7.3.0.29
cuDNN Download(need to login) https://developer.nvidia.com/rdp/cudnn-archive
Download the file
cuDNN v7.3.0 Library for Linux
cuDNN v7.3.0 Runtime Library for Ubuntu 16.04(Deb)
cuDNN v7.3.0 Developer Library for Ubuntu 16.04(Deb)
cuDNN v7.3.0 Code Samples and User Guide for Ubuntu 16.04(Deb)

cuDNN
```sh
$ cd Downloads/
$ tar –xavf cudnn-9.0-linux-x64-v7.tgz
```
다음의 파일을 CUDA Toolkit 디렉토리에 복사하고 파일권한을 변경
```sh
$ sudo cp cuda/include/cudnn.h /use/local/cuda/include
$ sudo cp cuda/lib64/libcudnn* /usr/local/cuda/lib64
$ sudo chmod a+r /usr/local/cuda/include/cudnn.h /usr/local/cuda/lib64/libcudnn*
```
Download all 3 .deb files: the runtime library, the developer library, and the code samples library for Ubuntu 16.04. Install them in the following order runtime, developer and code samples
```sh
$ sudo dpkg –i libcudnn7_7.3.0.29-1+cuda9.0_amd64.deb
$ sudo dpkg –i libcudnn7-dev_7.3.0.29-1+cuda9.0_amd64.deb
$ sudo dpkg –i libcudnn7-doc_7.3.0.29-1+cuda9.0_amd64.deb
```
Verify install by coping samples to home and compiling MNIST
```sh
$ cp –r /usr/src/cudnn_samples_v7/ ~.
$ cd ~/cudnn_samples_v7/mnistCUDNN.
$ make clean && make.
# Process 4 Configure CUDA and cuDNN paths
$ ./mnistCUDNN
```
If installed correctly you should see Test passed! at the end of the output

4. Configure CUDA and cuDNN paths
Put the following line in the end or your bashrc file
```sh
$ nano ~/.bashrc
$ export LD_LIBRARY_PATH=”$LD_LIBRARY_PATH:/usr/local/cuda/extras/CUPTI/lib64”
$ source ~/.bashrc
```
5. Install pip
```sh
$ sudo apt install python-pip
```
> Installing Anaconda on Ubuntu 16.04

First, go to the Anaconda homepage and download the installation script file.
Then go to the terminal and enter the following command : 
```sh
$ bash Anaconda3-5.0.1-Linux-x86_64.sh
```
Slowly proceed with the installation because it requires typing in turn the Enter key and Yes.
The installation path will normally be specified as /home/[userid]/anaconda3, then pass registration will be made to .bashrc.
After that,
```sh
$ source ~/.bashrc
```
You can run .bashrc one more time through the command.
You can also check if commands such as cona list operate normally.

> Creating a virtual environment

Check all available versions of Python.
```sh
$ conda search “^python$”
```
The version of Python you want to use, and create an anaconda environment.
```sh
$ conda create --name envname python=2.7.9
```
Enable the created environment.
```sh
$ source activate envname
```
> Install PointNet

Install Tensorflow gpu version
```sh
$ pip install tensorflow-gpu==1.8.0
$ pip install numpy
```sh
To install h5py for python
```sh
$ sudo apt-get install libhdf5-dev
$ pip install h5py
```sh
Install python-pcl 1.7.2
https://ubuntu.pkgs.org/16.04/ubuntu-universe-amd64/pcl-tools_1.7.2-14build1_amd64.deb.html
```sh
$ sudo apt install libpcl-dev –y
$ cd ~/Downloads
$ wget http://launchpadlibrarian.net/209530212/libeigen3-dev-3.2.5-4_all.deb
$ sudo dpkg -i libeigen3-dev_3.2.5-4_all.deb
$ sudo apt-mark hold libeigen3-dev
$ cd
$ pip install cython==0.22
$ sudo add-apt-repository ppa:v-launchpad-jochen-sprickerhof-de/pcl
$ sudo apt-get update
$ sudo apt-get install libpcl-all
```sh
If an ERROR occurs in the libpcl-all
Manually install related libraries with this command
```sh
$ sudo apt-cache search libpcl
```
Ex) $ sudo apt-get install libpcl-common1.7
```sh
$ git clone https://github.com/NLeSC/python-pcl.git
$ python setup.py build_ext –i
$ python setup.py install
# Test if PCL is available
$ cd python_pcl/examples
$ python kdtree.py
```sh
Note : https://github.com/NLeSC/python-pcl
PointNet repository download.
Move the semantic segmentation of the pointnet to the folder where you want to run it.
```sh
$ cd
$ git clone https://github.com/charlesq34/pointnet.git
$ cd pointnet/sem_seg/
```
Download if dataset does not exist
Dataset : 
Download prepared HDF5 data for training:
only hdf5 train file
```sh
$ sh download_data.sh
```
Download 3D indoor parsing dataset (S3DIS Dataset) for testing and visualization. Version 1.2 of the dataset is used in this work.
```sh
$ python collect_indoor3d_data.py
$ python gen_indoor3d_h5.py
```sh
To prepare your own HDF5 data, you need to firstly download 3D indoor parsing dataset and then use
Training if no model has been learnt
Training : 
Once you have downloaded prepared HDF5 files or prepared them by yourself, to start training:
```sh
$ python train.py --log_dir log6 --test_area 6
```
In default a simple model based on vanilla PointNet is used for training. Area 6 is used for test set. “””
Testing : 
Testing requires download of 3D indoor parsing data and preprocessing with collect_indoor3d_data.py

After training, use batch_inference.py command to segment rooms in test set. In our work we use 6-fold training that trains 6 models. For model 1, area2-6 are used as train set, area1 is used as test set. For model2, area1, 3-6 are used as train set and area2 is used as test set... Note that S3DIS dataset paper uses a different 3-fold training, which was not publicly announced at the time of our work.
For example, to test model6, use command:
```sh
$ python batch_inference.py --model_path log_5cls/model.ckpt --dump_dir log_5cls/dump --output_filelist log_5cls/output_filelist.txt --room_data_filelist meta/area_data_label.txt --visu
```


   [PointNet]: <https://github.com/charlesq34/pointnet>
   [PCL]: <https://anaconda.org/sirokujira/python-pcl>
   [3DCityDB]: <https://www.3dcitydb.org/3dcitydb/d3ddatabase/>
