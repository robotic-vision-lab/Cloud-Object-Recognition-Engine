## Cloud-based Object Recognition Engine (CORE)

CORE is a framework for performing object recognition on 3D point cloud data 
[1]. It includes machine learning classifiers, a Robot Operating System (ROS) 
[2] module, and wraps existing data filters, feature descriptors, and 
segmentation techniques found in the Point Cloud Library (PCL) [3]. The goal of 
CORE is to leverage cloud computing facilities for the purpose of storing data, 
training classifiers, and performing object recognition tasks offloaded by 
network connected robots.

## Building CORE on Ubuntu 14.04

First, install PCL: 

    $ sudo add-apt-repository ppa:v-launchpad-jochen-sprickerhof-de/pcl  
    $ sudo apt-get update  
    $ sudo apt-get install libpcl-all  

Next, build CORE:

    $ cd core  
    $ mkdir build && cd build  
    $ cmake ..  
    $ make  
    $ sudo make install  
    $ echo '/usr/local/lib' | sudo tee /etc/ld.so.conf.d/usr-local-lib.conf > /dev/null  
    $ sudo ldconfig  

Install ROS Indigo [2]. Then, build the ROS module:

    $ cd core/ros  
    $ catkin_make

## Usage 

### Remote server example

#### Server setup 

We use CloudLab [4] as our remote computing facility in this example. We've
created a profile (disk image) named 'ubuntu-14\_04-ros-indigo-full' that 
captures the entire cloud environment. This profile is shared within CloudLab 
under the project 'core-robotics'. It consists of one x86 node running Ubuntu 
14.04 with ROS Indigo installed.

After starting an experiment with the above profile, clone and build CORE:

    $ git clone https://username@github.com/wjbeksi/core  
    $ cd core  
    $ mkdir build && cd build  
    $ cmake ..  
    $ make  
    $ sudo make install  
    $ echo '/usr/local/lib' | sudo tee /etc/ld.so.conf.d/usr-local-lib.conf > /dev/null  
    $ sudo ldconfig  

Next, build the ROS module:

    $ cd core/ros  
    $ source /opt/ros/indigo/setup.bash  
    $ catkin_make  

Then, launch the rosbridge server node:

    $ cd core/ros   
    $ source devel/setup.bash   
    $ roslaunch rosbridge_server rosbridge_websocket.launch  

Finally, in another terminal launch the CORE server nodes:

    $ cd core/ros   
    $ source devel/setup.bash   
    $ roslaunch core_server core_server.launch  

#### Client setup 

Follow the instructions above for building CORE on the client (robot). We'll
assume that you have an RGB-D sensor connected, such as the Asus Xtion, with the
necessary driver modules installed (OpenNI and PrimeSense). Proceed by launching
the ROS OpenNI2 driver:

    $ roslaunch openni2_launch openni2.launch depth_registration:=true 

Then, launch the CORE client nodes: 

    $ cd core/ros   
    $ source devel/setup.bash   
    $ roslaunch core_client core_client.launch server_ip_addr:=xxx.xxx.xxx.xx:9090

where 'xxx.xxx.xxx.xx' is the IP address of the CloudLab server found by using
the command 'if\_config'.

Point clouds captured by the client will be preprocessed (filtered, culled, 
compressed, etc.) and sent to the remote server for classification.

### Standalone classification example

In this example, we show how to use CORE for object classification using
covariance descriptors and a support vector machine (SVM).

#### Computing the covariance descriptor for a set of PCD files

First, create a file named 'pcd\_categories' that contains a list of absolute 
paths to PCD files, one category per line. The covariance files are written out 
for each category under the directory 'covariance\_dir':

    $ compute_covariance pcd_categories covariance_dir  

#### Learning an SVM model based on covariance descriptors

The learning parameter gamma is given as the first argument. The model is 
constructed using the file 'cov\_categories' which contains the absolute paths,
one category per line, to the covariance files created in the previous step:

    $ svm_learn_model 0.005 cov_categories  

#### SVM classification based on covariance descriptors

Now we can predict the classification labels of objects in a point cloud given 
the trained SVM model and a test input PCD file:

    $ svm_predict_class model.txt pc_1.pcd 

## References

[1] W.J. Beksi, J. Spruth and N. Papanikolopoulos, "CORE: A Cloud-based Object Recognition Engine for Robotics", IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS), Hamburg, Germany, 2015.  
[2] Robot Operating System: http://www.ros.org  
[3] Point Cloud Library: http://www.pointclouds.org  
[4] CloudLab: https://www.cloudlab.us
