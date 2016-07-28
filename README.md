# Presentation
This project performs a ICP mapping with a graph-based slam from a dataset gathered in underwater enviroement. 

I have tried and installed it on ubuntu 14.04 LTS.
# Requirements

* dlib 18.18
* kalman filter librairy 1.3
* ROS indigo or kinetic 
* rviz
* cmake
* Eigen3 
* Octave (freeware, only used to display the result of the slam) 

## Installation 
### dlib C++ library

Dlib is a modern C++ toolkit containing machine learning algorithms and tools
for creating complex software in C++ to solve real world problems.  See
[dlib website](http://dlib.net for the main project documentation and API reference)

Type the following to compile the librairy after the Download of dlib 18.18:
  first go to the dlib-18.18 folder.
>cd dlib 
>mkdir build 
>cd build 
>cmake ..
>sudo make install

Please refer to README.txt of dlib for more information.

### kalman filter librairy
Please refer to INSTALL.txt of kalman librairy for more information.

### g2o librairy
g2o is an open-source C++ framework for optimizing graph-based nonlinear error functions.
The librairy is a git repository you can find it [here](https://github.com/RainerKuemmerle/g2o).


To use it you need some requirements to build : 
* cmake
* Eigen3

You can install them manualy or follow the full installation [here](http://sayantanfoto.blogspot.se/2015/06/installing-g2o-on-ubuntu.html).

# How to run
To launch the dataset from an abondoned marina near St. Pere Percador, Spain :
>// go to abondonedmarina package 
>mkdir bagfiles
>cd bagfiles 
>// set your data files
>roslaunch abondonedmarina marina.launch file:=myDataSetName.bag

In this package, only the ICP mapping is perform.

To launch the dataset from the Stevens pier on the Hudson river :
>// go to mapping package 
>mkdir bagfiles 
>cd bagfiles
>// set your data files
>roslaunch mapping mapping.launch file:=myDataSetName.bag

In this package you can find the scan matching slam, with feedback on rviz, and a graph-based slam on background. The resuts of the graph-based slam can be find on the Octave folder.

# OctaveFiles folder 
In this folder you can find programs displaying the slam results. Please refer to [README.txt] (filtering process/OctaveFiles/README.txt) on OcataveFiles folder. 

# License
This project is licensed under the MIT License. However, some librairies are available under different license terms. 
Please refer to README.txt of the g2o librairy for more information.
