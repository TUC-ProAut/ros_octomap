# ProAut OctoMap

This repository contains three packages:

* [main package](octomap_pa/)
* [messages package](octomap_pa_msgs/)
* [matlab wrapper](octomap_pa_matlab/)

## octomap_pa package

The [main package](octomap_pa/) was designed to automatically remove outdated
voxels from the [original octomap](http://wiki.ros.org/octomap).

We described our motivation and concept on our
[octomap-website](https://www.tu-chemnitz.de/etit/proaut/octo) - here you
will also find two supportive videos.
For further explanations, you may want to have a look at this
[workshop abstract](http://nbn-resolving.de/urn:nbn:de:bsz:ch1-qucosa-226576).

## other packages
The [messages package](octomap_pa_msgs/) only contains the octomap messages,
which you may need for communication with one of our octomap nodes.

The [matlab wrapper](octomap_pa_matlab/) includes an easy-to-use matlab class
and the ros custom messages for matlab.

## Links

### Source code at github
https://github.com/TUC-ProAut/ros_octomap

### ROS packages
ros-kinetic-octomap-pa _(eol)_

ros-melodic-octomap-pa _(upcoming)_ \
ros-melodic-octomap-pa-msgs _(upcoming)_ \
ros-melodic-octomap-pa-matlab _(upcoming)_

ros-noetic-octomap-pa _(upcoming)_ \
ros-noetic-octomap-pa-msgs _(upcoming)_ \
ros-noetic-octomap-pa-matlab _(upcoming)_


## ROS Build-Status and Documentation

ROS-Distribution | Build-Status                                                                                                                                                            | Documentation
-----------------|-------------------------------------------------------------------------------------------------------------------------------------------------------------------------|---------------
Indigo           | EOL April 2019                                                                                                                                                          | [docs.ros.org/indigo](http://docs.ros.org/indigo/api/octomap_pa/html/index.html)
Jade             | EOL May 2017                                                                                                                                                            | [docs.ros.org/jade](http://docs.ros.org/jade/api/octomap_pa/html/index.html)
Kinetic          | EOL April 2021                                                                                                                                                          | [docs.ros.org/kinetic](http://docs.ros.org/kinetic/api/octomap_pa/html/index.html)
Lunar            | EOL May 2019                                                                                                                                                            | [docs.ros.org/lunar](http://docs.ros.org/lunar/api/octomap_pa/html/index.html)
Melodic          | [![Build Status](http://build.ros.org/buildStatus/icon?job=Mdev__octomap_pa__ubuntu_bionic_amd64)](http://build.ros.org/job/Mdev__octomap_pa__ubuntu_bionic_amd64/)     | [docs.ros.org/melodic](http://docs.ros.org/melodic/api/octomap_pa/html/index.html)
Noetic           | [![Build Status](http://build.ros.org/buildStatus/icon?job=Ndev__octomap_pa__ubuntu_focal_amd64)](http://build.ros.org/job/Ndev__octomap_pa__ubuntu_focal_amd64/)       | [docs.ros.org/noetic](http://docs.ros.org/noetic/api/octomap_pa/html/index.html)
