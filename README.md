# cohoma_app
Tactic Operator HMI 

## Installation 

### Dependencies

#### Djnn-smala
The HMI is built using the djnn-smala framework

Djnn is a C++ library, which can be found here : https://github.com/lii-enac/djnn-cpp

To build from sources : 
```
sudo apt install git make pkg-config
git clone --depth 1 https://github.com/lii-enac/djnn-cpp.git  
cd djnn-cpp  
make install-pkgdeps  
make -j4
cd ..  
```
Smala is a language built on top of djnn, it can be found here : https://github.com/lii-enac/smala

To build from sources : 
```
git clone https://github.com/lii-enac/smala.git  
cd smala  
make install-pkgdeps  
make -j4
make lib
cd ..  
```
djnn-cpp and smala should be installed in the same directory, e.g : 
```
$ls
djnn-cpp
smala
```
to test a sample project : 
```
cd smala
make -j simplest_test  #or any other directory in smala/cookbook, replace simplest with the directory name
```
#### Ros

The following line should be enough, if there is any issues follow ths instructions at : 
https://docs.ros.org/en/galactic/Installation/Ubuntu-Install-Debians.html
```
sudo apt install ros-galactic-desktop
```

To build ROS2 packages, you will need the tool colcon. Follow the instructions here :
https://colcon.readthedocs.io/en/released/user/installation.html


If have never used ROS before, follow the instructions here to create your first ROS workspace: 
https://docs.ros.org/en/galactic/Tutorials/Workspace/Creating-A-Workspace.html

For convenience sake, consider adding the following line to your ~/.bashrc : 
```
source /opt/ros/galactic/setup.bash
```
It will grant you access to every ROS2 command-line tools, which are required to build packages, whenever you start a terminal.

#### Icare interfaces 
clone this repo  : https://gitlab.com/cohoma-icare/icare_interfaces in the /src folder of a ros workspace.

Icare interfaces depends on the geocraphic_msg package, to install it run 
```
sudo apt-get install ros-galactic-geographic-msgs
```

To indicate where to find icare interfaces either :

Uncomment and update the following line in cohoma_app/srcs.mk
```
PATH_TO_WORKSPACE = /home/ubuntu/djnn_smala_install_debug/dev_ws
```
or add the following line to your .bashrc
```
export PATH_TO_WORKSPACE=/my/path/to/icare_workspace
```
## Run the app

At the moment make -j may output some errors. In the meantime the following make test should run the app
```
cd cohoma_app
make -j
make test
```
