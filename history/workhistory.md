팀프로젝트기록

07/15

https://spyjetson.blogspot.com/2020/06/jetson-nano-yolov4-installation.html 수행


https://www.youtube.com/watch?v=PblUjNyaQzo pycharm linux download trial


May '19
OK, I got my Jetson Nano yesterday and have resolved the issue of getting Processing working.
Download the Zulu JDK from here:
https://www.azul.com/downloads/zulu-embedded/ 7
You need a 64-bit Arm build. I used the JDK 8 one for simplicity.
Untar the download using tar -xvf zulu8.38.0.162-ca-jdk1.8.0_212-linux_aarch64.tar.gz
In the Processing directory, there is a java sub-directory. Remove this or move it to something like java-bad.
Copy the untared zulu8 directory to the processing directory and rename it java
Run processing. It will whinge about it not being an Oracle JDK but that doesn’t matter, Processing will now work.

Pycharm 설치 이전에 jdk 설치


sudo apt-get install openjdk-8-jre

sudo apt-get install openjdk-8-jdk


Java-1.8.0-openjdk-arm64
Java-8-openjdk-arm64

폴더 이동
$ cp -R <source_folder> <destination_folder> 하니깐 잘됨오예쓰

Python 파일 실행은 python 파일 우클릭 후 실행하면 된다.

Pycharm install sudo로 하고 inherit global 이거 체크해야함
오케이
Cv2version 까지완료


Sudo apt-get install v4l-utils

Gstreamer 가 제대로 깔려있는지 확인하기위해 
apt-get install libgstreamer1.0-0 gstreamer1.0-plugins-base gstreamer1.0-plugins-good gstreamer1.0-plugins-bad gstreamer1.0-plugins-ugly gstreamer1.0-libav gstreamer1.0-doc gstreamer1.0-tools gstreamer1.0-x gstreamer1.0-alsa gstreamer1.0-gl gstreamer1.0-gtk3 gstreamer1.0-qt5 gstreamer1.0-pulseaudio
실행하였음

담배피고 갔다오니깐 됨

07/16


Pycharm venv에 package 추가가 잘 안되어서 검색

Official tensorflow for Jetson nano 설치 

AastaLLL
Moderator
5 
Mar '19
Our official TensorFlow release for Jetson Nano!
Python 3.6+JetPack4.4
sudo apt-get install libhdf5-serial-dev hdf5-tools libhdf5-dev zlib1g-dev zip libjpeg8-dev liblapack-dev libblas-dev gfortran
sudo apt-get install python3-pip
sudo pip3 install -U pip
sudo pip3 install -U pip testresources setuptools numpy==1.16.1 future==0.17.1 mock==3.0.5 h5py==2.9.0 keras_preprocessing==1.0.5 keras_applications==1.0.8 gast==0.2.2 futures protobuf pybind11
# TF-2.x
$ sudo pip3 install --pre --extra-index-url https://developer.download.nvidia.com/compute/redist/jp/v44 tensorflow==2.2.0+nv20.6
# TF-1.15
$ sudo pip3 install --pre --extra-index-url https://developer.download.nvidia.com/compute/redist/jp/v44 ‘tensorflow<2’




https://github.com/dusty-nv/jetson-inference/   이거 까지 수행하였음./


표윤석 ROS 강의 유튜브

마스터 구동 XMLRPC(XML-Remote Processing 을 해야지 메시지 통신 가능
https://www.youtube.com/watch?v=skqzz_xQQXg&list=PLRG6WP3c31_VIFtFAxSke2NG_DumVZPgw&index=4

ubunta version 18.04
Python v = 2.7.17
Ros v =1.14.6 
 ROS Desktop-Full Install: (Recommended)
http://wiki.ros.org/kinetic/Installation/Ubuntu >>>xxxx 안됌
이거 ubuntu랑 버전이 안맞아서 안됬음 melodic morenia 로 다운받아야됌
http://wiki.ros.org/melodic/Installation/Ubuntu

Setup your sources.list
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
 
 
Set up your keys
sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
Installation
sudo apt update
Desktop-Full Install: (Recommended) : ROS, rqt, rviz, robot-generic libraries, 2D/3D simulators and 2D/3D perception
sudo apt install ros-melodic-desktop-full
여기까지 함 - 상우

Environment setup
It's convenient if the ROS environment variables are automatically added to your bash session every time a new shell is launched:
echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc
source ~/.bashrc
If you have more than one ROS distribution installed, ~/.bashrc must only source the setup.bash for the version you are currently using.
If you just want to change the environment of your current shell, instead of the above you can type:
source /opt/ros/melodic/setup.bash

첫번째꺼 실행 아무 결과가 안떠서 2번째 실행 결과가 안떠서 그냥 다음 진행

Dependencies for building packages
Up to now you have installed what you need to run the core ROS packages. To create and manage your own ROS workspaces, there are various tools and requirements that are distributed separately. For example, rosinstall is a frequently used command-line tool that enables you to easily download many source trees for ROS packages with one command.
To install this tool and other dependencies for building ROS packages, run:
sudo apt install python-rosdep python-rosinstall python-rosinstall-generator python-wstool build-essential
Initialize rosdep
Before you can use many ROS tools, you will need to initialize rosdep. rosdep enables you to easily install system dependencies for source you want to compile and is required to run some core components in ROS. If you have not yet installed rosdep, do so as follows.
sudo apt install python-rosdep
With the following, you can initialize rosdep.
sudo rosdep init
rosdep update
home/sangwoo/.ros/rosdep/sources.cache - 경로인듯 melodic version
Installing and Configuring Your ROS Environment
Managing Your Environment
During the installation of ROS, you will see that you are prompted to source one of several setup.*sh files, or even add this 'sourcing' to your shell startup script. This is required because ROS relies on the notion of combining spaces using the shell environment. This makes developing against different versions of ROS or against different sets of packages easier.
If you are ever having problems finding or using your ROS packages make sure that you have your environment properly setup. A good way to check is to ensure that environment variables like ROS_ROOT and ROS_PACKAGE_PATH are set:
$ printenv | grep ROS
ROS_ETC_DIR = /opt/ros/melodic/etc/ros
ROS_ROOT = /opt/ros/melodic/etc/ros
ROS_MASTER_URI = https://localhost:11311
ROS_VERSION = 1
ROS_PYTHON_VERSION = 2
ROS_PACKAGE_PATE= /opt/ros/melodic/share
ROSLISP_PACKAGE_DIRECTORIES=
ROS_DISTRO=melodic

If they are not then you might need to 'source' some setup.*sh files.
Environment setup files are generated for you, but can come from different places:
ROS packages installed with package managers provide setup.*sh files
rosbuild workspaces provide setup.*sh files using tools like rosws
Setup.*sh files are created as a by-product of building or installing catkin packages
Note: Throughout the tutorials you will see references to rosbuild and catkin. These are the two available methods for organizing and building your ROS code. rosbuild is not recommended or maintained anymore but kept for legacy. catkin is the recommended way to organise your code, it uses more standard CMake conventions and provides more flexibility especially for people wanting to integrate external code bases or who want to release their software. For a full break down visit catkin or rosbuild.
If you just installed ROS from apt on Ubuntu then you will have setup.*sh files in '/opt/ros/<distro>/', and you could source them like so:
$ source /opt/ros/<distro>/setup.bash

Create a ROS Workspace
These instructions are for ROS Groovy and later. For ROS Fuerte and earlier, select rosbuild.
Let's create and build a catkin workspace:
$ mkdir -p ~/catkin_ws/src
$ cd ~/catkin_ws/
$ catkin_make
The catkin_make command is a convenience tool for working with catkin workspaces. Running it the first time in your workspace, it will create a CMakeLists.txt link in your 'src' folder.

To make sure your workspace is properly overlayed by the setup script, make sure ROS_PACKAGE_PATH environment variable includes the directory you're in.
$ echo $ROS_PACKAGE_PATH
/home/youruser/catkin_ws/src:/opt/ros/kinetic/share
경로 체크 완료-.

Creating a catkin Package
This tutorial will demonstrate how to use the catkin_create_pkg script to create a new catkin package, and what you can do with it after it has been created.
First change to the source space directory of the catkin workspace you created in the Creating a Workspace for catkin tutorial:
# You should have created this in the Creating a Workspace Tutorial
$ cd ~/catkin_ws/src
Now use the catkin_create_pkg script to create a new package called 'beginner_tutorials' which depends on std_msgs, roscpp, and rospy:
$ catkin_create_pkg beginner_tutorials std_msgs rospy roscpp
Building a catkin workspace and sourcing the setup file
Now you need to build the packages in the catkin workspace:
$ cd ~/catkin_ws
$ catkin_make
그 아래는 cmake커스텀마이징이라 따로 하지않고 workspack 공간만들기까지
이건 왜 해야되는지 모름 찾아봐야됌 안해도 될수도
----------------------- 따로 안해도 됨 그냥 패키지들 간에 의존성? 연관성? 보는거-------
$ rospack depends1 beginner_tutorials
roscpp
rospy
std_msgs
$ roscd beginner_tutorials
$ cat package.xml
<package format="2">
...
  <buildtool_depend>catkin</buildtool_depend>
  <build_depend>roscpp</build_depend>
  <build_depend>rospy</build_depend>
  <build_depend>std_msgs</build_depend>
...
</package>
$ rospack depends1 rospy
genpy
roscpp
rosgraph
rosgraph_msgs
roslib
std_msgs
----------------------------여기까지-----------------------------



Python package 2.7로 빌드했기 때문에, 3으로 다시 빌드함
https://gist.github.com/drmaj/20b365ddd3c4d69e37c79b01ca17587a 참고하였음
그리고 설치하는중에 etc/ros/rosdep/sources.list.d/20-default.list 를 삭제해달라고 하여 삭제하고 sudo rosdep init 
rosdep update를 수행하였음

https://www.miguelalonsojr.com/blog/robotics/ros/python3/2019/08/20/ros-melodic-python-3-build.html

굿

https://catkin-tools.readthedocs.io/en/latest/installing.html


Install ROS melodic with opencv4++version
https://github.com/ros-perception/vision_opencv/issues/272

I was able to successfully compile cv_bridge with opencv4 below are the rough notes of what i did:
Add set (CMAKE_CXX_STANDARD 11) to your top level cmake
In cv_bridge/src CMakeLists.txt line 35 change to if (OpenCV_VERSION_MAJOR VERSION_EQUAL 4)
In cv_bridge/src/module_opencv3.cpp change signature of two functions
3.1) UMatData* allocate(int dims0, const int* sizes, int type, void* data, size_t* step, int flags, UMatUsageFlags usageFlags) const
to
UMatData* allocate(int dims0, const int* sizes, int type, void* data, size_t* step, AccessFlag flags, UMatUsageFlags usageFlags) const
3.2) bool allocate(UMatData* u, int accessFlags, UMatUsageFlags usageFlags) const
to
bool allocate(UMatData* u, AccessFlag accessFlags, UMatUsageFlags usageFlags) const



