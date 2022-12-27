# ORB-SLAM2
#### :warning: ไม่สามารถเอาไฟล์ลงได้เนื่องจากขนาดไฟล์เกินจากที่กำหนด
##### $\color[RGB]{196, 43,214}โหลดไฟล์ตั้งต้นได้จาก$ https://github.com/raulmur/ORB_SLAM2

> **Note**
> Want opencv 3+ (Ubuntu18.04) and cmake 3.8+

<p>$\color[RGB]{214, 73, 41}1.$ $\color[RGB]{74, 214, 43}Install$ $\color[RGB]{74, 214, 43}Pangolin$</p>

```
git clone https://github.com/stevenlovegrove/Pangolin.git
```
```
cd Pangolin
```
```
mkdir build
```
```
cd build
```
```
cmake ..
```
```
make
```
<p>$\color[RGB]{214, 73, 41}2.$ $\color[RGB]{74, 214, 43}Install$ $\color[RGB]{214, 123, 43}CHOLMOD$, $\color[RGB]{214, 123, 43}BLAS$, $\color[RGB]{214, 123, 43}LAPACK$ $\color[RGB]{214, 123, 43}and$ $\color[RGB]{214, 123, 43}Eigen3.$</p>

```
sudo apt install libsuitesparse-dev libblas-dev liblapack-dev libeigen3-dev
```
<p>$\color[RGB]{214, 73, 41}3.$ $\color[RGB]{74, 214, 43}Downloads$ $\color[RGB]{196, 43,214}ORB-SLAM2$</p>

```
git clone https://github.com/raulmur/ORB_SLAM2.git
```
```
cd /ORB_SLAM2/Vocabulary
```
```
tar -xzvf ORBvoc.txt.tar.gz
```
#### $\color[RGB]{214, 73, 41}4.$ $\color[RGB]{74, 214, 43}Fix$ $\color[RGB]{74, 214, 43}file$ $\color[RGB]{74, 214, 43}at$ $\color[RGB]{74, 214, 43}download$ $\color[RGB]{74, 214, 43}come$

$\color[RGB]{231, 104,104}Error$ $\color[RGB]{231, 104,104}message:$

` /home/melanie/source/smartcar/orm_slam2/orb_slam2/src/viewer.cc:159:28:error: ' Usleep ' is not declared in this scope
Usleep (3000); `
$\color[RGB]{104, 189,231}Give$ $\color[RGB]{104, 189,231}add$ $\color[RGB]{104, 189,231}this$ $\color[RGB]{104, 189,231}line$ $\color[RGB]{104, 189,231}on$ $\color[RGB]{104, 189,231}most$ $\color[RGB]{104, 189,231}files.cc$ $\color[RGB]{104, 189,231}($ $\color[RGB]{104, 189,231}in$ $\color[RGB]{104, 189,231}the$ $\color[RGB]{104, 189,231}top$ $\color[RGB]{104, 189,231})$
```
include <unistd.h>
```
$\color[RGB]{104, 189,231}in$ $\color[RGB]{104, 189,231}the$ $\color[RGB]{104, 189,231}file$ $\color[RGB]{104, 189,231}folder$ $\color[RGB]{104, 189,231}src$ $\color[RGB]{104, 189,231}as$ $\color[RGB]{104, 189,231}follows:$

`examples/monocular/mono_euroc.cc`
`examples/monocular/mono_kitti.cc`
`examples/monocular/mono_tum.cc`
`examples/rgb-d/rgbd_tum.cc`
`examples/stereo/stereo_euroc.cc`
`examples/stereo/stereo_kitti.cc`
`src/localmapping.cc`
`src/loopclosing.cc`
`src/system.cc`
`src/tracking.cc`
`examples/ros/orb-slam2/src/vr/ViewerAR.cc`
`src/viewer.cc`

$\color[RGB]{231, 104,104}Cuda$ $\color[RGB]{231, 104,104}Error$ $\color[RGB]{231, 104,104}:$

$\color[RGB]{104, 189,231}Cuda-related$ $\color[RGB]{104, 189,231}errors$ $\color[RGB]{104, 189,231}occur$ $\color[RGB]{104, 189,231}when$ $\color[RGB]{104, 189,231}a$ $\color[RGB]{104, 189,231}Cuda-installed$ $\color[RGB]{104, 189,231}machine$ $\color[RGB]{104, 189,231}is$ $\color[RGB]{104, 189,231}compiled,$ $\color[RGB]{104, 189,231}and$ $\color[RGB]{104, 189,231}the$ $\color[RGB]{104, 189,231}modified$ $\color[RGB]{104, 189,231}build.sh$ $\color[RGB]{104, 189,231}are$ $\color[RGB]{104, 189,231}as$ $\color[RGB]{104, 189,231}follows:$
```
cd ORB_SLAM2/Thirdparty/DBow2
```
```
mkdir Build
```
```
cd  Build
```
```
cmake
```
```
make
```
```
cd ORB_SLAM2/Thirdparty/g2o 
```
```
mkdir Build
```
```
cd  Build
```
```
cmake
```
```
make
```
$\color[RGB]{231, 104,104}Error$ $\color[RGB]{231, 104,104}message:$

`/home/melanie/tools/eigen/eigen/src/core/assignevaluator.h:817:3: Error:static Assertion Failed:you_mixed_ different_numeric_types__you_need_to_use_the_cast_method_of_matrixbase_to_cast_numeric_types_explicitly
Eigen_check_binary_compatibiliy (Func,typename actualdsttypecleaned::scalar,typename Src::Scalar);
^
`

$\color[RGB]{104, 189,231}Solution:$

```
nano Thirdparty/g2o/g2o/solvers/linear_solver_eigen.h 
```

<p>$\color[RGB]{104, 189,231}And$ $\color[RGB]{104, 189,231}the$ $\color[RGB]{104, 189,231}following$ $\color[RGB]{104, 189,231}code$</p>

```
Template <typename matrixtype>
Class Linearsolvereigen:public Linearsolver<matrixtype>
{
Public
typedef eigen::sparsematrix<double, eigen::colmajor> Sparsematrix;
typedef eigen::triplet<double> Triplet;
typedef Eigen::P Ermutationmatrix<eigen::D ynamic, Eigen::D ynamic, sparsematrix::index> Permutationmatrix;
```
$\color[RGB]{104, 189,231}Modified$ $\color[RGB]{104, 189,231}to:$
```
Template <typename matrixtype>
Class Linearsolvereigen:public Linearsolver<matrixtype>
{
Public
typedef eigen::sparsematrix<double, eigen::colmajor> Sparsematrix;
typedef eigen::triplet<double> Triplet;
typedef Eigen::P Ermutationmatrix<eigen::D ynamic, Eigen::D ynamic, int> Permutationmatrix;
```
## D435i 
###### add file.ymal in ORB_SLAM2/Examples/RGB-D
```
%YAML:1.0

#--------------------------------------------------------------------------------------------
# Camera Parameters. Adjust them!
#--------------------------------------------------------------------------------------------

# Camera calibration and distortion parameters (OpenCV)
Camera.fx: 614.216064453125
Camera.fy: 613.1551513671875
Camera.cx: 326.0156555175781
Camera.cy: 245.55152893066406

Camera.k1: 0.0
Camera.k2: 0.0
Camera.p1: 0.0
Camera.p2: 0.0
Camera.k3: 0.0

Camera.width: 640
Camera.height: 480

# Camera frames per second
Camera.fps: 30.0

# IR projector baseline times fx (aprox.)
#bf = baseline (in meters) * fx, D435i的 baseline = 50 mm
Camera.bf: 30.711

# Color order of the images (0: BGR, 1: RGB. It is ignored if images are grayscale)
Camera.RGB: 1

# Close/Far threshold. Baseline times.
ThDepth: 40.0

# Deptmap values factor
DepthMapFactor: 1000.0

#--------------------------------------------------------------------------------------------
# ORB Parameters
#--------------------------------------------------------------------------------------------

# ORB Extractor: Number of features per image
ORBextractor.nFeatures: 1000

# ORB Extractor: Scale factor between levels in the scale pyramid  
ORBextractor.scaleFactor: 1.2

# ORB Extractor: Number of levels in the scale pyramid 
ORBextractor.nLevels: 8

# ORB Extractor: Fast threshold
# Image is divided in a grid. At each cell FAST are extracted imposing a minimum response.
# Firstly we impose iniThFAST. If no corners are detected we impose a lower value minThFAST
# You can lower these values if your images have low contrast          
ORBextractor.iniThFAST: 20
ORBextractor.minThFAST: 7

#--------------------------------------------------------------------------------------------
# Viewer Parameters
#--------------------------------------------------------------------------------------------
Viewer.KeyFrameSize: 0.05
Viewer.KeyFrameLineWidth: 1
Viewer.GraphLineWidth: 0.9
Viewer.PointSize:2
Viewer.CameraSize: 0.08
Viewer.CameraLineWidth: 3
Viewer.ViewpointX: 0
Viewer.ViewpointY: -0.7
Viewer.ViewpointZ: -1.8
Viewer.ViewpointF: 500
```
$\color[RGB]{206, 231,104}Realsense$ $\color[RGB]{206, 231,104}D435i$ $\color[RGB]{206, 231,104}ROS$
###### SDK  : https://github.com/IntelRealSense/librealsense/blob/master/doc/distribution_linux.md#installing-the-packages
###### Node : https://github.com/IntelRealSense/realsense-ros#installation-instructions
$\color[RGB]{206, 231,104}ORB-SLAM2$ $\color[RGB]{206, 231,104}+$ $\color[RGB]{206, 231,104}D435i$
###### เว็บไซต์ตั้งต้นตัวอย่าง : https://www.fatalerrors.org/a/orb-slam-2-based-on-depth-camera-realsense-d435i.html
###### Solution: install the rgbd-launch package with the command (make sure to adjust the ROS distro if needed):
```
sudo apt install ros-melodic-rgbd-launch
```
###### link : https://blog.csdn.net/qq_36898914/article/details/88780649
```
roscore
roslaunch realsense2_camera rs_rgbd.launch
rosrun ORB_SLAM2 RGBD Vocabulary/ORBvoc.txt Examples/RGB-D/D435i.ymal
```
$\color[RGB]{206, 231,104}./build_ros.sh$

$\color[RGB]{104, 189,231}stayExamples$ $\color[RGB]{104, 189,231}/ROS/ORB_SLAM2/CMakeLists.txt$ $\color[RGB]{104, 189,231}Add$ $\color[RGB]{104, 189,231}compilation$ $\color[RGB]{104, 189,231}information$ $\color[RGB]{104, 189,231}to$ $\color[RGB]{104, 189,231}file – lboost$ $\color[RGB]{104, 189,231}-$ $\color[RGB]{104, 189,231}systema$

```
set(LIBS 
      ${OpenCV_LIBS} 
      ${EIGEN3_LIBS}
      ${Pangolin_LIBRARIES}
      ${PROJECT_SOURCE_DIR}/../../../Thirdparty/DBoW2/lib/libDBoW2.so
      ${PROJECT_SOURCE_DIR}/../../../Thirdparty/g2o/lib/libg2o.so
      ${PROJECT_SOURCE_DIR}/../../../lib/libORB_SLAM2.so
      -lboost_ Systema
  )
```
$\color[RGB]{104, 189,231}Find$ $\color[RGB]{104, 189,231}ROS$ $\color[RGB]{104, 189,231}package$ $\color[RGB]{104, 189,231}or$ $\color[RGB]{104, 189,231}ROS$

`Test method`

    echo $ROS_ROOT
      /opt/ros/melodic/share/ros
    echo $ROS_PACKAGE_PATH
      /opt/ros/melodic/share:/home/test/slam/ORB_SLAM2-master/Examples/ROS/ORB_SLAM2
      
$\color[RGB]{206, 231,104}Solution$

$\color[RGB]{104, 189,231}User$ $\color[RGB]{104, 189,231}directory$ $\color[RGB]{104, 189,231}in$ $\color[RGB]{104, 189,231}.Bashrc$

###### Add ROS in order_ root path
```
source /opt/ros/melodic/setup.bash
```
###### Add ROS_ package_ path
```
export ROS_PACKAGE_PATH=${ROS_PACKAGE_PATH}:/home/test/slam/ORB_SLAM2-master/Examples/ROS/ORB_SLAM2
```
###### Apply added environment variables
```
source ~/.bashrc
```
## WebCam usb_cam
link : https://www.huaweicloud.com/articles/12586449.html

$\color[RGB]{206, 231,104}Astra$ $\color[RGB]{206, 231,104}fix$

link : https://blog.csdn.net/crp997576280/article/details/104220926

`error while loading shared libraries: libpango_core.so`
```
sudo gedit /etc/ld.so.conf
```
`add` 
```
include /usr/local/lib
```
```
sudo ldconfig
```
