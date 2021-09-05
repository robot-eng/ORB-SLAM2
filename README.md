# ORB-SLAM2
#### ไม่สามารถเอาไฟล์ลงได้เนื่องจากขนาดไฟล์เกินจากที่กำหนด
โหลดไฟล์ตั้งต้นได้จาก https://github.com/raulmur/ORB_SLAM2
`Note : want opencv 3+ (Ubuntu18.04) and cmake 3.8+`
- install Pangolin
###### git clone https://github.com/stevenlovegrove/Pangolin.git
###### cd Pangolin
###### mkdir build
###### cd build
###### cmake ..
###### make
- install CHOLMOD, BLAS, LAPACK and Eigen3.
```
sudo apt-get install libsuitesparse-dev
sudo apt-get install libblas-dev
sudo apt-get install liblapack-dev
sudo apt-get install libeigen3-dev
```
#### Fix file at download come

Error message:

/home/melanie/source/smartcar/orm_slam2/orb_slam2/src/viewer.cc:159:28:error: ' Usleep ' is not declared in this scope
Usleep (3000);
##### Also don't forget to add this line on most files.cc ( in the top )
###### include <unistd.h>

##### File in folder src:
###### examples/monocular/mono_euroc.cc
###### examples/monocular/mono_kitti.cc
###### examples/monocular/mono_tum.cc
###### examples/rgb-d/rgbd_tum.cc
###### examples/stereo/stereo_euroc.cc
###### examples/stereo/stereo_kitti.cc
###### src/localmapping.cc
###### src/loopclosing.cc
###### src/system.cc
###### src/tracking.cc
###### src/viewer.cc
#### Cuda Error :
##### Cuda-related errors occur when a Cuda-installed machine is compiled, and the modified build.sh are as follows:
###### cd Thirdparty/DBow2 , cd Thirdparty/g2o 
###### mkdir Build
###### cd  Build
###### cmake
###### make
##### Error message:
/home/melanie/tools/eigen/eigen/src/core/assignevaluator.h:817:3: Error:static Assertion Failed:you_mixed_ different_numeric_types__you_need_to_use_the_cast_method_of_matrixbase_to_cast_numeric_types_explicitly
Eigen_check_binary_compatibiliy (Func,typename actualdsttypecleaned::scalar,typename Src::Scalar);
^
##### Solution:

Open Thirdparty/g2o/g2o/solvers/linear_solver_eigen.h, and the following code
```
Template <typename matrixtype>
Class Linearsolvereigen:public Linearsolver<matrixtype>
{
Public
typedef eigen::sparsematrix<double, eigen::colmajor> Sparsematrix;
typedef eigen::triplet<double> Triplet;
typedef Eigen::P Ermutationmatrix<eigen::D ynamic, Eigen::D ynamic, sparsematrix::index> Permutationmatrix;
```
Modified to:
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
#### Realsense D435i ROS
###### SDK  : https://github.com/IntelRealSense/librealsense/blob/master/doc/distribution_linux.md#installing-the-packages
###### Node : https://github.com/IntelRealSense/realsense-ros#installation-instructions
#### ORB_SLAM2 + D435i
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
