# ORB-SLAM2
#### ไม่สามารถเอาไฟล์ลงได้เนื่องจากขนาดไฟล์เกินจากที่กำหนด
โหลดไฟล์ตั้งต้นได้จาก https://github.com/raulmur/ORB_SLAM2
`Note : want opencv 3+ (Ubuntu18.04) and cmake 3.8+`
#### Fix file at download come
###### Let me add the name of the file in question :
###### Thirdparty/g2o/g2o/solvers/linear_solver_eigen.h

###### typedef Eigen::PermutationMatrix<Eigen::Dynamic, Eigen::Dynamic, SparseMatrix::Index> PermutationMatrix;
###### to typedef Eigen::PermutationMatrix<Eigen::Dynamic, Eigen::Dynamic, SparseMatrix::StorageIndex> PermutationMatrix;

###### Also don't forget to add this line on most files.cc ( in the top )
###### include <unistd.h>

###### File in folder src:
###### LocalMapping.cc
###### System.cc
###### Tracking.cc
###### Viewer.cc
###### LoopClosing.cc

###### And in folder Examples...
###### stereo_euroc.cc
###### stereo_kitti.cc
###### rgbd_tum.cc
###### mono_kitti.cc
###### mono_tum.cc
###### mono_euro.cc

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
