# pose_estimation


Program to do online 3D reconstruction with pose correction of UAV. Very high localization and object dimensional accuracy


Inputs:

- disparity images

- camera images

- UAV GPS and IMU state estimate pose Quaternion


Current Speeds:

- sparse cloud ~ 22 fps, semi-dense cloud ~ 12 fps



Self notes:


pcl 1.6 requires vtk 5.10.1 to work

install using

sudo apt-get install libvtk5.10-qt4 libvtk5.10 libvtk5-dev
