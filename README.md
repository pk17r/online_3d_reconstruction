# pose_estimation


Program to do online 3D reconstruction with pose correction of UAV. Very high localization and object dimensional accuracy


Inputs:

- disparity images

- camera images

- UAV GPS and IMU state estimate pose Quaternion


Current Speeds:

- sparse cloud ~ 22 fps, semi-dense cloud ~ 12 fps

file:///home/pkr/pkr-work/pose_estimation/comparison_with_Agisoft_PhotoScan_results/1.png

Comparision of results (right) with Agisoft PhotoScan results (left)
[[https://github.com/pk17r/pose_estimation/blob/master/comparison_with_Agisoft_PhotoScan_results/1.png|alt=octocat]]
[[https://github.com/pk17r/pose_estimation/blob/master/comparison_with_Agisoft_PhotoScan_results/2.png|alt=octocat]]
[[https://github.com/pk17r/pose_estimation/blob/master/comparison_with_Agisoft_PhotoScan_results/3.png|alt=octocat]]
[[https://github.com/pk17r/pose_estimation/blob/master/comparison_with_Agisoft_PhotoScan_results/4.png|alt=octocat]]
[[https://github.com/pk17r/pose_estimation/blob/master/comparison_with_Agisoft_PhotoScan_results/5.png|alt=octocat]]



Self notes:


pcl 1.6 requires vtk 5.10.1 to work

install using

sudo apt-get install libvtk5.10-qt4 libvtk5.10 libvtk5-dev
