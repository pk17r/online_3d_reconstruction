# Online 3D Reconstruction on a UAV

Program to do online 3D reconstruction with pose correction of UAV. Very high localization and object dimensional accuracy

## Dependencies:
- OpenCV 3.1
- PCL 1.7
- CUDA 8.0

## Inputs:
- disparity images
- camera images
- UAV GPS and IMU state estimate pose Quaternion

## Speeds:
- sparse cloud ~ 22 fps, semi-dense cloud ~ 12 fps

## Accuracy:
- UAV flying altitude 22m. localization with < 10 cm, object dimension accuracy < 3 cm

## Results:
![alt text](https://github.com/pk17r/pose_estimation/blob/master/comparison_with_Agisoft_PhotoScan_results/6.png)
![alt text](https://github.com/pk17r/pose_estimation/blob/master/comparison_with_Agisoft_PhotoScan_results/7.png)

## Comparision of results (right) with Agisoft PhotoScan results (left)
![alt text](https://github.com/pk17r/pose_estimation/blob/master/comparison_with_Agisoft_PhotoScan_results/1.png)
![alt text](https://github.com/pk17r/pose_estimation/blob/master/comparison_with_Agisoft_PhotoScan_results/2.png)
![alt text](https://github.com/pk17r/pose_estimation/blob/master/comparison_with_Agisoft_PhotoScan_results/3.png)
![alt text](https://github.com/pk17r/pose_estimation/blob/master/comparison_with_Agisoft_PhotoScan_results/4.png)
![alt text](https://github.com/pk17r/pose_estimation/blob/master/comparison_with_Agisoft_PhotoScan_results/5.png)


## Self notes:
pcl 1.6 requires vtk 5.10.1 to work
install using
sudo apt-get install libvtk5.10-qt4 libvtk5.10 libvtk5-dev
