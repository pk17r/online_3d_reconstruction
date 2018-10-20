# Online 3D Reconstruction on a UAV

Program to do online 3D reconstruction with pose correction of UAV. Very high localization and object dimensional accuracy
![alt text](https://github.com/pk17r/pose_estimation/blob/master/info/Slide1.PNG)

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

## Algo:
![alt text](https://github.com/pk17r/pose_estimation/blob/master/info/Slide17.PNG)
![alt text](https://github.com/pk17r/pose_estimation/blob/master/info/Slide18.PNG)
![alt text](https://github.com/pk17r/pose_estimation/blob/master/info/Slide19.PNG)
![alt text](https://github.com/pk17r/pose_estimation/blob/master/info/Without_Correction.png)
![alt text](https://github.com/pk17r/pose_estimation/blob/master/info/With_Correction.png)

## Comparision of Presented 3D Reconstruction results (fitst) with Agisoft PhotoScan results (second)
![alt text](https://github.com/pk17r/pose_estimation/blob/master/info/Presented_3D_Reconstruction.png)
![alt text](https://github.com/pk17r/pose_estimation/blob/master/info/Agisoft_Software.png)
![alt text](https://github.com/pk17r/pose_estimation/blob/master/info/Slide24.PNG)

## Accuracy:
- UAV flying altitude 22m. localization with < 10 cm, object dimension accuracy < 3 cm
![alt text](https://github.com/pk17r/pose_estimation/blob/master/info/Slide26.PNG)


## Appendix:

## Filtering Raw Stereo:
![alt text](https://github.com/pk17r/pose_estimation/blob/master/info/Slide10.PNG)
![alt text](https://github.com/pk17r/pose_estimation/blob/master/info/Slide13.PNG)
![alt text](https://github.com/pk17r/pose_estimation/blob/master/info/Slide14.PNG)
![alt text](https://github.com/pk17r/pose_estimation/blob/master/info/Slide16.PNG)

## Self notes:
pcl 1.6 requires vtk 5.10.1 to work
install using
sudo apt-get install libvtk5.10-qt4 libvtk5.10 libvtk5-dev

## Usage:
./pose 1230 1400 --seq_len 50 --preview --voxel_size 0.05 --jump_pixels 15 --range_width 100 --dist_nearby 6 --min_points_per_voxel 1 --blur_kernel 30 
./pose 1230 1400 --seq_len 50 --preview --voxel_size 0.05 --jump_pixels 15 --range_width 100 --dist_nearby 6 --min_points_per_voxel 1 --blur_kernel 30 --dont_downsample 
