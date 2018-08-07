/* STEPS:
*
* 1. extract 2D features
* 2. find feature correspondence
* 3. convert corresponding features to 3D using disparity image information
* 4. find transformation between corresponding 3D points using estimateAffine3D: Output 3D affine transformation matrix  3 x 4
* 5. use decomposeProjectionMatrix to get rotation or Euler angles
*
*
*
* */

#include "pose.h"

int main(int argc, char* argv[])
{
	cout << "Pose Estimation Program!!" << endl;
	
#if 0
	cv::setBreakOnError(true);
#endif

	int retVal = parseCmdArgs(argc, argv);
	if (retVal == -1) return -1;
	
	readCalibFile();
	readPoseFile();
	readImages();

	int64 app_start_time = getTickCount();
	int64 t = getTickCount();
	
	cout << "Finding features..." << endl;
	findFeatures();
	cout << "Finding features, time: " << ((getTickCount() - t) / getTickFrequency()) << " sec" << endl;
	
	cout << "Pairwise matching" << endl;
	t = getTickCount();
	pairWiseMatching();
	cout << "Pairwise matching, time: " << ((getTickCount() - t) / getTickFrequency()) << " sec" << endl;
	
	if(use_segment_labels)
	{
		cout << "Use_segment_labels to improve disparity images..." << endl;
		createPlaneFittedDisparityImages();
	}
	
	//view 3D point cloud of first image & disparity map
	/*if(preview)
	{
		//int img_index = 0;
		pcl::visualization::PCLVisualizer viewer ("3d reconstruction");
		
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudrgb0 (new pcl::PointCloud<pcl::PointXYZRGB> ());
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudrgb1 (new pcl::PointCloud<pcl::PointXYZRGB> ());
		
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloudxyz0 (new pcl::PointCloud<pcl::PointXYZ> ());
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloudxyz1 (new pcl::PointCloud<pcl::PointXYZ> ());
		
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformed_cloudrgb0( new pcl::PointCloud<pcl::PointXYZRGB>() );
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformed_cloudrgb1( new pcl::PointCloud<pcl::PointXYZRGB>() );
		
		int img_index = 0;
		cout << "pose_seq_to_find " << img_numbers[img_index] << endl;
		int found_index = binary_search_find_index(pose_sequence, img_numbers[img_index]);
		cout << "found " << pose_data[found_index][0] << endl;
		
		Eigen::Affine3f transform_2 = Eigen::Affine3f::Identity();
		// Define a translation of 2.5 meters on the x axis.
		transform_2.translation() << pose_data[found_index][1], pose_data[found_index][2], pose_data[found_index][3];

		// The same rotation matrix as before; theta radians around Z axis
		double theta = 0;
		transform_2.rotate (Eigen::AngleAxisf (theta, Eigen::Vector3f::UnitZ()));
		
		// Print the transformation
		printf ("Correcting 3D pt cloud using an Affine3f\n");
		std::cout << transform_2.matrix() << std::endl;
		
		createPtCloud(0, cloudrgb0, cloudxyz0);
		transformPtCloud(cloudrgb0, transformed_cloudrgb0, transform_2);
		
		img_index = 1;
		cout << "pose_seq_to_find " << img_numbers[img_index] << endl;
		found_index = binary_search_find_index(pose_sequence, img_numbers[img_index]);
		cout << "found " << pose_data[found_index][0] << endl;
		
		transform_2 = Eigen::Affine3f::Identity();
		// Define a translation of 2.5 meters on the x axis.
		transform_2.translation() << pose_data[found_index][1], pose_data[found_index][2], pose_data[found_index][3];

		// The same rotation matrix as before; theta radians around Z axis
		theta = 0;
		transform_2.rotate (Eigen::AngleAxisf (theta, Eigen::Vector3f::UnitZ()));
		
		// Print the transformation
		printf ("Correcting 3D pt cloud using an Affine3f\n");
		std::cout << transform_2.matrix() << std::endl;
		
		createPtCloud(1, cloudrgb1, cloudxyz1);
		transformPtCloud(cloudrgb1, transformed_cloudrgb1, transform_2);
		
		pcl::registration::TransformationEstimationSVD<pcl::PointXYZRGB, pcl::PointXYZRGB> te;
		pcl::registration::TransformationEstimation<pcl::PointXYZRGB, pcl::PointXYZRGB>::Matrix4 T_SVD;
		//Eigen::Matrix4f T_SVD;
		te.estimateRigidTransformation(*cloudrgb0, *cloudrgb1, T_SVD);
		cout << "computed point cloud transformation is\n" << T_SVD << endl;
		const Eigen::Quaternionf   R_SVD (T_SVD.topLeftCorner  <3, 3> ());
		const Eigen::Translation3f t_SVD (T_SVD.topRightCorner <3, 1> ());
		cout << "R_SVD: x " << R_SVD.x() << " y " << R_SVD.y() << " z " << R_SVD.z() << " w " << R_SVD.w() << endl;
		cout << "t_SVD: x " << t_SVD.x() << " y " << t_SVD.y() << " z " << t_SVD.z() << endl;
				
		//cout << "pose_seq_to_find " << img_numbers[0] << endl;
		//int found_index = binary_search_find_index(pose_sequence, img_numbers[0]);
		//cout << "found_index " << found_index << endl;
		//cout << "pose_data[found_index] " << pose_data[found_index][0] << "," << pose_data[found_index][1] << "," << pose_data[found_index][2] << "," << pose_data[found_index][3] << "," << pose_data[found_index][4] << "," << pose_data[found_index][5] << "," << pose_data[found_index][6] << "," << pose_data[found_index][7] << endl;	
		//
		//Eigen::Vector4f sensor_origin = Eigen::Vector4f(pose_data[found_index][1], pose_data[found_index][2], pose_data[found_index][3], 0);
		//// Print the transformation
		//printf ("sensor_origin\n");
		//std::cout << sensor_origin << std::endl;
		//		
		//Eigen::Quaternion<float> sensor_quaternion = Eigen::Quaternion<float> (pose_data[found_index][4], pose_data[found_index][5], pose_data[found_index][6], pose_data[found_index][7]);
		//
		//// Define R,G,B colors for the point cloud
		//pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb0 (cloud0);
		////pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb1 (cloud1);
	    //
		//viewer.addPointCloud<pcl::PointXYZRGB> (cloud0, rgb0, &sensor_origin, &sensor_quaternion, "cloud" + to_string(0), 0);
		////viewer.addPointCloud<pcl::PointXYZRGB> (cloud1, rgb1, "cloud" + to_string(1));
		//
		//viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "transformed_cloud" + to_string(0));
		////viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "transformed_cloud" + to_string(1));
		
		pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb0 (transformed_cloudrgb0);
		pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb1 (transformed_cloudrgb1);
		//pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> transformed_cloud_color_handler0 (transformed_cloud0, 230, 20, 20); // Red
		//pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> transformed_cloud_color_handler1 (transformed_cloud1, 20, 230, 20); // Green
		
		viewer.addPointCloud<pcl::PointXYZRGB> (transformed_cloudrgb0, rgb0, "transformed_cloud" + to_string(0));
		viewer.addPointCloud<pcl::PointXYZRGB> (transformed_cloudrgb1, rgb1, "transformed_cloud" + to_string(1));
		
		viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "transformed_cloud" + to_string(0));
		viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "transformed_cloud" + to_string(1));
		
		cout << "*** Display the visualiser until 'q' key is pressed ***" << endl;
		
		viewer.addCoordinateSystem (1.0, 0, 0, 0);
		viewer.setBackgroundColor(0.05, 0.05, 0.05, 0); // Setting background to a dark grey
		viewer.setPosition(full_images[0].cols/2, full_images[0].rows/2); // Setting visualiser window position
		
		while (!viewer.wasStopped ()) { // Display the visualiser until 'q' key is pressed
			viewer.spinOnce();
		}
	}
	
	*/
	cout << "Converting 2D matches to 3D matches..." << endl;
	t = getTickCount();
	
	//f << "train is dst and query is src" << endl;
	if(log_stuff)
		f << "3D matched points: " << endl;
	
	MatchesInfo match = pairwise_matches[1];
	
	vector<KeyPoint> keypoints_src, keypoints_dst;
	keypoints_src = features[match.src_img_idx].keypoints;
	keypoints_dst = features[match.dst_img_idx].keypoints;
	
	Mat disp_img_src = disparity_images[match.src_img_idx];
	Mat disp_img_dst = disparity_images[match.dst_img_idx];
	
	if(reduction_ratio != 1.0)
	{
		Mat disp_img2_src, disp_img2_dst;
		resize(disp_img_src, disp_img2_src, Size(), reduction_ratio, reduction_ratio, INTER_NEAREST);
		resize(disp_img_dst, disp_img2_dst, Size(), reduction_ratio, reduction_ratio, INTER_NEAREST);
		disp_img_src = disp_img2_src * reduction_ratio;	//increasing the disparity on enlarging disparity image
		disp_img_dst = disp_img2_dst * reduction_ratio;
	}
	
	//define 3d points for all keypoints
	vector<Point3d> keypoints3D_src, keypoints3D_dst;
	vector<int> keypoints3D_2D_index_src, keypoints3D_2D_index_dst;
	
	Mat descriptor_src, descriptor_dst;
	features[match.src_img_idx].descriptors.copyTo(descriptor_src);
	features[match.dst_img_idx].descriptors.copyTo(descriptor_dst);
	
	for (int i = 0; i < match.inliers_mask.size(); i++)
	{
		if (match.inliers_mask[i] == 1 && match.matches[i].imgIdx == match.src_img_idx)
		{
			int trainIdx = match.matches[i].trainIdx;
			int queryIdx = match.matches[i].queryIdx;
			
			//*3. convert corresponding features to 3D using disparity image information
			int disp_val_src = (int)disp_img_src.at<char>(keypoints_src[queryIdx].pt.y, keypoints_src[queryIdx].pt.x);
			int disp_val_dst = (int)disp_img_dst.at<char>(keypoints_dst[trainIdx].pt.y, keypoints_dst[trainIdx].pt.x);
			
			if(log_stuff)
			{
				//f << "match_index " << i << " train_imgIdx " << match.matches[i].imgIdx << " src_img_idx " << match.src_img_idx << " dst_img_idx " << match.dst_img_idx;
				//f << " trainIdx " << trainIdx << " and queryIdx " << queryIdx << " distance " << match.matches[i].distance << endl;
				//f << "descript src query " << descriptor_src.row(queryIdx) << endl;
				//f << "descript dst train " << descriptor_dst.row(trainIdx) << endl;
				//f << "keypoint src query (" << keypoints_src[queryIdx].pt.x << "," << keypoints_src[queryIdx].pt.y;
				//f << ") keypoint dst train (" << keypoints_dst[trainIdx].pt.x << "," << keypoints_dst[trainIdx].pt.y << ")" << endl;
				//f << "disp_val_src " << disp_val_src << " disp_val_dst " << disp_val_dst << endl;
			}
			
			cv::Mat_<double> vec_src(4, 1);
			cv::Mat_<double> vec_dst(4, 1);

			if (disp_val_src > minDisparity && disp_val_dst > minDisparity && keypoints_src[queryIdx].pt.x >= cols_start_aft_cutout && keypoints_dst[trainIdx].pt.x >= cols_start_aft_cutout)
			{
				double xs = keypoints_src[queryIdx].pt.x;
				double ys = keypoints_src[queryIdx].pt.y;
				
				vec_src(0) = xs; vec_src(1) = ys; vec_src(2) = disp_val_src; vec_src(3) = 1;
				vec_src = Q * vec_src;
				vec_src /= vec_src(3);
				
				Point3d src_3D_pt = Point3d(vec_src(0), vec_src(1), vec_src(2));

				double xd = keypoints_dst[trainIdx].pt.x;
				double yd = keypoints_dst[trainIdx].pt.y;

				vec_dst(0) = xd; vec_dst(1) = yd; vec_dst(2) = disp_val_dst; vec_dst(3) = 1;
				vec_dst = Q * vec_dst;
				vec_dst /= vec_dst(3);

				Point3d dst_3D_pt = Point3d(vec_dst(0), vec_dst(1), vec_dst(2));
				
				keypoints3D_src.push_back(src_3D_pt);
				keypoints3D_2D_index_src.push_back(queryIdx);

				keypoints3D_dst.push_back(dst_3D_pt);
				keypoints3D_2D_index_dst.push_back(trainIdx);

				if(log_stuff)
					f << "srcQ3D_point " << src_3D_pt << " dstQ3D_point " << dst_3D_pt << endl;
			}

		}
	}
	
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud0 (new pcl::PointCloud<pcl::PointXYZ> ());
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1 (new pcl::PointCloud<pcl::PointXYZ> ());

	cloud0->width    = 7;
	cloud0->height   = 5;
	cloud0->is_dense = false;
	cloud0->points.resize (cloud0->width * cloud0->height);
	
	cloud1->width    = 7;
	cloud1->height   = 5;
	cloud1->is_dense = false;
	cloud1->points.resize (cloud1->width * cloud1->height);
	
	for (int i = 0; i < keypoints3D_src.size(); ++i)
	{
		pcl::PointXYZ pt_3d_src, pt_3d_dst;
		
		pt_3d_src.x = keypoints3D_src[i].x;
		pt_3d_src.y = keypoints3D_src[i].y;
		pt_3d_src.z = keypoints3D_src[i].z;
		
		pt_3d_dst.x = keypoints3D_dst[i].x;
		pt_3d_dst.y = keypoints3D_dst[i].y;
		pt_3d_dst.z = keypoints3D_dst[i].z;
		
		cloud0->points.push_back(pt_3d_src);
		cloud1->points.push_back(pt_3d_dst);
	}

	cout << "Converting 2D matches to 3D matches, time: " << ((getTickCount() - t) / getTickFrequency()) << " sec" << endl;
	
	cout << "Finding 3D transformation and Eular Angles..." << endl;
	t = getTickCount();
	
	pcl::registration::TransformationEstimationSVD<pcl::PointXYZ, pcl::PointXYZ> te2;
	pcl::registration::TransformationEstimation<pcl::PointXYZ, pcl::PointXYZ>::Matrix4 T_SVD2;
	//Eigen::Matrix4f T_SVD;
	te2.estimateRigidTransformation(*cloud0, *cloud1, T_SVD2);
	cout << "computed transformation between MATCHED KEYPOINTS T_SVD2 is\n" << T_SVD2 << endl;
	const Eigen::Quaternionf   R_SVD2 (T_SVD2.topLeftCorner  <3, 3> ());
	const Eigen::Translation3f t_SVD2 (T_SVD2.topRightCorner <3, 1> ());
	cout << "R_SVD2: x " << R_SVD2.x() << " y " << R_SVD2.y() << " z " << R_SVD2.z() << " w " << R_SVD2.w() << endl;
	cout << "t_SVD2: x " << t_SVD2.x() << " y " << t_SVD2.y() << " z " << t_SVD2.z() << endl;
	
	//Mat rotations = T_SVD2.topLeftCorner  <3, 3> ();
	Mat rotations = Mat::zeros(cv::Size(3, 3), CV_64FC1);
	rotations.at<double>(0,0) = T_SVD2(0,0);
	rotations.at<double>(0,1) = T_SVD2(0,1);
	rotations.at<double>(0,2) = T_SVD2(0,2);
	rotations.at<double>(1,0) = T_SVD2(1,0);
	rotations.at<double>(1,1) = T_SVD2(1,1);
	rotations.at<double>(1,2) = T_SVD2(1,2);
	rotations.at<double>(2,0) = T_SVD2(2,0);
	rotations.at<double>(2,1) = T_SVD2(2,1);
	rotations.at<double>(2,2) = T_SVD2(2,2);
	Mat mtxR, mtxQ, transVect, rotMatrixX, rotMatrixY, rotMatrixZ;
	//decomposeProjectionMatrix(affineTransformationMatrix, cameraMatrix, rotMatrix, transVect, rotMatrixX, rotMatrixY, rotMatrixZ, eulerAngles);
	RQDecomp3x3(rotations, mtxR, mtxQ, rotMatrixX, rotMatrixY, rotMatrixZ);
	
	//view 3D point cloud of first image & disparity map
	if(preview)
	{
		//int img_index = 0;
		pcl::visualization::PCLVisualizer viewer ("3d reconstruction");
		
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudrgb0 (new pcl::PointCloud<pcl::PointXYZRGB> ());
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudrgb1 (new pcl::PointCloud<pcl::PointXYZRGB> ());
		
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloudxyz0 (new pcl::PointCloud<pcl::PointXYZ> ());
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloudxyz1 (new pcl::PointCloud<pcl::PointXYZ> ());
		
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformed_cloudrgb0( new pcl::PointCloud<pcl::PointXYZRGB>() );
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformed_cloudrgb1( new pcl::PointCloud<pcl::PointXYZRGB>() );
		
		int found_index0 = binary_search_find_index(pose_sequence, img_numbers[0]);
		//cout << "found_index0 " << found_index0 << endl;
		cout << "pose_data of image " << img_numbers[0] << " is " << pose_data[found_index0][0] << " pos_x " << pose_data[found_index0][1] << " pos_y " << pose_data[found_index0][2] << " pos_z " << pose_data[found_index0][3] << " quat_x " << pose_data[found_index0][4] << " quat_y " << pose_data[found_index0][5] << " quat_z " << pose_data[found_index0][6] << " quat_w " << pose_data[found_index0][7] << endl;
		cout << "sq sum of quaternion " << sqrt(pose_data[found_index0][4]*pose_data[found_index0][4] + pose_data[found_index0][5]*pose_data[found_index0][5] + pose_data[found_index0][6]*pose_data[found_index0][6] + pose_data[found_index0][7]*pose_data[found_index0][7]) << endl;
		
		createPtCloud(0, cloudrgb0, cloudxyz0);
		pcl::registration::TransformationEstimation<pcl::PointXYZRGB, pcl::PointXYZRGB>::Matrix4 T_SVD0 = generateTmat(pose_data[found_index0]);
		transformPtCloud2(cloudrgb0, transformed_cloudrgb0, T_SVD2);
		
		int found_index1 = binary_search_find_index(pose_sequence, img_numbers[1]);
		//cout << "found_index1 " << found_index1 << endl;
		cout << "pose_data of image " << img_numbers[1] << " is " << pose_data[found_index1][0] << " pos_x " << pose_data[found_index1][1] << " pos_y " << pose_data[found_index1][2] << " pos_z " << pose_data[found_index1][3] << " quat_x " << pose_data[found_index1][4] << " quat_y " << pose_data[found_index1][5] << " quat_z " << pose_data[found_index1][6] << " quat_w " << pose_data[found_index1][7] << endl;
		cout << "sq sum of quaternion " << sqrt(pose_data[found_index1][4]*pose_data[found_index1][4] + pose_data[found_index1][5]*pose_data[found_index1][5] + pose_data[found_index1][6]*pose_data[found_index1][6] + pose_data[found_index1][7]*pose_data[found_index1][7]) << endl;
		
		createPtCloud(1, cloudrgb1, cloudxyz1);
		pcl::registration::TransformationEstimation<pcl::PointXYZRGB, pcl::PointXYZRGB>::Matrix4 T_SVD1 = generateTmat(pose_data[found_index1]);
		transformPtCloud2(cloudrgb1, transformed_cloudrgb1, T_SVD1);
		
		pcl::registration::TransformationEstimationSVD<pcl::PointXYZRGB, pcl::PointXYZRGB> te;
		pcl::registration::TransformationEstimation<pcl::PointXYZRGB, pcl::PointXYZRGB>::Matrix4 T_SVD;
		//Eigen::Matrix4f T_SVD;
		te.estimateRigidTransformation(*transformed_cloudrgb0, *transformed_cloudrgb1, T_SVD);
		cout << "computed transformation between COMPLETE POINT CLOUDS T_SVD is\n" << T_SVD << endl;
		const Eigen::Quaternionf   R_SVD (T_SVD.topLeftCorner  <3, 3> ());
		const Eigen::Translation3f t_SVD (T_SVD.topRightCorner <3, 1> ());
		cout << "R_SVD: x " << R_SVD.x() << " y " << R_SVD.y() << " z " << R_SVD.z() << " w " << R_SVD.w() << endl;
		cout << "t_SVD: x " << t_SVD.x() << " y " << t_SVD.y() << " z " << t_SVD.z() << endl;
		
		double trans_x = pose_data[found_index1][1] - pose_data[found_index0][1];
		double trans_y = pose_data[found_index1][2] - pose_data[found_index0][2];
		double trans_z = pose_data[found_index1][3] - pose_data[found_index0][3];
		
		cout << "recorded translation between images is   trans_x " << trans_x <<    " trans_y " << trans_y <<    " trans_z " << trans_z << endl;
		cout << "calculated translation between images is t_SVDx " << t_SVD.x() << " t_SVD2y " << t_SVD.y() << " t_SVD2z " << t_SVD.z() << endl;
		cout << "difference between recorded and calculated translation between images is x " << trans_x - t_SVD.x() << " y " << trans_y - t_SVD.y() << " z " << trans_z - t_SVD.z	() << endl;
		
		//Eigen::Vector4f sensor_origin = Eigen::Vector4f(pose_data[found_index][1], pose_data[found_index][2], pose_data[found_index][3], 0);
		//// Print the transformation
		//printf ("sensor_origin\n");
		//std::cout << sensor_origin << std::endl;
		//		
		//Eigen::Quaternion<float> sensor_quaternion = Eigen::Quaternion<float> (pose_data[found_index][4], pose_data[found_index][5], pose_data[found_index][6], pose_data[found_index][7]);
		//
		//// Define R,G,B colors for the point cloud
		//pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb0 (cloud0);
		////pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb1 (cloud1);
	    //
		//viewer.addPointCloud<pcl::PointXYZRGB> (cloud0, rgb0, &sensor_origin, &sensor_quaternion, "cloud" + to_string(0), 0);
		////viewer.addPointCloud<pcl::PointXYZRGB> (cloud1, rgb1, "cloud" + to_string(1));
		//
		//viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "transformed_cloud" + to_string(0));
		////viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "transformed_cloud" + to_string(1));
		
		//pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb0 (cloudrgb0);
		//pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb1 (cloudrgb1);
		pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> trans_rgb0 (transformed_cloudrgb0);
		pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> trans_rgb1 (transformed_cloudrgb1);
		//pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> transformed_cloud_color_handler0 (transformed_cloudrgb0, 230, 20, 20); // Red
		//pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> transformed_cloud_color_handler1 (transformed_cloud1, 20, 230, 20); // Green
		
		//viewer.addPointCloud<pcl::PointXYZRGB> (cloudrgb0, rgb0, "cloud" + to_string(0));
		//viewer.addPointCloud<pcl::PointXYZRGB> (cloudrgb1, rgb1, "cloud" + to_string(1));
		viewer.addPointCloud<pcl::PointXYZRGB> (transformed_cloudrgb0, trans_rgb0, "transformed_cloud" + to_string(0));
		viewer.addPointCloud<pcl::PointXYZRGB> (transformed_cloudrgb1, trans_rgb1, "transformed_cloud" + to_string(1));
		
		//viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "cloud" + to_string(0));
		//viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "cloud" + to_string(1));
		viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "transformed_cloud" + to_string(0));
		viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "transformed_cloud" + to_string(1));
		
		
		cout << "*** Display the visualiser until 'q' key is pressed ***" << endl;
		
		viewer.addCoordinateSystem (1.0, 0, 0, 0);
		viewer.setBackgroundColor(0.05, 0.05, 0.05, 0); // Setting background to a dark grey
		viewer.setPosition(full_images[0].cols/2, full_images[0].rows/2); // Setting visualiser window position
		
		while (!viewer.wasStopped ()) { // Display the visualiser until 'q' key is pressed
			viewer.spinOnce();
		}
	}
	
	//*4. find transformation between corresponding 3D points using estimateAffine3D: Output 3D affine transformation matrix  3 x 4
	if(log_stuff)
		f << "Q:\n" << Q << endl;

	//Mat affineTransformationMatrix;
	//std::vector<uchar> inliers3D;
	//double ransacThreshold = 3, confidence = 0.99;
	//estimateAffine3D(keypoints3D_src, keypoints3D_dst, affineTransformationMatrix, inliers3D, ransacThreshold, confidence);
	//cout << "affineTransformationMatrix:\n" << affineTransformationMatrix << endl;
	//
	//int inlierCount = 0;
	//for (int i = 0; i < inliers3D.size(); i++)
	//{
	//	if (inliers3D[i] == 1)
	//		inlierCount++;
	//	f << (int)inliers3D[i] << ", ";
	//}
	//cout << "inlierCount: " << inlierCount << "/" << inliers3D.size() << endl;
	
	//* 5. use decomposeProjectionMatrix to get rotation or Euler angles
	//Mat cameraMatrix, rotMatrix, transVect, rotMatrixX, rotMatrixY, rotMatrixZ, eulerAngles;
	//decomposeProjectionMatrix(affineTransformationMatrix, cameraMatrix, rotMatrix, transVect, rotMatrixX, rotMatrixY, rotMatrixZ, eulerAngles);
	//cout << "EulerAngles :\n" << eulerAngles << endl;
	
	cout << "Finding 3D transformation and Eular Angles, time: " << ((getTickCount() - t) / getTickFrequency()) << " sec" << endl;
	cout << "Finished Pose Estimation, total time: " << ((getTickCount() - app_start_time) / getTickFrequency()) << " sec" << endl;
	
	if(log_stuff)
	{
		//f << "affineTransformationMatrix:\n" << affineTransformationMatrix << endl;
		//f << "ransacThreshold: " << ransacThreshold << " confidence: " << confidence << endl;
		//f << "inliers3D:\n[";
		//f << "]" << endl;
		//f << "inlierCount: " << inlierCount << "/" << inliers3D.size() << endl;
		//f << "cameraMatrix:\n" << cameraMatrix << "\nrotMatrix:\n" << rotMatrix << "\ntransVect :\n" << transVect << "\nrotMatrixX :\n" << rotMatrixX << "\nrotMatrixY :\n" << rotMatrixY << "\nrotMatrixZ :\n" << rotMatrixZ << "\nEulerAngles :\n" << eulerAngles << endl;
		f << "computed point cloud transformation is\n" << T_SVD2 << endl;
		f << "Decomposition:\nrotations:\n" << rotations << "\nmtxR:\n" << mtxR << "\nmtxQ :\n" << mtxQ << "\nrotMatrixX :\n" << rotMatrixX << "\nrotMatrixY :\n" << rotMatrixY << "\nrotMatrixZ :\n" << rotMatrixZ << endl;
	}
	
	cout << "Decomposition:\nrotations:\n" << rotations << "\nmtxR:\n" << mtxR << "\nmtxQ :\n" << mtxQ << "\nrotMatrixX :\n" << rotMatrixX << "\nrotMatrixY :\n" << rotMatrixY << "\nrotMatrixZ :\n" << rotMatrixZ << endl;
	
	cout << "Drawing and saving feature matches images..." << endl;
	Mat matchedImage, matchedImageInliers;
	drawMatches(full_images[0], keypoints_src, full_images[1], keypoints_dst, match.matches, matchedImage);
	vector<char> inliers(match.inliers_mask.size());
	for (int i = 0; i < match.inliers_mask.size(); i++)
	{
		inliers[i] = match.inliers_mask[i];
	}
	drawMatches(full_images[0], keypoints_src, full_images[1], keypoints_dst, match.matches, matchedImageInliers, Scalar::all(-1), Scalar::all(-1), inliers, DrawMatchesFlags::DEFAULT);
	imwrite("output/matchedImage.jpg", matchedImage);
	imwrite("output/matchedImageInliers.jpg", matchedImageInliers);
	
	//do 
	//{
	//	cout << '\n' << "Press a key to continue...";
	//} while (cin.get() != '\n');

	
	return 0;
}

static void printUsage()
{
	cout <<
		"Rotation model images stitcher.\n\n"
		"pose img1 img2 [...imgN] [flags]\n\n"
		"Flags:\n"
		"  --preview\n"
		"      Run stitching in the preview mode. Works faster than usual mode,\n"
		"      but output image will have lower resolution.\n"
		"  --try_cuda (yes|no)\n"
		"      Try to use CUDA. The default value is 'yes'.\n"
		"\nMotion Estimation Flags:\n"
		"  --work_megapix <float>\n"
		"      Resolution for image registration step. The default is 1.2288 Mpx.\n"
		"  --features (surf|orb)\n"
		"      Type of features used for images matching. The default is orb.\n"
		"  --match_conf <float>\n"
		"      Confidence for feature matching step. The default is 0.65 for surf and 0.3 for orb.\n"
		"  --conf_thresh <float>\n"
		"      Threshold for two images are from the same panorama confidence.\n"
		"      The default is 1.0.\n"
		"  --ba (reproj|ray)\n"
		"      Bundle adjustment cost function. The default is ray.\n"
		"  --ba_refine_mask (mask)\n"
		"      Set refinement mask for bundle adjustment. It looks like 'x_xxx',\n"
		"      where 'x' means refine respective parameter and '_' means don't\n"
		"      refine one, and has the following format:\n"
		"      <fx><skew><ppx><aspect><ppy>. The default mask is 'xxxxx'. If bundle\n"
		"      adjustment doesn't support estimation of selected parameter then\n"
		"      the respective flag is ignored.\n"
		"  --wave_correct (no|horiz|vert)\n"
		"      Perform wave effect correction. The default is 'vert'.\n"
		"  --save_graph <file_name>\n"
		"      Save matches graph represented in DOT language to <file_name> file.\n"
		"      Labels description: Nm is number of matches, Ni is number of inliers,\n"
		"      C is confidence.\n"
		"\nCompositing Flags:\n"
		"  --warp (plane|cylindrical|spherical|fisheye|stereographic|compressedPlaneA2B1|compressedPlaneA1.5B1|compressedPlanePortraitA2B1|compressedPlanePortraitA1.5B1|paniniA2B1|paniniA1.5B1|paniniPortraitA2B1|paniniPortraitA1.5B1|mercator|transverseMercator)\n"
		"      Warp surface type. The default is 'spherical'.\n"
		"  --seam_megapix <float>\n"
		"      Resolution for seam estimation step. The default is 0.1 Mpx.\n"
		"  --seam (no|voronoi|gc_color|gc_colorgrad)\n"
		"      Seam estimation method. The default is 'voronoi'.\n"
		"  --compose_megapix <float>\n"
		"      Resolution for compositing step. Use -1 for original resolution.\n"
		"      The default is -1.\n"
		"  --expos_comp (no|gain|gain_blocks)\n"
		"      Exposure compensation method. The default is 'gain_blocks'.\n"
		"  --blend (no|feather|multiband)\n"
		"      Blending method. The default is 'multiband'.\n"
		"  --blend_strength <float>\n"
		"      Blending strength from [0,100] range. The default is 5.\n"
		"  --output <result_img>\n"
		"      The default is 'result.jpg'.\n"
		"  --timelapse (as_is|crop) \n"
		"      Output warped images separately as frames of a time lapse movie, with 'fixed_' prepended to input file names.\n"
		"  --rangewidth <int>\n"
		"      uses range_width to limit number of images to match with.\n";
}

string type2str(int type)
{
  string r;

  uchar depth = type & CV_MAT_DEPTH_MASK;
  uchar chans = 1 + (type >> CV_CN_SHIFT);

  switch ( depth ) {
    case CV_8U:  r = "8U"; break;
    case CV_8S:  r = "8S"; break;
    case CV_16U: r = "16U"; break;
    case CV_16S: r = "16S"; break;
    case CV_32S: r = "32S"; break;
    case CV_32F: r = "32F"; break;
    case CV_64F: r = "64F"; break;
    default:     r = "User"; break;
  }

  r += "C";
  r += (chans+'0');

  return r;
}

static int parseCmdArgs(int argc, char** argv)
{
	//if (argc == 1)
	//{
	//	printUsage();
	//	return -1;
	//}
	int n_imgs = 0;
	for (int i = 1; i < argc; ++i)
	{
		if (string(argv[i]) == "--help" || string(argv[i]) == "/?")
		{
			printUsage();
			return -1;
		}
		else if (string(argv[i]) == "--log")
		{
			cout << "log" << endl;
			log_stuff = true;
		}
		else if (string(argv[i]) == "--preview")
		{
			preview = true;
		}
		else if (string(argv[i]) == "--use_segment_labels")
		{
			cout << "use_segment_labels" << endl;
			use_segment_labels = true;
		}
		else if (string(argv[i]) == "--try_cuda")
		{
			if (string(argv[i + 1]) == "no")
				try_cuda = false;
			else if (string(argv[i + 1]) == "yes")
				try_cuda = true;
			else
			{
				cout << "Bad --try_cuda flag value\n";
				return -1;
			}
			i++;
		}
		else if (string(argv[i]) == "--work_megapix")
		{
			work_megapix = atof(argv[i + 1]);
			i++;
		}
		else if (string(argv[i]) == "--boundingBox")
		{
			boundingBox = atoi(argv[i + 1]);
			i++;
		}
		else if (string(argv[i]) == "--seam_megapix")
		{
			seam_megapix = atof(argv[i + 1]);
			i++;
		}
		else if (string(argv[i]) == "--compose_megapix")
		{
			compose_megapix = atof(argv[i + 1]);
			i++;
		}
		else if (string(argv[i]) == "--result")
		{
			result_name = argv[i + 1];
			i++;
		}
		else if (string(argv[i]) == "--features")
		{
			features_type = argv[i + 1];
			if (features_type == "orb")
				match_conf = 0.3f;
			i++;
		}
		else if (string(argv[i]) == "--match_conf")
		{
			match_conf = static_cast<float>(atof(argv[i + 1]));
			i++;
		}
		else if (string(argv[i]) == "--conf_thresh")
		{
			conf_thresh = static_cast<float>(atof(argv[i + 1]));
			i++;
		}
		else if (string(argv[i]) == "--ba")
		{
			ba_cost_func = argv[i + 1];
			i++;
		}
		else if (string(argv[i]) == "--ba_refine_mask")
		{
			ba_refine_mask = argv[i + 1];
			if (ba_refine_mask.size() != 5)
			{
				cout << "Incorrect refinement mask length.\n";
				return -1;
			}
			i++;
		}
		else if (string(argv[i]) == "--wave_correct")
		{
			if (string(argv[i + 1]) == "no")
				do_wave_correct = false;
			else if (string(argv[i + 1]) == "horiz")
			{
				do_wave_correct = true;
				wave_correct = detail::WAVE_CORRECT_HORIZ;
			}
			else if (string(argv[i + 1]) == "vert")
			{
				do_wave_correct = true;
				wave_correct = detail::WAVE_CORRECT_VERT;
			}
			else
			{
				cout << "Bad --wave_correct flag value\n";
				return -1;
			}
			i++;
		}
		else if (string(argv[i]) == "--save_graph")
		{
			save_graph = true;
			save_graph_to = argv[i + 1];
			i++;
		}
		else if (string(argv[i]) == "--warp")
		{
			warp_type = string(argv[i + 1]);
			i++;
		}
		else if (string(argv[i]) == "--expos_comp")
		{
			if (string(argv[i + 1]) == "no")
				expos_comp_type = ExposureCompensator::NO;
			else if (string(argv[i + 1]) == "gain")
				expos_comp_type = ExposureCompensator::GAIN;
			else if (string(argv[i + 1]) == "gain_blocks")
				expos_comp_type = ExposureCompensator::GAIN_BLOCKS;
			else
			{
				cout << "Bad exposure compensation method\n";
				return -1;
			}
			i++;
		}
		else if (string(argv[i]) == "--seam")
		{
			if (string(argv[i + 1]) == "no" ||
				string(argv[i + 1]) == "voronoi" ||
				string(argv[i + 1]) == "gc_color" ||
				string(argv[i + 1]) == "gc_colorgrad" ||
				string(argv[i + 1]) == "dp_color" ||
				string(argv[i + 1]) == "dp_colorgrad")
				seam_find_type = argv[i + 1];
			else
			{
				cout << "Bad seam finding method\n";
				return -1;
			}
			i++;
		}
		else if (string(argv[i]) == "--blend")
		{
			if (string(argv[i + 1]) == "no")
				blend_type = Blender::NO;
			else if (string(argv[i + 1]) == "feather")
				blend_type = Blender::FEATHER;
			else if (string(argv[i + 1]) == "multiband")
				blend_type = Blender::MULTI_BAND;
			else
			{
				cout << "Bad blending method\n";
				return -1;
			}
			i++;
		}
		else if (string(argv[i]) == "--timelapse")
		{
			timelapse = true;

			if (string(argv[i + 1]) == "as_is")
				timelapse_type = Timelapser::AS_IS;
			else if (string(argv[i + 1]) == "crop")
				timelapse_type = Timelapser::CROP;
			else
			{
				cout << "Bad timelapse method\n";
				return -1;
			}
			i++;
		}
		else if (string(argv[i]) == "--rangewidth")
		{
			range_width = atoi(argv[i + 1]);
			i++;
		}
		else if (string(argv[i]) == "--blend_strength")
		{
			blend_strength = static_cast<float>(atof(argv[i + 1]));
			i++;
		}
		else if (string(argv[i]) == "--output")
		{
			result_name = argv[i + 1];
			i++;
		}
		else
		{
			if (n_imgs == 0)
			{
				img_numbers.clear();
			}
			img_numbers.push_back(atoi(argv[i]));
			cout << atoi(argv[i]) << endl;
			++n_imgs;
		}
	}
	if (preview)
	{
		compose_megapix = 0.6;
	}
	if (img_numbers.size() < 2)
	{
		cout << "Number of images needs to be greater than two!" << endl;
		return -1;
	}
	return 0;
}

//http://www.cplusplus.com/forum/general/17771/
//-----------------------------------------------------------------------------
// Let's overload the stream input operator to read a list of CSV fields (which a CSV record).
// Remember, a record is a list of doubles separated by commas ','.
istream& operator >> ( istream& ins, record_t& record )
  {
  // make sure that the returned record contains only the stuff we read now
  record.clear();

  // read the entire line into a string (a CSV record is terminated by a newline)
  string line;
  getline( ins, line );

  // now we'll use a stringstream to separate the fields out of the line
  stringstream ss( line );
  string field;
  while (getline( ss, field, ',' ))
    {
    // for each field we wish to convert it to a double
    // (since we require that the CSV contains nothing but floating-point values)
    stringstream fs( field );
    double f = 0.0;  // (default value is 0.0)
    fs >> f;

    // add the newly-converted field to the end of the record
    record.push_back( f );
    }

  // Now we have read a single line, converted into a list of fields, converted the fields
  // from strings to doubles, and stored the results in the argument record, so
  // we just return the argument stream as required for this kind of input overload function.
  return ins;
  }

//-----------------------------------------------------------------------------
// Let's likewise overload the stream input operator to read a list of CSV records.
// This time it is a little easier, just because we only need to worry about reading
// records, and not fields.
istream& operator >> ( istream& ins, data_t& data )
  {
  // make sure that the returned data only contains the CSV data we read here
  data.clear();

  // For every record we can read from the file, append it to our resulting data
  record_t record;
  while (ins >> record)
    {
    data.push_back( record );
    }

  // Again, return the argument stream as required for this kind of input stream overload.
  return ins;  
  }

inline int binary_search_find_index(std::vector<int> v, int data)
{
	int difference = -293;
	data -= difference;
    auto it = std::lower_bound(v.begin(), v.end(), data);
    if (it == v.end() || *it != data) {
        return -1;
    } else {
        std::size_t index = std::distance(v.begin(), it);
        return index;
    }   
}

void readCalibFile()
{
	std::ifstream infile(calib_file);
	cv::FileStorage fs(calib_file, cv::FileStorage::READ);
	fs["Q"] >> Q;
	fs.release();
	cout << "read calib file." << endl;
}

void readPoseFile()
{
	//read pose file
	// Here is the file containing the data. Read it into data.
	ifstream psfile(pose_file);
	psfile >> pose_data;
	// Complain if something went wrong.
	if (!psfile.eof())
	{
		cout << "***** Could not open pose file! *****\n";
	}
	psfile.close();
	
	for (int i = 0; i < pose_data.size(); i++)
	{
		pose_sequence.push_back(int(pose_data[i][0]));
	}
	
	// Otherwise, list some basic information about the file.
	cout << "Your CSV file contains " << pose_data.size() << " records.\n";
	cout << pose_sequence[2] << " : ";
	cout << pose_data[2][0] << "," << pose_data[2][1] << "," << pose_data[2][2] << "," << pose_data[2][3] << "," << pose_data[2][4] << "," << pose_data[2][5] << endl;
	cout << "finding " << 1050 << " and found " << pose_data[binary_search_find_index(pose_sequence,1050)][0] << endl;
}

inline void readImages()
{
	images = vector<Mat>(img_numbers.size());
	full_images = vector<Mat>(img_numbers.size());
	disparity_images = vector<Mat>(img_numbers.size());
	full_img_sizes = vector<Size>(img_numbers.size());
	if(use_segment_labels)
	{
		segment_maps = vector<Mat>(img_numbers.size());
		//double_disparity_images = vector<Mat>(img_numbers.size());
	}
	save_graph_to = "output/graph";
	//logging stuff
	if(log_stuff)
		f.open(save_graph_to.c_str(), ios::out);
	
	for (int i = 0; i < img_numbers.size(); ++i)
	{
		cout << img_numbers[i] << endl;
		full_img = imread(imagePrefix + to_string(img_numbers[i]) + ".png");
		if (i == 0)
		{
			work_megapix = 1.0 * full_img.rows * full_img.cols / 1000000;
			cout << "work_megapix: " << work_megapix << endl;
			rows = full_img.rows;
			cols = full_img.cols;
			cols_start_aft_cutout = (int)(cols/cutout_ratio);
		}
		full_images[i] = full_img;
		full_img_sizes[i] = full_img.size();
		
		//read labelled segment maps
		if(use_segment_labels)
		{
			//Mat segment_img(rows,cols, CV_16UC1);
			//segment_img = imread(segmentlblPrefix + to_string(img_numbers[i]) + ".png",CV_LOAD_IMAGE_ANYDEPTH);
			//segment_maps[i] = segment_img;
			
			//read segmentation map
			Mat segment_img(rows,cols, CV_8UC1);
			segment_img = imread(segmentlblPrefix + to_string(img_numbers[i]) + ".png",CV_LOAD_IMAGE_GRAYSCALE);
			segment_maps[i] = segment_img;
			
			//read disparity image
			Mat disp_img(rows,cols, CV_8UC1);
			disp_img = imread(disparityPrefix + to_string(img_numbers[i]) + ".png",CV_LOAD_IMAGE_GRAYSCALE);
			disparity_images[i] = disp_img;
		}
		else
		{
			//read disparity image
			Mat disp_img(rows,cols, CV_8UC1);
			disp_img = imread(disparityPrefix + to_string(img_numbers[i]) + ".png",CV_LOAD_IMAGE_GRAYSCALE);
			disparity_images[i] = disp_img;
		}
		
		
		if (full_img.empty())
		{
			cout << "Can't open image " << img_numbers[i] << endl;
			//return -1;
		}
		if (work_megapix < 0)
		{
			img = full_img;
			work_scale = 1;
			is_work_scale_set = true;
		}
		else
		{
			if (!is_work_scale_set)
			{
				work_scale = min(1.0, sqrt(work_megapix * 1e6 / full_img.size().area()));
				is_work_scale_set = true;
			}
			resize(full_img, img, Size(), work_scale, work_scale);
		}
		if (!is_seam_scale_set)
		{
			seam_scale = min(1.0, sqrt(seam_megapix * 1e6 / full_img.size().area()));
			seam_work_aspect = seam_scale / work_scale;
			cout << "seam_scale: " << seam_scale << "\nseam_work_aspect: " << seam_work_aspect << endl;
			is_seam_scale_set = true;
		}

		resize(full_img, img, Size(), seam_scale, seam_scale);
		images[i] = img.clone();
		
		save_graph_to = save_graph_to + "_" + to_string(img_numbers[i]);
	}
	
	full_img.release();
	img.release();
	save_graph_to = save_graph_to + ".txt";
	reduction_ratio = 1.0 * full_images[0].rows / disparity_images[0].rows;
}

inline void findFeatures()
{
	Ptr<FeaturesFinder> finder;
	if (features_type == "surf")
	{
#ifdef HAVE_OPENCV_XFEATURES2D
		if (try_cuda && cuda::getCudaEnabledDeviceCount() > 0)
			finder = makePtr<SurfFeaturesFinderGpu>();
		else
#endif
			finder = makePtr<SurfFeaturesFinder>();
	}
	else if (features_type == "orb")
	{
		finder = makePtr<OrbFeaturesFinder>();
	}
	else if (features_type == "fast")
	{
		finder = makePtr<OrbFeaturesFinder>();
	}
	else
	{
		cout << "Unknown 2D features type: '" << features_type << "'.\n";
		//return -1;
	}

	//Ptr<FastFeatureDetector> detector = FastFeatureDetector::create();
	features = vector<ImageFeatures>(img_numbers.size());
	
	for (int i = 0; i < img_numbers.size(); ++i)
	{
		if (work_scale != 1)
		{
			full_img = full_images[i];
			resize(full_img, img, Size(), work_scale, work_scale);
		}
		else
		{
			img = full_images[i];
		}

		(*finder)(img, features[i]);
		//vector<KeyPoint> keypointsD;
		//detector->detect(img,keypointsD,Mat());
		features[i].img_idx = i;
		//features[i].keypoints = keypointsD;
		cout << "Features in image #" << i + 1 << ": " << features[i].keypoints.size() << endl;
	}
	
	finder->collectGarbage();
	//free(detector);
	img.release();
}

inline void pairWiseMatching()
{
	if (range_width == -1)
	{
		//cv::detail::AffineBestOf2NearestMatcher::AffineBestOf2NearestMatcher	(	bool 	full_affine = false, bool 	try_use_gpu = false, float 	match_conf = 0.3f, int 	num_matches_thresh1 = 6 )
		//AffineBestOf2NearestMatcher matcher(true, try_cuda, match_conf,1);
		BestOf2NearestMatcher matcher(try_cuda, match_conf);
		matcher(features, pairwise_matches);
		matcher.collectGarbage();
	}
	else
	{
		BestOf2NearestRangeMatcher matcher(range_width, try_cuda, match_conf);
		matcher(features, pairwise_matches);
		matcher.collectGarbage();
	}
	
	if(log_stuff)
	{
		cout << "Baseline (in m) " << baseline << " focallength (in pixel) " << focallength << endl;
		//cout << matchesGraphAsString(img_numbers, pairwise_matches, conf_thresh) << endl;
		cout << "H between images 0 and 1:" << endl;
		cout << pairwise_matches[1].H << endl;
		cout << "H between images 1 and 0:" << endl;
		cout << pairwise_matches[2].H << endl;
		
		f << "Baseline (in m) " << baseline << " focallength (in pixel) " << focallength << endl;
		//f << matchesGraphAsString(img_numbers, pairwise_matches, conf_thresh) << endl;
		f << "H between images 0 and 1:" << endl;
		f << pairwise_matches[1].H << endl;
		f << "H between images 1 and 0:" << endl;
		f << pairwise_matches[2].H << endl;
	}
	
}

void createPlaneFittedDisparityImages()
{
	cout << "boundingBox " << boundingBox << " cols_start_aft_cutout " << cols_start_aft_cutout << endl;
	for (int i = 0; i < segment_maps.size(); i++)
	{
		cout << "Image" << i << endl;
		Mat segment_img = segment_maps[i];
		cout << "segment_img.rows " << segment_img.rows << " segment_img.cols " << segment_img.cols << endl;
		
		Mat disp_img = disparity_images[i];
		
		Mat new_disp_img = Mat::zeros(disp_img.rows,disp_img.cols, CV_64F);
		
		for (int cluster = 1; cluster < 256; cluster++)
		{
			//find pixels in this segment
			vector<int> Xc, Yc;
			for (int l = 0; l < segment_img.rows; l++)
			{
				for (int k = 0; k < segment_img.cols; k++)
				{
					if(segment_img.at<uchar>(l,k) == cluster)
					{
						Xc.push_back(k);
						Yc.push_back(l);
					}
				}
			}
			cout << "cluster" << cluster << " size:" << Xc.size();
			if(Xc.size() == 0)		//all labels covered!
				break;
			
			
			vector<double> Zp, Xp, Yp;
			for (int p = 0; p < Xc.size(); p++)
			{
				if (Xc[p] > cols_start_aft_cutout && Xc[p] < segment_img.cols - boundingBox && Yc[p] > boundingBox && Yc[p] < segment_img.rows - boundingBox)
				{
					//cout << "disp_img.at<uchar>(Yc[p],Xc[p]): " << disp_img.at<uchar>(Yc[p],Xc[p]) << endl;
					Zp.push_back((double)disp_img.at<uchar>(Yc[p],Xc[p]));
					Xp.push_back((double)Xc[p]);
					Yp.push_back((double)Yc[p]);
				}
			}
			//cout << "read all cluster disparities..." << endl;
			cout << " Accepted points: " << Xp.size() << endl;
			if(Xp.size() == 0)		//all labels covered!
				continue;
			
			//define A matrix
			Mat A = Mat::zeros(Xp.size(),3, CV_64F);
			Mat b = Mat::zeros(Xp.size(),1, CV_64F);
			for (int p = 0; p < Xp.size(); p++)
			{
				A.at<double>(p,0) = Xp[p];
				A.at<double>(p,1) = Yp[p];
				A.at<double>(p,2) = 1;
				b.at<double>(p,0) = Zp[p];
			}
			//cout << "A.size() " << A.size() << endl;
			
			// Pseudo Inverse in Solution of Over-determined Linear System of Equations
			// https://math.stackexchange.com/questions/99299/best-fitting-plane-given-a-set-of-points
			
			Mat At = A.t();
			//cout << "At.size() " << At.size() << endl;
			Mat AtA = At * A;
			//cout << "AtA " << AtA << endl;
			Mat AtAinv;
			invert(AtA, AtAinv, DECOMP_SVD);
			//cout << "AtAinv:\n" << AtAinv << endl;
		
			Mat x = AtAinv * At * b;
			//cout << "x:\n" << x << endl;
			
			for (int p = 0; p < Xc.size(); p++)
			{
				new_disp_img.at<double>(Yc[p],Xc[p]) = 1.0 * x.at<double>(0,0) * Xc[p] + 1.0 * x.at<double>(0,1) * Yc[p] + 1.0 * x.at<double>(0,2);
			}
		}
		double_disparity_images.push_back(new_disp_img);		
	}
}

void createPtCloud(int img_index, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudrgb, pcl::PointCloud<pcl::PointXYZ>::Ptr cloudxyz)
{
	cout << "Image index: " << img_index << endl;
	cloudrgb->width    = 7;
	cloudrgb->height   = 5;
	cloudrgb->is_dense = false;
	cloudrgb->points.resize (cloudrgb->width * cloudrgb->height);
	
	cloudxyz->width    = cloudrgb->width;
	cloudxyz->height   = cloudrgb->height;
	cloudxyz->is_dense = cloudrgb->is_dense;
	cloudxyz->points.resize (cloudxyz->width * cloudxyz->height);
	
	int point_clout_pts = 0;
	cv::Mat_<double> vec_tmp(4,1);
	
	Mat disp_img;
	if(use_segment_labels)
		disp_img = double_disparity_images[img_index];
	else
		disp_img = disparity_images[img_index];
	cout << "disp_img.type(): " << type2str(disp_img.type()) << endl;;
	for (int y = boundingBox; y < rows - boundingBox; ++y)
	{
		for (int x = cols_start_aft_cutout; x < cols - boundingBox; ++x)
		{
			double disp_val = 0;
			if(use_segment_labels)
				disp_val = disp_img.at<double>(y,x);		//disp_val = (double)disp_img.at<uint16_t>(y,x) / 200.0;
			else
				disp_val = (double)disp_img.at<uchar>(y,x);
			
			if (disp_val > minDisparity)
			{
				//reference: https://stackoverflow.com/questions/22418846/reprojectimageto3d-in-opencv
				vec_tmp(0)=x; vec_tmp(1)=y; vec_tmp(2)=disp_val; vec_tmp(3)=1;
				vec_tmp = Q*vec_tmp;
				vec_tmp /= vec_tmp(3);
				
				point_clout_pts++;
				
				pcl::PointXYZ pt_3d;
				pt_3d.x = (float)vec_tmp(0);
				pt_3d.y = (float)vec_tmp(1);
				pt_3d.z = (float)vec_tmp(2);
				
				pcl::PointXYZRGB pt_3drgb;
				pt_3drgb.x = pt_3d.x;
				pt_3drgb.y = pt_3d.y;
				pt_3drgb.z = pt_3d.z;
				Vec3b color = full_images[img_index].at<Vec3b>(Point(x, y));
				uint32_t rgb = ((uint32_t)color[2] << 16 | (uint32_t)color[1] << 8 | (uint32_t)color[0]);
				pt_3drgb.rgb = *reinterpret_cast<float*>(&rgb);
				
				cloudrgb->points.push_back(pt_3drgb);
				cloudxyz->points.push_back(pt_3d);
				//cout << pt_3d << endl;
			}
		}
	}
	cout << "point_clout_pts: " << point_clout_pts << endl;
	if(log_stuff)
		f << "point_clout_pts: " << point_clout_pts << endl;
}

pcl::registration::TransformationEstimation<pcl::PointXYZRGB, pcl::PointXYZRGB>::Matrix4 generateTmat(record_t pose)
{
	pcl::registration::TransformationEstimation<pcl::PointXYZRGB, pcl::PointXYZRGB>::Matrix4 t_mat;
	
	double tx = pose[1];
	double ty = pose[2];
	double tz = pose[3];
	double qx = pose[4];
	double qy = pose[5];
	double qz = pose[6];
	double qw = pose[7];
	
	double sqw = qw*qw;
	double sqx = qx*qx;
	double sqy = qy*qy;
	double sqz = qz*qz;
	
	Mat rot = Mat::zeros(cv::Size(3, 3), CV_64FC1);
	
	rot.at<double>(0,0) = sqx - sqy - sqz + sqw; // since sqw + sqx + sqy + sqz =1
	rot.at<double>(1,1) = -sqx + sqy - sqz + sqw;
	rot.at<double>(2,2) = -sqx - sqy + sqz + sqw;

	double tmp1 = qx*qy;
	double tmp2 = qz*qw;
	rot.at<double>(0,1) = 2.0 * (tmp1 + tmp2);
	rot.at<double>(1,0) = 2.0 * (tmp1 - tmp2);

	tmp1 = qx*qz;
	tmp2 = qy*qw;
	rot.at<double>(0,2) = 2.0 * (tmp1 - tmp2);
	rot.at<double>(2,0) = 2.0 * (tmp1 + tmp2);

	tmp1 = qy*qz;
	tmp2 = qx*qw;
	rot.at<double>(1,2) = 2.0 * (tmp1 + tmp2);
	rot.at<double>(2,1) = 2.0 * (tmp1 - tmp2);
	
	rot = rot.t();
	
	t_mat(0,0) = rot.at<double>(0,0);
	t_mat(0,1) = rot.at<double>(0,1);
	t_mat(0,2) = rot.at<double>(0,2);
	t_mat(1,0) = rot.at<double>(1,0);
	t_mat(1,1) = rot.at<double>(1,1);
	t_mat(1,2) = rot.at<double>(1,2);
	t_mat(2,0) = rot.at<double>(2,0);
	t_mat(2,1) = rot.at<double>(2,1);
	t_mat(2,2) = rot.at<double>(2,2);
	
	t_mat(0,3) = tx - tx * t_mat(0,0) - ty * t_mat(0,1) - tz * t_mat(0,2);
	t_mat(1,3) = ty - tx * t_mat(1,0) - ty * t_mat(1,1) - tz * t_mat(1,2);
	t_mat(2,3) = tz - tx * t_mat(2,0) - ty * t_mat(2,1) - tz * t_mat(2,2);
	t_mat(3,0) = t_mat(3,1) = t_mat(3,2) = 0.0;
	t_mat(3,3) = 1.0;
	
	return t_mat;
}

void transformPtCloud2(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudrgb, pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformed_cloudrgb, pcl::registration::TransformationEstimation<pcl::PointXYZRGB, pcl::PointXYZRGB>::Matrix4 transform)
{
	// Executing the transformation
	pcl::transformPointCloud(*cloudrgb, *transformed_cloudrgb, transform);
}

void transformPtCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudrgb, pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformed_cloudrgb, Eigen::Affine3f transform_2)
{
	// Executing the transformation
	pcl::transformPointCloud(*cloudrgb, *transformed_cloudrgb, transform_2);
}
