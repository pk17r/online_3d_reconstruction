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

Pose::Pose(int argc, char* argv[])
{
#if 0
	cv::setBreakOnError(true);
#endif
	int retVal = parseCmdArgs(argc, argv);
	if (retVal == -1)
		throw "Exception: Incorrect inputs!";
	
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
	
	op_fl.open("output/points.txt", ios::out);
	
	//view 3D point cloud of first image & disparity map
	if(preview)
	{
		pcl::visualization::PCLVisualizer viewer ("3d reconstruction");
		
		vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> cloudrgbVec;
		vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cloudxyzVec;
		vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> transformed_cloudrgbVec;
		
		for (int i = 0; i < img_numbers.size(); i++)
		{
			pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudrgb(new pcl::PointCloud<pcl::PointXYZRGB> ());
			pcl::PointCloud<pcl::PointXYZ>::Ptr cloudxyz(new pcl::PointCloud<pcl::PointXYZ> ());
			pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformed_cloudrgb( new pcl::PointCloud<pcl::PointXYZRGB>() );
			
			createPtCloud(i, cloudrgb, cloudxyz);
			cout << "Created point cloud " << i << endl;
			
			//SEARCH PROCESS: get NSECS from images_times_data and search for corresponding or nearby entry in pose_data and heading_data
			int* data_index_arr = data_index_finder(img_numbers[i]);
			int pose_index = data_index_arr[0];
			int heading_index = data_index_arr[1];
			
			Eigen::Affine3f transform_MAVLink = Eigen::Affine3f::Identity();
			// Define a translation on x, y and z axis.
			transform_MAVLink.translation() << pose_data[pose_index][tx_ind], pose_data[pose_index][ty_ind], pose_data[pose_index][tz_ind];
			
			// The same rotation matrix as before; theta radians around Z axis
			double theta = heading_data[heading_index][hdg_ind] * PI / 180;
			transform_MAVLink.rotate (Eigen::AngleAxisf (theta, Eigen::Vector3f::UnitZ()));
			
			//Eigen::Quaternion<float> uav_quaternion = Eigen::Quaternion<float> (pose_data[pose_index][qx_ind], pose_data[pose_index][qy_ind], pose_data[pose_index][qz_ind], pose_data[pose_index][qw_ind]);
			pcl::registration::TransformationEstimation<pcl::PointXYZRGB, pcl::PointXYZRGB>::Matrix4 t_mat = generateTmat(pose_data[pose_index]);
			
			// Print the transformation
			printf ("Correcting 3D pt cloud using transform_MAVLink\n");
			std::cout << transform_MAVLink.matrix() << std::endl;
			
			//transformPtCloud(cloudrgb, transformed_cloudrgb, transform_MAVLink);
			transformPtCloud2(cloudrgb, transformed_cloudrgb, t_mat);
			cout << "Transformed point cloud " << i << endl;
			
			pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb (transformed_cloudrgb);
			viewer.addPointCloud<pcl::PointXYZRGB> (transformed_cloudrgb, rgb, "transformed_cloud" + to_string(i));
			viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "transformed_cloud" + to_string(i));
			
			cloudrgbVec.push_back(cloudrgb);
			cloudxyzVec.push_back(cloudxyz);
			transformed_cloudrgbVec.push_back(transformed_cloudrgb);
			
		}

		cout << "*** Display the visualiser until 'q' key is pressed ***" << endl;
		
		viewer.addCoordinateSystem (1.0, 0, 0, 0);
		viewer.setBackgroundColor(0.05, 0.05, 0.05, 0); // Setting background to a dark grey
		viewer.setPosition(full_images[0].cols/2, full_images[0].rows/2); // Setting visualiser window position
		
		while (!viewer.wasStopped ()) { // Display the visualiser until 'q' key is pressed
			viewer.spinOnce();
		}
		
		//3D reconstruction using Feature Matching
		
		pcl::visualization::PCLVisualizer viewer1 ("3d reconstruction Feature Matching");
		
		vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> transformedFeatureMatch_cloudrgbVec;
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformed_cloudrgb = transformed_cloudrgbVec[0];
		
		pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb0 (transformed_cloudrgb);
		viewer1.addPointCloud<pcl::PointXYZRGB> (transformed_cloudrgb, rgb0, "transformedFM_cloud" + to_string(0));
		viewer1.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "transformedFM_cloud" + to_string(0));
		
		transformedFeatureMatch_cloudrgbVec.push_back(transformed_cloudrgb);
		
		printPoints(transformed_cloudrgb, 100 + 0);
		
		for (int i = 1; i < img_numbers.size(); i++)
		{
			//using sequential matched points to estimate the rigid body transformation between matched 3D points
			MatchesInfo match;
			for (int j = 0; j < pairwise_matches.size(); j++)
			{
				if(pairwise_matches[j].src_img_idx == i-1 && pairwise_matches[j].dst_img_idx == i)
				{
					//sequential pair found!
					match = pairwise_matches[j];
					cout << "src_img_idx: " << match.src_img_idx << " dst_img_idx: " << match.dst_img_idx << endl;
				}
			}
					
			
			//working with 3D point clouds
			pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformedFM_cloudrgb0 = transformedFeatureMatch_cloudrgbVec[i-1];
			pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformedFM_cloudrgb1 = transformed_cloudrgbVec[i];
			pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformedFM_cloudrgb2( new pcl::PointCloud<pcl::PointXYZRGB>() );
			
			pcl::registration::TransformationEstimationSVD<pcl::PointXYZRGB, pcl::PointXYZRGB> te00;
			pcl::registration::TransformationEstimation<pcl::PointXYZRGB, pcl::PointXYZRGB>::Matrix4 T_SVD00;
			
			te00.estimateRigidTransformation(*transformedFM_cloudrgb1, *transformedFM_cloudrgb0, T_SVD00);
			cout << "computed point cloud transformation is\n" << T_SVD00 << endl;
			
			transformPtCloud2(transformedFM_cloudrgb1, transformedFM_cloudrgb2, T_SVD00);
			cout << "Transformed point cloud " << i << endl;
			
			pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb (transformedFM_cloudrgb2);
			viewer1.addPointCloud<pcl::PointXYZRGB> (transformedFM_cloudrgb2, rgb, "transformedFM_cloud" + to_string(i));
			viewer1.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "transformedFM_cloud" + to_string(i));
			
			transformedFeatureMatch_cloudrgbVec.push_back(transformedFM_cloudrgb2);
			
			printPoints(transformedFM_cloudrgb1, i);
			printPoints(transformedFM_cloudrgb2, 100 + i);
		}
		
		cout << "*** Display the visualiser until 'q' key is pressed ***" << endl;
		
		viewer1.addCoordinateSystem (1.0, 0, 0, 0);
		viewer1.setBackgroundColor(0.05, 0.05, 0.05, 0); // Setting background to a dark grey
		viewer1.setPosition(full_images[0].cols/2, full_images[0].rows/2); // Setting visualiser window position
		
		while (!viewer1.wasStopped ()) { // Display the visualiser until 'q' key is pressed
			viewer1.spinOnce();
		}
		
	}
	
	cout << "Converting 2D matches to 3D matches..." << endl;
	t = getTickCount();
	
	//f << "train is dst and query is src" << endl;
	if(log_stuff)
		f << "3D matched points: " << endl;
	
	MatchesInfo match = pairwise_matches[1];
	
	cout << "pairwise_matches size: " << pairwise_matches.size() << endl;
	for (int i = 0; i < pairwise_matches.size(); i++)
	{
		cout << "src_img_idx: " << pairwise_matches[i].src_img_idx << " dst_img_idx: " << pairwise_matches[i].dst_img_idx << endl;
	}
	
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
		
		Eigen::Vector4f sensor_origin = Eigen::Vector4f(pose_data[found_index][1], pose_data[found_index][2], pose_data[found_index][3], 0);
		// Print the transformation
		printf ("sensor_origin\n");
		std::cout << sensor_origin << std::endl;
				
		Eigen::Quaternion<float> sensor_quaternion = Eigen::Quaternion<float> (pose_data[found_index][4], pose_data[found_index][5], pose_data[found_index][6], pose_data[found_index][7]);
		
		// Define R,G,B colors for the point cloud
		pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb0 (cloud0);
		//pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb1 (cloud1);
		
		viewer.addPointCloud<pcl::PointXYZRGB> (cloud0, rgb0, &sensor_origin, &sensor_quaternion, "cloud" + to_string(0), 0);
		//viewer.addPointCloud<pcl::PointXYZRGB> (cloud1, rgb1, "cloud" + to_string(1));
		
		viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "transformed_cloud" + to_string(0));
		//viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "transformed_cloud" + to_string(1));
		
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
	*/
	//*4. find transformation between corresponding 3D points using estimateAffine3D: Output 3D affine transformation matrix  3 x 4
	if(log_stuff)
		f << "Q:\n" << Q << endl;

	cout << "Finding 3D transformation, time: " << ((getTickCount() - t) / getTickFrequency()) << " sec" << endl;
	cout << "Finished Pose Estimation, total time: " << ((getTickCount() - app_start_time) / getTickFrequency()) << " sec" << endl;
	
	if(log_stuff)
	{
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
	
}
