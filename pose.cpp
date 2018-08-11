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
		vector<pcl::registration::TransformationEstimation<pcl::PointXYZRGB, pcl::PointXYZRGB>::Matrix4> t_matVec;
		
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
			//int heading_index = data_index_arr[1];
			//
			//Eigen::Affine3f transform_MAVLink = Eigen::Affine3f::Identity();
			//// Define a translation on x, y and z axis.
			//transform_MAVLink.translation() << pose_data[pose_index][tx_ind], pose_data[pose_index][ty_ind], pose_data[pose_index][tz_ind];
			//
			//// The same rotation matrix as before; theta radians around Z axis
			//double theta = heading_data[heading_index][hdg_ind] * PI / 180;
			//transform_MAVLink.rotate (Eigen::AngleAxisf (theta, Eigen::Vector3f::UnitZ()));
			
			//Eigen::Quaternion<float> uav_quaternion = Eigen::Quaternion<float> (pose_data[pose_index][qx_ind], pose_data[pose_index][qy_ind], pose_data[pose_index][qz_ind], pose_data[pose_index][qw_ind]);
			pcl::registration::TransformationEstimation<pcl::PointXYZRGB, pcl::PointXYZRGB>::Matrix4 t_mat = generateTmat(pose_data[pose_index]);
			t_matVec.push_back(t_mat);
			
			// Print the transformation
			//printf ("Correcting 3D pt cloud using transform_MAVLink\n");
			//std::cout << transform_MAVLink.matrix() << std::endl;
			
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
		vector<pcl::registration::TransformationEstimation<pcl::PointXYZRGB, pcl::PointXYZRGB>::Matrix4> t_FMVec;
		
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformed_cloudrgb = transformed_cloudrgbVec[0];
		
		pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb0 (transformed_cloudrgb);
		viewer1.addPointCloud<pcl::PointXYZRGB> (transformed_cloudrgb, rgb0, "transformedFM_cloud" + to_string(0));
		viewer1.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "transformedFM_cloud" + to_string(0));
		
		transformedFeatureMatch_cloudrgbVec.push_back(transformed_cloudrgb);
		t_FMVec.push_back(t_matVec[0]);
		
		//printPoints(transformed_cloudrgb, 100 + 0);
		
		for (int i = 1; i < img_numbers.size(); i++)
		{
			//estimate rigid body transform between matched points
			pcl::registration::TransformationEstimation<pcl::PointXYZ, pcl::PointXYZ>::Matrix4 T_SVD_matched_pts = estimateRigidBodyTransformBetweenMatchedPoints(i, i-1, t_matVec[i], t_FMVec[i-1]);
			t_FMVec.push_back(T_SVD_matched_pts * t_matVec[i]);
			
			//working with 3D point clouds
			pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformedFM_cloudrgb0 = transformedFeatureMatch_cloudrgbVec[i-1];
			pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformedFM_cloudrgb1 = transformed_cloudrgbVec[i];
			pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformedFM_cloudrgb2( new pcl::PointCloud<pcl::PointXYZRGB>() );
			
			//pcl::registration::TransformationEstimationSVD<pcl::PointXYZRGB, pcl::PointXYZRGB> te00;
			//pcl::registration::TransformationEstimation<pcl::PointXYZRGB, pcl::PointXYZRGB>::Matrix4 T_SVD00;
			//
			//te00.estimateRigidTransformation(*transformedFM_cloudrgb1, *transformedFM_cloudrgb0, T_SVD00);
			//cout << "computed point cloud transformation is\n" << T_SVD00 << endl;
			
			transformPtCloud2(transformedFM_cloudrgb1, transformedFM_cloudrgb2, T_SVD_matched_pts);
			cout << "Transformed point cloud " << i << endl;
			
			pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb (transformedFM_cloudrgb2);
			viewer1.addPointCloud<pcl::PointXYZRGB> (transformedFM_cloudrgb2, rgb, "transformedFM_cloud" + to_string(i));
			viewer1.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "transformedFM_cloud" + to_string(i));
			
			transformedFeatureMatch_cloudrgbVec.push_back(transformedFM_cloudrgb2);
			
			//printPoints(transformedFM_cloudrgb1, i);
			//printPoints(transformedFM_cloudrgb2, 100 + i);
		}
		
		cout << "*** Display the visualiser until 'q' key is pressed ***" << endl;
		
		viewer1.addCoordinateSystem (1.0, 0, 0, 0);
		viewer1.setBackgroundColor(0.05, 0.05, 0.05, 0); // Setting background to a dark grey
		viewer1.setPosition(full_images[0].cols/2, full_images[0].rows/2); // Setting visualiser window position
		
		while (!viewer1.wasStopped ()) { // Display the visualiser until 'q' key is pressed
			viewer1.spinOnce();
		}
	}
	
	//Mat mtxR, mtxQ, transVect, rotMatrixX, rotMatrixY, rotMatrixZ;
	//decomposeProjectionMatrix(affineTransformationMatrix, cameraMatrix, rotMatrix, transVect, rotMatrixX, rotMatrixY, rotMatrixZ, eulerAngles);
	//RQDecomp3x3(rotations, mtxR, mtxQ, rotMatrixX, rotMatrixY, rotMatrixZ);
	//cout << "Decomposition:\nrotations:\n" << rotations << "\nmtxR:\n" << mtxR << "\nmtxQ :\n" << mtxQ << "\nrotMatrixX :\n" << rotMatrixX << "\nrotMatrixY :\n" << rotMatrixY << "\nrotMatrixZ :\n" << rotMatrixZ << endl;
	
	//cout << "Drawing and saving feature matches images..." << endl;
	//Mat matchedImage, matchedImageInliers;
	//drawMatches(full_images[0], keypoints_src, full_images[1], keypoints_dst, match.matches, matchedImage);
	//vector<char> inliers(match.inliers_mask.size());
	//for (int i = 0; i < match.inliers_mask.size(); i++)
	//{
	//	inliers[i] = match.inliers_mask[i];
	//}
	//drawMatches(full_images[0], keypoints_src, full_images[1], keypoints_dst, match.matches, matchedImageInliers, Scalar::all(-1), Scalar::all(-1), inliers, DrawMatchesFlags::DEFAULT);
	//imwrite("output/matchedImage.jpg", matchedImage);
	//imwrite("output/matchedImageInliers.jpg", matchedImageInliers);
	
	cout << "Finding 3D transformation, time: " << ((getTickCount() - t) / getTickFrequency()) << " sec" << endl;
	cout << "Finished Pose Estimation, total time: " << ((getTickCount() - app_start_time) / getTickFrequency()) << " sec" << endl;
	
}


void Pose::printPoints(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, int num)
{
	op_fl << "\npoint cloud " << num << endl;
	for (int i = 1000; i < 1010; i++)
	{
		op_fl << cloud->points[i].x << " " << cloud->points[i].y << " " << cloud->points[i].z << endl;
	}
}


pcl::registration::TransformationEstimation<pcl::PointXYZRGB, pcl::PointXYZRGB>::Matrix4 Pose::estimateRigidBodyTransformBetweenMatchedPoints(int img0_index, int img1_index,
	pcl::registration::TransformationEstimation<pcl::PointXYZRGB, pcl::PointXYZRGB>::Matrix4 t_mat0,
	pcl::registration::TransformationEstimation<pcl::PointXYZRGB, pcl::PointXYZRGB>::Matrix4 t_mat1)
{
	//using sequential matched points to estimate the rigid body transformation between matched 3D points
	MatchesInfo match;
	for (int j = 0; j < pairwise_matches.size(); j++)
	{
		if(pairwise_matches[j].src_img_idx == img0_index && pairwise_matches[j].dst_img_idx == img1_index)
		{
			//sequential pair found!
			match = pairwise_matches[j];
			cout << "src_img_idx: " << match.src_img_idx << " dst_img_idx: " << match.dst_img_idx << endl;
			f << "src_img_idx: " << match.src_img_idx << " dst_img_idx: " << match.dst_img_idx << endl;
			break;
		}
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
	
	cout << "Converting 2D matches to 3D matches..." << endl;
	cout << "match.inliers_mask.size() " << match.inliers_mask.size() << endl;
	f << "\nfinding transformation between images " << img0_index << " and " << img1_index << endl;
	f << "match.inliers_mask[i] match.matches[i].imgIdx queryIdx trainIdx" << endl;
	for (int i = 0; i < match.inliers_mask.size(); i++)
	{
		f << match.inliers_mask[i] << " " << match.matches[i].imgIdx << " " << match.matches[i].queryIdx << " " << match.matches[i].trainIdx << endl;
		
		//if (match.inliers_mask[i] == 1 && match.matches[i].imgIdx == match.src_img_idx)
		if (match.inliers_mask[i] == 1 && match.matches[i].imgIdx != -1)
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
	
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud0 (new pcl::PointCloud<pcl::PointXYZRGB> ());
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud1 (new pcl::PointCloud<pcl::PointXYZRGB> ());
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_t0 (new pcl::PointCloud<pcl::PointXYZRGB> ());
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_t1 (new pcl::PointCloud<pcl::PointXYZRGB> ());
	
	cloud0->width    = 7;
	cloud0->height   = 5;
	cloud0->is_dense = false;
	cloud0->points.resize (cloud0->width * cloud0->height);
	
	cloud1->width    = 7;
	cloud1->height   = 5;
	cloud1->is_dense = false;
	cloud1->points.resize (cloud1->width * cloud1->height);
	
	op_fl << "\npoint clouds " << img0_index << " and " << img1_index << endl;
	
	op_fl << "keypoints3D_src.size() " << keypoints3D_src.size() << endl;
	f << "keypoints3D_src.size() " << keypoints3D_src.size() << endl;
	cout << "keypoints3D_src.size() " << keypoints3D_src.size() << endl;
	
	for (int i = 0; i < keypoints3D_src.size(); ++i)
	{
		pcl::PointXYZRGB pt_3d_src, pt_3d_dst;
		
		pt_3d_src.x = keypoints3D_src[i].x;
		pt_3d_src.y = keypoints3D_src[i].y;
		pt_3d_src.z = keypoints3D_src[i].z;
		
		pt_3d_dst.x = keypoints3D_dst[i].x;
		pt_3d_dst.y = keypoints3D_dst[i].y;
		pt_3d_dst.z = keypoints3D_dst[i].z;
		
		cloud0->points.push_back(pt_3d_src);
		cloud1->points.push_back(pt_3d_dst);
		
		//op_fl << pt_3d_src.x << ">" << pt_3d_dst.x << "  " << pt_3d_src.y << ">" << pt_3d_dst.y << "  " << pt_3d_src.z << ">" << pt_3d_dst.z << "  " << endl;
	}
	
	//cout << "t_mat0 computations" << endl;
	//pcl::registration::TransformationEstimation<pcl::PointXYZ, pcl::PointXYZ>::Matrix4 t_mat0 = t_matVec[img0_index];
	//cout << "t_matVec[" << img0_index << "]\n" << t_matVec[img0_index] << endl;
	//int i = 0;
	//while (i < img0_index-1)
	//{
	//	cout << "T_SVD_matched_pts_Vec[" << i << "]\n" << T_SVD_matched_pts_Vec[i] << endl;
	//	t_mat0 = T_SVD_matched_pts_Vec[i] * t_mat0;
	//	++i;
	//}
	//cout << "t_mat0\n" << t_mat0 << endl;
	//cout << "t_mat1 computations" << endl;
	//pcl::registration::TransformationEstimation<pcl::PointXYZ, pcl::PointXYZ>::Matrix4 t_mat1 = t_matVec[img1_index];
	//cout << "t_matVec[" << img1_index << "]\n" << t_matVec[img1_index] << endl;
	//i = 0;
	//while (i < img1_index-1)
	//{
	//	cout << "T_SVD_matched_pts_Vec[" << i << "]\n" << T_SVD_matched_pts_Vec[i] << endl;
	//	t_mat1 = T_SVD_matched_pts_Vec[i] * t_mat1;
	//	++i;
	//}
	//cout << "t_mat1\n" << t_mat1 << endl;
	pcl::transformPointCloud(*cloud0, *cloud_t0, t_mat0);
	pcl::transformPointCloud(*cloud1, *cloud_t1, t_mat1);
	cout << "transformations done " << endl;
	cout << "cloud_t0->size() " << cloud_t0->size() << endl;
	cout << "cloud_t1->size() " << cloud_t1->size() << endl;
	
	cout << "Finding Rigid Body Transformation..." << endl;
	
	pcl::registration::TransformationEstimationSVD<pcl::PointXYZRGB, pcl::PointXYZRGB> te2;
	pcl::registration::TransformationEstimation<pcl::PointXYZRGB, pcl::PointXYZRGB>::Matrix4 T_SVD2;
	
	te2.estimateRigidTransformation(*cloud_t0, *cloud_t1, T_SVD2);
	cout << "computed transformation between MATCHED KEYPOINTS T_SVD2 is\n" << T_SVD2 << endl;
	f << "computed transformation between MATCHED KEYPOINTS T_SVD2 is\n" << T_SVD2 << endl;
	const Eigen::Quaternionf   R_SVD2 (T_SVD2.topLeftCorner  <3, 3> ());
	const Eigen::Translation3f t_SVD2 (T_SVD2.topRightCorner <3, 1> ());
	cout << "R_SVD2: x " << R_SVD2.x() << " y " << R_SVD2.y() << " z " << R_SVD2.z() << " w " << R_SVD2.w() << endl;
	cout << "t_SVD2: x " << t_SVD2.x() << " y " << t_SVD2.y() << " z " << t_SVD2.z() << endl;
	
	return T_SVD2;
}
