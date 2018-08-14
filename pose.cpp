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
	currentDateTimeStr = currentDateTime();
	cout << "currentDateTime=" << currentDateTimeStr << endl;

#if 0
	cv::setBreakOnError(true);
#endif
	int retVal = parseCmdArgs(argc, argv);
	if (retVal == -1)
		throw "Exception: Incorrect inputs!";
	
	if (downsample)
	{
		pcl::registration::TransformationEstimation<pcl::PointXYZRGB, pcl::PointXYZRGB>::Matrix4 tf_icp;
		if(downsample_transform)
		{
			Mat tf_icp_mat;
		
			cv::FileStorage fs0(downsample_transform_file, cv::FileStorage::READ);
			fs0["tf_icp"] >> tf_icp_mat;
			fs0.release();	// close tf values file
			
			for (int i = 0; i < 4; i++)
				for (int j = 0; j < 4; j++)
					tf_icp(i,j) = tf_icp_mat.at<double>(i,j);
		}
		
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudrgb = read_PLY_File(read_PLY_filename0);
		
		if (downsample_transform)
		{
			cout << "Transforming cloud using\n" << tf_icp << endl;
			pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudrgb_transformed (new pcl::PointCloud<pcl::PointXYZRGB> ());
			
			pcl::transformPointCloud(*cloudrgb, *cloudrgb_transformed, tf_icp);
			
			cloudrgb = cloudrgb_transformed;
			cout << "cloud transformed." << endl;
		}
		
		cerr << "PointCloud before filtering: " << cloudrgb->width * cloudrgb->height 
			<< " data points (" << pcl::getFieldsList (*cloudrgb) << ")." << endl;
		
		for (int i = 0; i < cloudrgb->size(); i++)
			cloudrgb->points[i].z += 500;	//increasing height to place all points at center of voxel of size 1000 m
		
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudrgb_filtered (new pcl::PointCloud<pcl::PointXYZRGB> ());
		
		// Create the filtering object
		pcl::VoxelGrid<pcl::PointXYZRGB> sor;
		sor.setInputCloud (cloudrgb);
		sor.setLeafSize (voxel_size,voxel_size,1000);
		sor.filter (*cloudrgb_filtered);

		for (int i = 0; i < cloudrgb_filtered->size(); i++)
			cloudrgb_filtered->points[i].z -= 500;	//changing back height to original place
		
		cerr << "PointCloud after filtering: " << cloudrgb_filtered->width * cloudrgb_filtered->height 
			<< " data points (" << pcl::getFieldsList (*cloudrgb_filtered) << ")." << endl;
		
		string writePath = "downsampled_" + read_PLY_filename0;
		save_pt_cloud_to_PLY_File(cloudrgb_filtered, writePath);
		
		visualize_pt_cloud(cloudrgb_filtered, "Downsampled Point Cloud");
		
		cout << "Cya!" << endl;
		return;
	}
	
	if (smooth_surface)
	{
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudrgb = read_PLY_File(read_PLY_filename0);
		
		pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr xyz_cloud_smoothed (new pcl::PointCloud<pcl::PointXYZRGBNormal> ());

		pcl::MovingLeastSquares<pcl::PointXYZRGB, pcl::PointXYZRGBNormal> mls;
		mls.setInputCloud (cloudrgb);
		mls.setSearchRadius (search_radius);
		mls.setSqrGaussParam (sqr_gauss_param);
		mls.setPolynomialOrder (polynomial_order);

		//  mls.setUpsamplingMethod (pcl::MovingLeastSquares<pcl::PointXYZRGB, pcl::PointNormal>::SAMPLE_LOCAL_PLANE);
		//  mls.setUpsamplingMethod (pcl::MovingLeastSquares<pcl::PointXYZRGB, pcl::PointNormal>::RANDOM_UNIFORM_DENSITY);
		//  mls.setUpsamplingMethod (pcl::MovingLeastSquares<pcl::PointXYZRGB, pcl::PointNormal>::VOXEL_GRID_DILATION);
		mls.setUpsamplingMethod (pcl::MovingLeastSquares<pcl::PointXYZRGB, pcl::PointXYZRGBNormal>::NONE);
		mls.setPointDensity (60000 * int (search_radius)); // 300 points in a 5 cm radius
		mls.setUpsamplingRadius (0.025);
		mls.setUpsamplingStepSize (0.015);
		mls.setDilationIterations (2);
		mls.setDilationVoxelSize (0.01f);

		// Define the tree to find the neighbors
		pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB> ());
		tree->setInputCloud (cloudrgb);
		mls.setSearchMethod (tree);
		mls.setComputeNormals (true);
		
		// Define the output and the normals
		pcl::PointCloud<pcl::PointXYZRGB> mls_points;
		pcl::PointCloud<pcl::Normal>::Ptr mls_normals (new pcl::PointCloud<pcl::Normal> ());
		mls.setOutputNormals (mls_normals);
		// Compute the smoothed cloud
		mls.reconstruct (mls_points);
		// Extra: merge fields
		pcl::PointCloud<pcl::PointNormal> mls_cloud;
		pcl::concatenateFields (mls_points, *mls_normals, mls_cloud);

		printf("Computing smoothed surface and normals with search_radius %f , sqr_gaussian_param %f, polynomial order %d\n", mls.getSearchRadius(), mls.getSqrGaussParam(), mls.getPolynomialOrder());
		
		mls.process (*xyz_cloud_smoothed);
		
		cout << "done smoothing." << endl;
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud;
		pcl::copyPointCloud(*xyz_cloud_smoothed,*cloud);
		
		//pcl::PCLPointCloud2 output;
		//pcl::toPCLPointCloud2 (*xyz_cloud_smoothed, output);
		//cout << "done conversion to PCLPointCloud2." << endl;
		//pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud;
		//pcl::fromPCLPointCloud2( output, *cloud);
		//cout << "done conversion to PointXYZRGB." << endl;
		
		string writePath = "smoothed_" + read_PLY_filename0;
		save_pt_cloud_to_PLY_File(cloud, writePath);
		
		visualize_pt_cloud(cloud, writePath);
		cout << "Cya!" << endl;
		return;
	}
	
	if (visualize)
	{
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudrgb = read_PLY_File(read_PLY_filename0);
		visualize_pt_cloud(cloudrgb, read_PLY_filename0);
		cout << "Cya!" << endl;
		return;
	}
	
	if (align_point_cloud)
	{
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in = read_PLY_File(read_PLY_filename0);
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_out = read_PLY_File(read_PLY_filename1);
		
		cout << "Running ICP to align point clouds..." << endl;
		pcl::IterativeClosestPoint<pcl::PointXYZRGB, pcl::PointXYZRGB> icp;
		icp.setInputSource(cloud_in);
		icp.setInputTarget(cloud_out);
		pcl::PointCloud<pcl::PointXYZRGB> Final;
		icp.align(Final);
		cout << "has converged: " << icp.hasConverged() << " score: " << icp.getFitnessScore() << endl;
		Eigen::Matrix4f icp_tf = icp.getFinalTransformation();
		cout << icp_tf << endl;
		
		Mat tf_icp = Mat::zeros(cv::Size(4, 4), CV_64FC1);
		for (int i = 0; i < 4; i++)
			for (int j = 0; j < 4; j++)
				tf_icp.at<double>(i,j) = icp_tf(i,j);
		
		string writePath = "tf_icp_" + currentDateTimeStr + ".yml";
		cv::FileStorage fs(writePath, cv::FileStorage::WRITE);
		fs << "tf_icp" << tf_icp;
		cout << "Wrote tf_icp values to " << writePath << endl;
		fs.release();	// close Settings file
		
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr Final_ptr (&Final);
		
		writePath = "ICP_aligned_" + read_PLY_filename0;
		//save_pt_cloud_to_PLY_File(Final_ptr, writePath);
		
		visualize_pt_cloud(Final_ptr, writePath);
		
		return;
	}
	
	if (join_point_clouds)
	{
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudrgb0 = read_PLY_File(read_PLY_filename0);
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudrgb1 = read_PLY_File(read_PLY_filename1);
		
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr combined_initial_pt_cloud (new pcl::PointCloud<pcl::PointXYZRGB> ());
		copyPointCloud(*cloudrgb0,*combined_initial_pt_cloud);
		combined_initial_pt_cloud->insert(combined_initial_pt_cloud->end(),cloudrgb1->begin(),cloudrgb1->end());
		visualize_pt_cloud(combined_initial_pt_cloud, "combined_initial_pt_cloud");
		
		//reading first and last tf values to yml file which can be used to join rows of point clouds later
		int row1_first, row1_last, row2_first, row2_last;
		Mat row1_tf_first_mat, row1_tf_last_mat, row2_tf_first_mat, row2_tf_last_mat;
		
		cv::FileStorage fs0(read_tf_filename0, cv::FileStorage::READ);
		fs0["img_first"] >> row1_first;
		fs0["tf_first"] >> row1_tf_first_mat;
		fs0["img_last"] >> row1_last;
		fs0["tf_last"] >> row1_tf_last_mat;
		fs0.release();	// close tf values file
		
		cv::FileStorage fs1(read_tf_filename1, cv::FileStorage::READ);
		fs1["img_first"] >> row2_first;
		fs1["tf_first"] >> row2_tf_first_mat;
		fs1["img_last"] >> row2_last;
		fs1["tf_last"] >> row2_tf_last_mat;
		fs1.release();	// close tf values file
		
		cout << "Read tf values" << endl;
		
		pcl::registration::TransformationEstimation<pcl::PointXYZRGB, pcl::PointXYZRGB>::Matrix4 row1_tf_first, row1_tf_last, row2_tf_first, row2_tf_last;
		for (int i = 0; i < 4; i++)
		{
			for (int j = 0; j < 4; j++)
			{
				row1_tf_first(i,j) = row1_tf_first_mat.at<double>(i,j);
				row1_tf_last(i,j) = row1_tf_last_mat.at<double>(i,j);
				row2_tf_first(i,j) = row2_tf_first_mat.at<double>(i,j);
				row2_tf_last(i,j) = row2_tf_last_mat.at<double>(i,j);
			}
		}
		
		//read first and last row images and generate matched keypoints
		img_numbers.push_back(row1_first);
		img_numbers.push_back(row1_last);
		img_numbers.push_back(row2_first);
		img_numbers.push_back(row2_last);
		cout << "images: ";
		for (int i = 0; i < img_numbers.size(); i++)
			cout << img_numbers[i] << " ";
		cout << endl;
		readCalibFile();
		readPoseFile();
		readImages();
		findFeatures();
		pairWiseMatching();
		
		//generate point clouds of matched keypoints and estimate rigid body transform between them
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr row1_ptcloud_first (new pcl::PointCloud<pcl::PointXYZRGB> ());
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr row1_ptcloud_last (new pcl::PointCloud<pcl::PointXYZRGB> ());
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr row2_ptcloud_first (new pcl::PointCloud<pcl::PointXYZRGB> ());
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr row2_ptcloud_last (new pcl::PointCloud<pcl::PointXYZRGB> ());
		generate_Matched_Keypoints_Point_Clouds(3, 0, row2_tf_last, row1_tf_first, row2_ptcloud_last, row1_ptcloud_first);
		generate_Matched_Keypoints_Point_Clouds(1, 2, row2_tf_first, row1_tf_last, row2_ptcloud_first, row1_ptcloud_last);
		
		//generating the combined first last image row point clouds
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr row1_ptcloud (new pcl::PointCloud<pcl::PointXYZRGB> ());
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr row2_ptcloud (new pcl::PointCloud<pcl::PointXYZRGB> ());
		row1_ptcloud->is_dense = true;
		row2_ptcloud->is_dense = true;
		//copyPointCloud(*row1_ptcloud_first,*row1_ptcloud);
		//row1_ptcloud->insert(row1_ptcloud->end(),row1_ptcloud_last->begin(),row1_ptcloud_last->end());
		//copyPointCloud(*row2_ptcloud_last,*row2_ptcloud);
		//row2_ptcloud->insert(row2_ptcloud->end(),row2_ptcloud_first->begin(),row2_ptcloud_first->end());
		copyPointCloud(*row1_ptcloud_last,*row1_ptcloud);
		row1_ptcloud->insert(row1_ptcloud->end(),row1_ptcloud_first->begin(),row1_ptcloud_first->end());
		copyPointCloud(*row2_ptcloud_first,*row2_ptcloud);
		row2_ptcloud->insert(row2_ptcloud->end(),row2_ptcloud_last->begin(),row2_ptcloud_last->end());
		
		cout << "Finding Rigid Body Transformation..." << endl;
	
		pcl::registration::TransformationEstimationSVD<pcl::PointXYZRGB, pcl::PointXYZRGB> te2;
		pcl::registration::TransformationEstimation<pcl::PointXYZRGB, pcl::PointXYZRGB>::Matrix4 T_SVD2;
		
		te2.estimateRigidTransformation(*row2_ptcloud, *row1_ptcloud, T_SVD2);
		cout << "computed transformation between MATCHED KEYPOINTS T_SVD2 is\n" << T_SVD2 << endl;
		f << "computed transformation between MATCHED KEYPOINTS T_SVD2 is\n" << T_SVD2 << endl;
		
		//transforming actual row2 point cloud to overlay over row1 point cloud
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudrgb1_transformed (new pcl::PointCloud<pcl::PointXYZRGB> ());
		pcl::transformPointCloud(*cloudrgb1, *cloudrgb1_transformed, T_SVD2);
		
		//generating the bigger combined point cloud
		cloudrgb0->insert(cloudrgb0->end(),cloudrgb1_transformed->begin(),cloudrgb1_transformed->end());
		
		string writePath = "joined_" + read_PLY_filename0 + "_" + read_PLY_filename1;
		save_pt_cloud_to_PLY_File(cloudrgb0, writePath);
		
		visualize_pt_cloud(cloudrgb0, writePath);
		cout << "Cya!" << endl;
		return;
	}
	
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
	//vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> cloudrgbVec;
	//vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> transformed_cloudrgbVec;
	vector<pcl::registration::TransformationEstimation<pcl::PointXYZRGB, pcl::PointXYZRGB>::Matrix4> t_matVec;
	//vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> transformedFeatureMatch_cloudrgbVec;
	//pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformedFeatureMatch_cloudrgb_last;
	vector<pcl::registration::TransformationEstimation<pcl::PointXYZRGB, pcl::PointXYZRGB>::Matrix4> t_FMVec;
	
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudrgb_MAVLink(new pcl::PointCloud<pcl::PointXYZRGB> ());
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudrgb_FeatureMatched(new pcl::PointCloud<pcl::PointXYZRGB> ());
	cloudrgb_MAVLink->is_dense = true;
	cloudrgb_FeatureMatched->is_dense = true;
	
	for (int i = 0; i < img_numbers.size(); i++)
	{
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudrgb(new pcl::PointCloud<pcl::PointXYZRGB> ());
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformed_cloudrgb( new pcl::PointCloud<pcl::PointXYZRGB>() );
		
		createPtCloud(i, cloudrgb);
		cout << "Created point cloud " << i << endl;
		
		//SEARCH PROCESS: get NSECS from images_times_data and search for corresponding or nearby entry in pose_data and heading_data
		int* data_index_arr = data_index_finder(img_numbers[i]);
		int pose_index = data_index_arr[0];
		//int heading_index = data_index_arr[1];
		//Eigen::Affine3f transform_MAVLink = Eigen::Affine3f::Identity();
		//// Define a translation on x, y and z axis.
		//transform_MAVLink.translation() << pose_data[pose_index][tx_ind], pose_data[pose_index][ty_ind], pose_data[pose_index][tz_ind];
		//// The same rotation matrix as before; theta radians around Z axis
		//double theta = heading_data[heading_index][hdg_ind] * PI / 180;
		//transform_MAVLink.rotate (Eigen::AngleAxisf (theta, Eigen::Vector3f::UnitZ()));
		
		pcl::registration::TransformationEstimation<pcl::PointXYZRGB, pcl::PointXYZRGB>::Matrix4 t_mat = generateTmat(pose_data[pose_index]);
		t_matVec.push_back(t_mat);
		
		transformPtCloud2(cloudrgb, transformed_cloudrgb, t_mat);
		cout << "Transformed point cloud " << i << endl;
		
		//cloudrgbVec.push_back(cloudrgb);
		//transformed_cloudrgbVec.push_back(transformed_cloudrgb);
		
		//generating the bigger point cloud
		cloudrgb_MAVLink->insert(cloudrgb_MAVLink->end(),transformed_cloudrgb->begin(),transformed_cloudrgb->end());
		cout << "inserted to cloudrgb_MAVLink" << endl;
		
		//Feature Matching Alignment
		if (i == 0)
		{
			//transformedFeatureMatch_cloudrgbVec.push_back(transformed_cloudrgb);
			//transformedFeatureMatch_cloudrgb_last = transformed_cloudrgb;
			t_FMVec.push_back(t_mat);
			
			//generating the bigger point cloud
			copyPointCloud(*transformed_cloudrgb,*cloudrgb_FeatureMatched);
		}
		else
		{
			//generate point clouds of matched keypoints and estimate rigid body transform between them
			//pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_t0 (new pcl::PointCloud<pcl::PointXYZRGB> ());
			//pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_t1 (new pcl::PointCloud<pcl::PointXYZRGB> ());
			//pcl::registration::TransformationEstimation<pcl::PointXYZRGB, pcl::PointXYZRGB>::Matrix4 T_SVD_matched_pts = generate_Matched_Keypoints_Point_Clouds(i, i-1, t_mat, t_FMVec[i-1], cloud_t0, cloud_t1);
			
			//generate point clouds of matched keypoints and estimate rigid body transform between them
			pcl::registration::TransformationEstimation<pcl::PointXYZRGB, pcl::PointXYZRGB>::Matrix4 T_SVD_matched_pts = generate_tf_of_Matched_Keypoints_Point_Cloud(i, t_FMVec, t_mat);
			
			t_FMVec.push_back(T_SVD_matched_pts * t_mat);
			
			//working with 3D point clouds
			//pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformedFM_cloudrgb0 = transformedFeatureMatch_cloudrgb_last;
			pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformedFM_cloudrgb1( new pcl::PointCloud<pcl::PointXYZRGB>() );
			
			transformPtCloud2(transformed_cloudrgb, transformedFM_cloudrgb1, T_SVD_matched_pts);
			cout << "Feature Matched and Transformed point cloud " << i << endl;
			
			//transformedFeatureMatch_cloudrgbVec.push_back(transformedFM_cloudrgb1);
			//transformedFeatureMatch_cloudrgb_last = transformedFM_cloudrgb1;
			
			//generating the bigger point cloud
			cloudrgb_FeatureMatched->insert(cloudrgb_FeatureMatched->end(),transformedFM_cloudrgb1->begin(),transformedFM_cloudrgb1->end());
		}
	}
	
	string writePath = "cloudrgb_MAVLink_" + currentDateTimeStr + ".ply";
	save_pt_cloud_to_PLY_File(cloudrgb_MAVLink, writePath);
	
	writePath = "cloudrgb_FeatureMatched_" + currentDateTimeStr + ".ply";
	save_pt_cloud_to_PLY_File(cloudrgb_FeatureMatched, writePath);
	
	cout << "Finding 3D transformation, time: " << ((getTickCount() - t) / getTickFrequency()) << " sec" << endl;
	cout << "Finished Pose Estimation, total time: " << ((getTickCount() - app_start_time) / getTickFrequency()) << " sec" << endl;
	
	//writing first and last tf values to yml file which can be used to join rows of point clouds later
	int img_first = img_numbers[0];
	int img_last = img_numbers[img_numbers.size()-1];
	//pcl::registration::TransformationEstimation<pcl::PointXYZRGB, pcl::PointXYZRGB>::Matrix4 tf_first = t_FMVec[0];
	//pcl::registration::TransformationEstimation<pcl::PointXYZRGB, pcl::PointXYZRGB>::Matrix4 tf_last = t_FMVec[img_numbers.size()-1];
	Mat tf_first = Mat::zeros(cv::Size(4, 4), CV_64FC1);
	Mat tf_last = Mat::zeros(cv::Size(4, 4), CV_64FC1);
	for (int i = 0; i < 4; i++)
	{
		for (int j = 0; j < 4; j++)
		{
			tf_first.at<double>(i,j) = t_FMVec[0](i,j);
			tf_last.at<double>(i,j) = t_FMVec[img_numbers.size()-1](i,j);
		}
	}
	cout << "img_numbers[0]: " << img_first << endl;
	cout << "computed point cloud transformation is\n" << tf_first << endl;
	cout << "img_numbers[img_numbers.size()-1]: " << img_last << endl;
	cout << "computed point cloud transformation is\n" << tf_last << endl;
	
	writePath = "tf_info_" + currentDateTimeStr + ".yml";
	cv::FileStorage fs(writePath, cv::FileStorage::WRITE);
	fs << "img_first" << img_first;
	fs << "tf_first" << tf_first;
	fs << "img_last" << img_last;
	fs << "tf_last" << tf_last;
	cout << "Wrote tf values to " << writePath << endl;
	fs.release();	// close Settings file
	
	if(preview)
	{
		visualize_pt_cloud(cloudrgb_MAVLink, "cloudrgb_MAVLink");
		visualize_pt_cloud(cloudrgb_FeatureMatched, "cloudrgb_FeatureMatched");
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
	
}

pcl::registration::TransformationEstimation<pcl::PointXYZRGB, pcl::PointXYZRGB>::Matrix4 Pose::generate_Matched_Keypoints_Point_Clouds(int img0_index, int img1_index,
	pcl::registration::TransformationEstimation<pcl::PointXYZRGB, pcl::PointXYZRGB>::Matrix4 t_mat0,
	pcl::registration::TransformationEstimation<pcl::PointXYZRGB, pcl::PointXYZRGB>::Matrix4 t_mat1,
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud_t0, pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud_t1)
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
	cout << "Confidence: " << match.confidence << endl;
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
	//f << "\nfinding transformation between images " << img0_index << " and " << img1_index << endl;
	//f << "match.inliers_mask[i] match.matches[i].imgIdx queryIdx trainIdx" << endl;
	for (int i = 0; i < match.inliers_mask.size(); i++)
	{
		//f << match.inliers_mask[i] << " " << match.matches[i].imgIdx << " " << match.matches[i].queryIdx << " " << match.matches[i].trainIdx << endl;
		
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

				//if(log_stuff)
				//	f << "srcQ3D_point " << src_3D_pt << " dstQ3D_point " << dst_3D_pt << endl;
			}

		}
	}
	
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud0 (new pcl::PointCloud<pcl::PointXYZRGB> ());
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud1 (new pcl::PointCloud<pcl::PointXYZRGB> ());
	cloud0->is_dense = true;
	cloud1->is_dense = true;
	
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
	}
	
	pcl::transformPointCloud(*cloud0, *cloud_t0, t_mat0);
	pcl::transformPointCloud(*cloud1, *cloud_t1, t_mat1);
	cout << "transformations done " << endl;
	cout << "cloud_t0->size() " << cloud_t0->size() << endl;
	cout << "cloud_t1->size() " << cloud_t1->size() << endl;
	
	cout << "Finding Rigid Body Transformation..." << endl;
	
	pcl::registration::TransformationEstimationSVD<pcl::PointXYZRGB, pcl::PointXYZRGB> te2;
	pcl::registration::TransformationEstimation<pcl::PointXYZRGB, pcl::PointXYZRGB>::Matrix4 T_SVD_matched_pts;
	
	te2.estimateRigidTransformation(*cloud_t0, *cloud_t1, T_SVD_matched_pts);
	cout << "computed transformation between MATCHED KEYPOINTS T_SVD2 is\n" << T_SVD_matched_pts << endl;
	f << "computed transformation between MATCHED KEYPOINTS T_SVD2 is\n" << T_SVD_matched_pts << endl;
	
	return T_SVD_matched_pts;
}

pcl::registration::TransformationEstimation<pcl::PointXYZRGB, pcl::PointXYZRGB>::Matrix4 Pose::generate_tf_of_Matched_Keypoints_Point_Cloud
(int img_index, vector<pcl::registration::TransformationEstimation<pcl::PointXYZRGB, pcl::PointXYZRGB>::Matrix4> &t_FMVec, 
pcl::registration::TransformationEstimation<pcl::PointXYZRGB, pcl::PointXYZRGB>::Matrix4 t_mat_MAVLink)
{
	cout << "generate_tf_of_Matched_Keypoints_Point_Cloud img_index " << img_index << endl;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_current (new pcl::PointCloud<pcl::PointXYZRGB> ());
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_prior (new pcl::PointCloud<pcl::PointXYZRGB> ());
	cloud_current->is_dense = true;
	cloud_prior->is_dense = true;
	
	//using sequential matched points to estimate the rigid body transformation between matched 3D points
	bool first_match = true;
	for (int j = 0; j < pairwise_matches.size(); j++)
	{
		MatchesInfo match = pairwise_matches[j];
		if(match.src_img_idx == img_index && match.dst_img_idx <= img_index-1 && match.inliers_mask.size() > 150)
		{
			cout << "img_index " << img_index << " src " << match.src_img_idx << " dst " << match.dst_img_idx << " confidence " << match.confidence << " inliers " << match.inliers_mask.size() << " matches " << match.matches.size() << endl;
			f << "img_index " << img_index << " src " << match.src_img_idx << " dst " << match.dst_img_idx << " confidence " << match.confidence << " inliers " << match.inliers_mask.size() << " matches " << match.matches.size() << endl;
			
			vector<KeyPoint> keypoints_src, keypoints_dst;
			keypoints_src = features[match.src_img_idx].keypoints;
			keypoints_dst = features[match.dst_img_idx].keypoints;
			
			Mat disp_img_src = disparity_images[match.src_img_idx];
			Mat disp_img_dst = disparity_images[match.dst_img_idx];
			
			//define 3d points for all keypoints
			vector<Point3d> keypoints3D_src, keypoints3D_dst;
			vector<int> keypoints3D_2D_index_src, keypoints3D_2D_index_dst;
			
			cout << "Converting 2D matches to 3D matches..." << endl;
			cout << "match.inliers_mask.size() " << match.inliers_mask.size() << endl;
			
			for (int i = 0; i < match.inliers_mask.size(); i++)
			{
				if (match.inliers_mask[i] == 1 && match.matches[i].imgIdx != -1)
				{
					int trainIdx = match.matches[i].trainIdx;
					int queryIdx = match.matches[i].queryIdx;
					
					//*3. convert corresponding features to 3D using disparity image information
					int disp_val_src = (int)disp_img_src.at<char>(keypoints_src[queryIdx].pt.y, keypoints_src[queryIdx].pt.x);
					int disp_val_dst = (int)disp_img_dst.at<char>(keypoints_dst[trainIdx].pt.y, keypoints_dst[trainIdx].pt.x);

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
					}

				}
			}
			f << "keypoints3D_src.size() " << keypoints3D_src.size() << endl;
			cout << "keypoints3D_src.size() " << keypoints3D_src.size() << endl;
			
			pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_current_temp (new pcl::PointCloud<pcl::PointXYZRGB> ());
			pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_prior_temp (new pcl::PointCloud<pcl::PointXYZRGB> ());
			cloud_current_temp->is_dense = true;
			cloud_prior_temp->is_dense = true;
			
			for (int i = 0; i < keypoints3D_src.size(); ++i)
			{
				pcl::PointXYZRGB pt_3d_src, pt_3d_dst;
				
				pt_3d_src.x = keypoints3D_src[i].x;
				pt_3d_src.y = keypoints3D_src[i].y;
				pt_3d_src.z = keypoints3D_src[i].z;
				
				pt_3d_dst.x = keypoints3D_dst[i].x;
				pt_3d_dst.y = keypoints3D_dst[i].y;
				pt_3d_dst.z = keypoints3D_dst[i].z;
				
				cloud_current_temp->points.push_back(pt_3d_src);
				cloud_prior_temp->points.push_back(pt_3d_dst);
			}
			cout << "cloud_current_temp->size() " << cloud_current_temp->size() << endl;
			cout << "cloud_prior_temp->size() " << cloud_prior_temp->size() << endl;
			pcl::registration::TransformationEstimation<pcl::PointXYZRGB, pcl::PointXYZRGB>::Matrix4 t_FM = t_FMVec[match.dst_img_idx];
			cout << "t_FMVec[" << match.dst_img_idx << "]\n" << t_FM << endl;
			
			pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_current_t_temp (new pcl::PointCloud<pcl::PointXYZRGB> ());
			pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_prior_t_temp (new pcl::PointCloud<pcl::PointXYZRGB> ());
			
			pcl::transformPointCloud(*cloud_current_temp, *cloud_current_t_temp, t_mat_MAVLink);
			cout << "cloud_current_temp transformed." << endl;
			pcl::transformPointCloud(*cloud_prior_temp, *cloud_prior_t_temp, t_FM);
			cout << "cloud_prior_temp transformed." << endl;
			
			if (first_match)
			{
				copyPointCloud(*cloud_current_t_temp,*cloud_current);
				copyPointCloud(*cloud_prior_t_temp,*cloud_prior);
				first_match = false;
				cout << "clouds copied!" << endl;
			}
			else
			{
				cloud_current->insert(cloud_current->end(),cloud_current_t_temp->begin(),cloud_current_t_temp->end());
				cloud_prior->insert(cloud_prior->end(),cloud_prior_t_temp->begin(),cloud_prior_t_temp->end());
				cout << "clouds inserted!" << endl;
			}
		}
		if (match.src_img_idx >= img_index && match.dst_img_idx >= img_index)
			break;
	}
	
	cout << "cloud_current->size() " << cloud_current->size() << endl;
	cout << "cloud_prior->size() " << cloud_prior->size() << endl;
	
	cout << "Finding Rigid Body Transformation..." << endl;
	
	pcl::registration::TransformationEstimationSVD<pcl::PointXYZRGB, pcl::PointXYZRGB> te2;
	pcl::registration::TransformationEstimation<pcl::PointXYZRGB, pcl::PointXYZRGB>::Matrix4 T_SVD_matched_pts;
	
	te2.estimateRigidTransformation(*cloud_current, *cloud_prior, T_SVD_matched_pts);
	cout << "computed transformation between MATCHED KEYPOINTS T_SVD2 is\n" << T_SVD_matched_pts << endl;
	f << "computed transformation between MATCHED KEYPOINTS T_SVD2 is\n" << T_SVD_matched_pts << endl;
	
	return T_SVD_matched_pts;
}
