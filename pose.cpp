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
	cout << "currentDateTime=" << currentDateTimeStr << "\n\n";

#if 0
	cv::setBreakOnError(true);
#endif
	if (parseCmdArgs(argc, argv) == -1) return;
	
	if (downsample)
	{
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudrgb = read_PLY_File(read_PLY_filename0);
		
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudrgb_filtered = downsamplePtCloud(cloudrgb);
		
		string writePath = "downsampled_" + read_PLY_filename0;
		save_pt_cloud_to_PLY_File(cloudrgb_filtered, writePath);
		
		visualize_pt_cloud(cloudrgb_filtered, "Downsampled Point Cloud");
		
		cout << "Cya!" << endl;
		return;
	}
	
	if (smooth_surface)
	{
		/*pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudrgb = read_PLY_File(read_PLY_filename0);
		
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
		return;*/
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
		
		pcl::registration::TransformationEstimation<pcl::PointXYZRGB, pcl::PointXYZRGB>::Matrix4 tf_icp = runICPalignment(cloud_in, cloud_out);
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_Fitted(new pcl::PointCloud<pcl::PointXYZRGB> ());
		transformPtCloud(cloud_in, cloud_Fitted, tf_icp);
		
		string writePath = "ICP_aligned_" + read_PLY_filename0;
		save_pt_cloud_to_PLY_File(cloud_Fitted, writePath);
		
		visualize_pt_cloud(cloud_Fitted, writePath);
		
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
	
	//view 3D point cloud of first image & disparity map
	//vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> cloudrgbVec;
	//vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> transformed_cloudrgbVec;
	vector<pcl::registration::TransformationEstimation<pcl::PointXYZRGB, pcl::PointXYZRGB>::Matrix4> t_matVec;
	//vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> transformedFeatureMatch_cloudrgbVec;
	//pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformedFeatureMatch_cloudrgb_last;
	vector<pcl::registration::TransformationEstimation<pcl::PointXYZRGB, pcl::PointXYZRGB>::Matrix4> t_FMVec;
	
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_hexPos_MAVLink(new pcl::PointCloud<pcl::PointXYZRGB> ());
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_hexPos_FM(new pcl::PointCloud<pcl::PointXYZRGB> ());
	cloud_hexPos_MAVLink->is_dense = true;
	cloud_hexPos_FM->is_dense = true;
	ofstream hexPosMAVLinkfile, hexPosFMfile, hexPosFMFittedfile;
	hexPosMAVLinkfile.open(hexPosMAVLinkfilename, ios::out);
	hexPosFMfile.open(hexPosFMfilename, ios::out);
	
	for (int i = 0; i < img_numbers.size(); i++)
	{
		//pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudrgb(new pcl::PointCloud<pcl::PointXYZRGB> ());
		//pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformed_cloudrgb( new pcl::PointCloud<pcl::PointXYZRGB>() );
		//
		//createPtCloud(i, cloudrgb);
		//cout << "Created point cloud " << i << endl;
		
		//SEARCH PROCESS: get NSECS from images_times_data and search for corresponding or nearby entry in pose_data and heading_data
		int pose_index = data_index_finder(img_numbers[i]);
		
		pcl::PointXYZRGB hexPosMAVLink;
		hexPosMAVLink.x = pose_data[pose_index][tx_ind];
		hexPosMAVLink.y = pose_data[pose_index][ty_ind];
		hexPosMAVLink.z = pose_data[pose_index][tz_ind];
		uint32_t rgbMAVLink = (uint32_t)255;
		hexPosMAVLink.rgb = *reinterpret_cast<float*>(&rgbMAVLink);
		hexPosMAVLinkVec.push_back(hexPosMAVLink);
		hexPosMAVLinkfile << hexPosMAVLink.x << "," << hexPosMAVLink.y << "," << hexPosMAVLink.z << endl;
		cloud_hexPos_MAVLink->points.push_back(hexPosMAVLink);
		
		pcl::registration::TransformationEstimation<pcl::PointXYZRGB, pcl::PointXYZRGB>::Matrix4 t_mat = generateTmat(pose_data[pose_index]);
		t_matVec.push_back(t_mat);
		
		//transformPtCloud(cloudrgb, transformed_cloudrgb, t_mat);
		//cout << "Transformed point cloud " << i << endl;
		
		//cloudrgbVec.push_back(cloudrgb);
		//transformed_cloudrgbVec.push_back(transformed_cloudrgb);
		
		//generating the bigger point cloud
		//cloudrgb_MAVLink->insert(cloudrgb_MAVLink->end(),transformed_cloudrgb->begin(),transformed_cloudrgb->end());
		//cout << "inserted to cloudrgb_MAVLink" << endl;
		
		//Feature Matching Alignment
		if (i == 0)
		{
			//transformedFeatureMatch_cloudrgbVec.push_back(transformed_cloudrgb);
			//transformedFeatureMatch_cloudrgb_last = transformed_cloudrgb;
			t_FMVec.push_back(t_mat);
			
			//generating the bigger point cloud
			//copyPointCloud(*transformed_cloudrgb,*cloudrgb_FeatureMatched);
		}
		else
		{			
			//generate point clouds of matched keypoints and estimate rigid body transform between them
			pcl::registration::TransformationEstimation<pcl::PointXYZRGB, pcl::PointXYZRGB>::Matrix4 T_SVD_matched_pts = generate_tf_of_Matched_Keypoints_Point_Cloud(i, t_FMVec, t_mat);
			
			t_FMVec.push_back(T_SVD_matched_pts * t_mat);
			
			pcl::PointXYZRGB hexPosFM;// = pcl::transformPoint(hexPosMAVLink, T_SVD_matched_pts);
			hexPosFM.x = static_cast<float> (T_SVD_matched_pts (0, 0) * hexPosMAVLink.x + T_SVD_matched_pts (0, 1) * hexPosMAVLink.y + T_SVD_matched_pts (0, 2) * hexPosMAVLink.z + T_SVD_matched_pts (0, 3));
			hexPosFM.y = static_cast<float> (T_SVD_matched_pts (1, 0) * hexPosMAVLink.x + T_SVD_matched_pts (1, 1) * hexPosMAVLink.y + T_SVD_matched_pts (1, 2) * hexPosMAVLink.z + T_SVD_matched_pts (1, 3));
			hexPosFM.z = static_cast<float> (T_SVD_matched_pts (2, 0) * hexPosMAVLink.x + T_SVD_matched_pts (2, 1) * hexPosMAVLink.y + T_SVD_matched_pts (2, 2) * hexPosMAVLink.z + T_SVD_matched_pts (2, 3));
			uint32_t rgbFM = (uint32_t)255 << 8;	//green
			hexPosFM.rgb = *reinterpret_cast<float*>(&rgbFM);
			hexPosFMVec.push_back(hexPosFM);
			hexPosFMfile << hexPosFM.x << "," << hexPosFM.y << "," << hexPosFM.z << endl;
			cloud_hexPos_FM->points.push_back(hexPosFM);
			
			//working with 3D point clouds
			//pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformedFM_cloudrgb0 = transformedFeatureMatch_cloudrgb_last;
			//pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformedFM_cloudrgb1( new pcl::PointCloud<pcl::PointXYZRGB>() );
			//
			//transformPtCloud(transformed_cloudrgb, transformedFM_cloudrgb1, T_SVD_matched_pts);
			//cout << "Feature Matched and Transformed point cloud " << i << endl;
			
			//transformedFeatureMatch_cloudrgbVec.push_back(transformedFM_cloudrgb1);
			//transformedFeatureMatch_cloudrgb_last = transformedFM_cloudrgb1;
			
			//generating the bigger point cloud
			//cloudrgb_FeatureMatched->insert(cloudrgb_FeatureMatched->end(),transformedFM_cloudrgb1->begin(),transformedFM_cloudrgb1->end());
		}
	}
	cout << "Completed calculating feature matched transformations." << endl;
	hexPosMAVLinkfile.close();
	hexPosFMfile.close();
	
	//transforming the camera positions using ICP
	pcl::registration::TransformationEstimation<pcl::PointXYZRGB, pcl::PointXYZRGB>::Matrix4 tf_icp = runICPalignment(cloud_hexPos_FM, cloud_hexPos_MAVLink);
	
	cout << "Creating point cloud." << endl;
	//pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudrgb_MAVLink(new pcl::PointCloud<pcl::PointXYZRGB> ());
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudrgb_FeatureMatched(new pcl::PointCloud<pcl::PointXYZRGB> ());
	//cloudrgb_MAVLink->is_dense = true;
	cloudrgb_FeatureMatched->is_dense = true;
	
	for (int i = 0; i < img_numbers.size(); i++)
	{
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudrgb(new pcl::PointCloud<pcl::PointXYZRGB> ());
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformed_cloudrgb( new pcl::PointCloud<pcl::PointXYZRGB>() );
		
		//createPtCloud(i, cloudrgb);
		createFeaturePtCloud(i, cloudrgb);
		//cout << "Created point cloud " << i << endl;
		
		pcl::registration::TransformationEstimation<pcl::PointXYZRGB, pcl::PointXYZRGB>::Matrix4 tf = tf_icp * t_FMVec[i];
		
		transformPtCloud(cloudrgb, transformed_cloudrgb, tf);
		cout << "Transformed point cloud " << i << endl;
		
		if (i == 0)
		{
			//generating the bigger point cloud
			copyPointCloud(*transformed_cloudrgb,*cloudrgb_FeatureMatched);
		}
		else
		{			
			//generating the bigger point cloud
			cloudrgb_FeatureMatched->insert(cloudrgb_FeatureMatched->end(),transformed_cloudrgb->begin(),transformed_cloudrgb->end());
		}
	}
	
	cout << "Finding 3D transformation, time: " << ((getTickCount() - t) / getTickFrequency()) << " sec" << endl;
	cout << "Finished Pose Estimation, total time: " << ((getTickCount() - app_start_time) / getTickFrequency()) << " sec" << endl;
	
	cout << "Saving point clouds..." << endl;
	//read_PLY_filename0 = "cloudrgb_MAVLink_" + currentDateTimeStr + ".ply";
	//save_pt_cloud_to_PLY_File(cloudrgb_MAVLink, read_PLY_filename0);
	
	read_PLY_filename1 = "cloudrgb_FeatureMatched_" + currentDateTimeStr + ".ply";
	save_pt_cloud_to_PLY_File(cloudrgb_FeatureMatched, read_PLY_filename1);
	
	//transforming the camera positions using ICP
	//pcl::registration::TransformationEstimation<pcl::PointXYZRGB, pcl::PointXYZRGB>::Matrix4 tf_icp = runICPalignment(cloud_hexPos_FM, cloud_hexPos_MAVLink);
	
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_hexPos_Fitted(new pcl::PointCloud<pcl::PointXYZRGB> ());
	transformPtCloud(cloud_hexPos_FM, cloud_hexPos_Fitted, tf_icp);
	hexPosFMFittedfile.open(hexPosFMFittedfilename, ios::out);
	for (int i = 0; i < cloud_hexPos_Fitted->points.size(); i++)
	{
		uint32_t rgbFitted = (uint32_t)255 << 16;	//blue
		cloud_hexPos_Fitted->points[i].rgb = *reinterpret_cast<float*>(&rgbFitted);
		hexPosFMFittedVec.push_back(cloud_hexPos_Fitted->points[i]);
		hexPosFMFittedfile << cloud_hexPos_Fitted->points[i].x << "," << cloud_hexPos_Fitted->points[i].y << "," << cloud_hexPos_Fitted->points[i].z << endl;
	}
	hexPosFMFittedfile.close();
	cloud_hexPos_Fitted->insert(cloud_hexPos_Fitted->end(),cloud_hexPos_FM->begin(),cloud_hexPos_FM->end());
	cloud_hexPos_Fitted->insert(cloud_hexPos_Fitted->end(),cloud_hexPos_MAVLink->begin(),cloud_hexPos_MAVLink->end());
	
	//rectifying Feature Matched Pt Cloud
	//cout << "rectifying Feature Matched Pt Cloud using ICP result..." << endl;
	//pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudrgb_FM_Fitted(new pcl::PointCloud<pcl::PointXYZRGB> ());
	//transformPtCloud(cloudrgb_FeatureMatched, cloudrgb_FM_Fitted, tf_icp);
	
	//downsampling
	cout << "downsampling..." << endl;
	//pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudrgb_MAVLink_downsamp = downsamplePtCloud(cloudrgb_MAVLink);
	//read_PLY_filename0 = "downsampled_" + read_PLY_filename0;
	//save_pt_cloud_to_PLY_File(cloudrgb_MAVLink_downsamp, read_PLY_filename0);
	
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudrgb_FeatureMatched_downsamp = downsamplePtCloud(cloudrgb_FeatureMatched);
	read_PLY_filename1 = "downsampled_rectified_" + read_PLY_filename1;
	save_pt_cloud_to_PLY_File(cloudrgb_FeatureMatched_downsamp, read_PLY_filename1);
	
	if(preview)
	{
		displayCamPositions = true;
		visualize_pt_cloud(cloud_hexPos_Fitted, "hexPos_Fitted red:MAVLink green:FeatureMatched blue:FM_Fitted");
		visualize_pt_cloud(cloudrgb_FeatureMatched_downsamp, "cloudrgb_FM_Fitted_downsampled");
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
			log_file << "img_index " << img_index << " src " << match.src_img_idx << " dst " << match.dst_img_idx << " confidence " << match.confidence << " inliers " << match.inliers_mask.size() << " matches " << match.matches.size() << endl;
			
			vector<KeyPoint> keypoints_src, keypoints_dst;
			keypoints_src = features[match.src_img_idx].keypoints;
			keypoints_dst = features[match.dst_img_idx].keypoints;
			
			Mat disp_img_src, disp_img_dst;
			if(use_segment_labels)
			{
				disp_img_src = double_disparity_images[match.src_img_idx];
				disp_img_dst = double_disparity_images[match.dst_img_idx];
				
			}
			else
			{
				disp_img_src = disparity_images[match.src_img_idx];
				disp_img_dst = disparity_images[match.dst_img_idx];
			}
			
			//define 3d points for all keypoints
			vector<Point3d> keypoints3D_src, keypoints3D_dst;
			vector<int> keypoints3D_2D_index_src, keypoints3D_2D_index_dst;
			
			//cout << "Converting 2D matches to 3D matches..." << endl;
			//cout << "match.inliers_mask.size() " << match.inliers_mask.size() << endl;
			
			for (int i = 0; i < match.inliers_mask.size(); i++)
			{
				if (match.inliers_mask[i] == 1 && match.matches[i].imgIdx != -1)
				{
					int trainIdx = match.matches[i].trainIdx;
					int queryIdx = match.matches[i].queryIdx;
					
					//*3. convert corresponding features to 3D using disparity image information
					double disp_val_src, disp_val_dst;
					if(use_segment_labels)
					{
						disp_val_src = disp_img_src.at<double>(keypoints_src[queryIdx].pt.y, keypoints_src[queryIdx].pt.x);
						disp_val_dst = disp_img_dst.at<double>(keypoints_dst[trainIdx].pt.y, keypoints_dst[trainIdx].pt.x);
					}
					else
					{
						disp_val_src = (double)disp_img_src.at<char>(keypoints_src[queryIdx].pt.y, keypoints_src[queryIdx].pt.x);
						disp_val_dst = (double)disp_img_dst.at<char>(keypoints_dst[trainIdx].pt.y, keypoints_dst[trainIdx].pt.x);
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
					}

				}
			}
			//log_file << "keypoints3D_src.size() " << keypoints3D_src.size() << endl;
			//cout << "keypoints3D_src.size() " << keypoints3D_src.size() << endl;
			
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
			//cout << "cloud_current_temp->size() " << cloud_current_temp->size() << endl;
			//cout << "cloud_prior_temp->size() " << cloud_prior_temp->size() << endl;
			pcl::registration::TransformationEstimation<pcl::PointXYZRGB, pcl::PointXYZRGB>::Matrix4 t_FM = t_FMVec[match.dst_img_idx];
			//cout << "t_FMVec[" << match.dst_img_idx << "]\n" << t_FM << endl;
			
			pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_current_t_temp (new pcl::PointCloud<pcl::PointXYZRGB> ());
			pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_prior_t_temp (new pcl::PointCloud<pcl::PointXYZRGB> ());
			
			pcl::transformPointCloud(*cloud_current_temp, *cloud_current_t_temp, t_mat_MAVLink);
			//cout << "cloud_current_temp transformed." << endl;
			pcl::transformPointCloud(*cloud_prior_temp, *cloud_prior_t_temp, t_FM);
			//cout << "cloud_prior_temp transformed." << endl;
			
			if (first_match)
			{
				copyPointCloud(*cloud_current_t_temp,*cloud_current);
				copyPointCloud(*cloud_prior_t_temp,*cloud_prior);
				first_match = false;
				//cout << "clouds copied!" << endl;
			}
			else
			{
				cloud_current->insert(cloud_current->end(),cloud_current_t_temp->begin(),cloud_current_t_temp->end());
				cloud_prior->insert(cloud_prior->end(),cloud_prior_t_temp->begin(),cloud_prior_t_temp->end());
				//cout << "clouds inserted!" << endl;
			}
		}
		if (match.src_img_idx >= img_index && match.dst_img_idx >= img_index)
			break;
	}
	
	//cout << "cloud_current->size() " << cloud_current->size() << endl;
	//cout << "cloud_prior->size() " << cloud_prior->size() << endl;
	
	//cout << "Finding Rigid Body Transformation..." << endl;
	
	pcl::registration::TransformationEstimationSVD<pcl::PointXYZRGB, pcl::PointXYZRGB> te2;
	pcl::registration::TransformationEstimation<pcl::PointXYZRGB, pcl::PointXYZRGB>::Matrix4 T_SVD_matched_pts;
	
	te2.estimateRigidTransformation(*cloud_current, *cloud_prior, T_SVD_matched_pts);
	//cout << "computed transformation between MATCHED KEYPOINTS T_SVD2 is\n" << T_SVD_matched_pts << endl;
	log_file << "computed transformation between MATCHED KEYPOINTS T_SVD2 is\n" << T_SVD_matched_pts << endl;
	
	return T_SVD_matched_pts;
}
