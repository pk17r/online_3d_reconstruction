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
#include <boost/filesystem.hpp>

Pose::Pose(int argc, char* argv[])
{
	currentDateTimeStr = currentDateTime();
	cout << "currentDateTime=" << currentDateTimeStr << "\n\n";
	
	//create directory
	folder = folder + currentDateTimeStr + "/";
	boost::filesystem::path dir(folder);
	if(boost::filesystem::create_directory(dir)) {
		cout << "Created save directory " << folder << endl;
	}
	else {
		cout << "Could not create save directory!" << folder << endl;
		return;
	}
	save_log_to = folder + "log.txt";
	
#if 0
	cv::setBreakOnError(true);
#endif
	if (parseCmdArgs(argc, argv) == -1) return;
	
	if (downsample)
	{
		//read cloud
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudrgb = read_PLY_File(read_PLY_filename0);
		
		//downsample cloud
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudrgb_filtered = downsamplePtCloud(cloudrgb);
		
		string writePath = "downsampled_" + read_PLY_filename0;
		save_pt_cloud_to_PLY_File(cloudrgb_filtered, writePath);
		
		pcl::PolygonMesh mesh;
		visualize_pt_cloud(true, cloudrgb_filtered, false, mesh, "Downsampled Point Cloud");
		
		cout << "Cya!" << endl;
		return;
	}
	
	if (smooth_surface)
	{
		void smoothPtCloud();
		return;
	}
	
	if(mesh_surface)
	{
		void meshSurface();
		return;
	}
	
	if (visualize)
	{
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudrgb = read_PLY_File(read_PLY_filename0);
		pcl::PolygonMesh mesh;
		visualize_pt_cloud(true, cloudrgb, false, mesh, read_PLY_filename0);
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
		
		pcl::PolygonMesh mesh;
		visualize_pt_cloud(true, cloud_Fitted, false, mesh, writePath);
		
		return;
	}
	
	readCalibFile();
	readPoseFile();
	populateData();
	
	//start program
	int64 app_start_time = getTickCount();
	
	//initialize some variables
	finder = makePtr<OrbFeaturesFinder>();
	features = vector<ImageFeatures>(img_numbers.size());
	
	//main point clouds
	//pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudrgb_MAVLink (new pcl::PointCloud<pcl::PointXYZRGB> ());
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudrgb_FeatureMatched (new pcl::PointCloud<pcl::PointXYZRGB> ());
	//cloudrgb_MAVLink->is_dense = true;
	cloudrgb_FeatureMatched->is_dense = true;
	
	//vectors to store transformations of point clouds
	vector<pcl::registration::TransformationEstimation<pcl::PointXYZRGB, pcl::PointXYZRGB>::Matrix4> t_matVec;
	vector<pcl::registration::TransformationEstimation<pcl::PointXYZRGB, pcl::PointXYZRGB>::Matrix4> t_FMVec;
	
	//cloud_hexPos will store combined MAVLink and FM_Fitted positions
	//pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_hexPos(new pcl::PointCloud<pcl::PointXYZRGB> ());
	//cloud_hexPos->is_dense = true;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_hexPos_MAVLink(new pcl::PointCloud<pcl::PointXYZRGB> ());
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_hexPos_FM(new pcl::PointCloud<pcl::PointXYZRGB> ());
	cloud_hexPos_MAVLink->is_dense = true;
	cloud_hexPos_FM->is_dense = true;
	
	int n_cycle;
	if(seq_len > 0)
		n_cycle = ceil(1.0 * img_numbers.size() / seq_len);
	else
		n_cycle = 1;
	
	boost::thread the_visualization_thread;
	
	for (int cycle = 0; cycle < n_cycle; cycle++)
	{
		int64 t = getTickCount();
		
		start_idx = cycle * seq_len;
		
		if(seq_len > 0)
			end_idx = min((cycle + 1) * seq_len - 1, (int)(img_numbers.size()) - 1);
		else
			end_idx = img_numbers.size() - 1;
		
		cout << "\nImages " << start_idx << " to " << end_idx << endl;
		log_file << "\nImages " << start_idx << " to " << end_idx << endl;
		findFeatures();
		cout << "Finding features, time: " << ((getTickCount() - t) / getTickFrequency()) << " sec" << endl;
		log_file << "Finding features, time: " << ((getTickCount() - t) / getTickFrequency()) << " sec" << endl;

		for (int i = start_idx; i < end_idx + 1; i++)
		{
			//SEARCH PROCESS: get NSECS from images_times_data and search for corresponding or nearby entry in pose_data and heading_data
			int pose_index = data_index_finder(img_numbers[i]);
			pcl::PointXYZRGB hexPosMAVLink = addPointFromPoseFile(pose_index);
			cloud_hexPos_MAVLink->points.push_back(hexPosMAVLink);
			
			pcl::registration::TransformationEstimation<pcl::PointXYZRGB, pcl::PointXYZRGB>::Matrix4 t_mat = generateTmat(pose_data[pose_index]);
			t_matVec.push_back(t_mat);
			
			if (i == 0)
			{
				t_FMVec.push_back(t_mat);
				continue;
			}
			
			//Feature Matching Alignment
			//generate point clouds of matched keypoints and estimate rigid body transform between them
			pcl::registration::TransformationEstimation<pcl::PointXYZRGB, pcl::PointXYZRGB>::Matrix4 T_SVD_matched_pts = generate_tf_of_Matched_Keypoints_Point_Cloud(i, t_FMVec, t_mat);
			
			t_FMVec.push_back(T_SVD_matched_pts * t_mat);
			
			pcl::PointXYZRGB hexPosFM = transformPoint(hexPosMAVLink, T_SVD_matched_pts);
			cloud_hexPos_FM->points.push_back(hexPosFM);
		}
		cout << "Completed calculating feature matched transformations." << endl;
		log_file << "Completed calculating feature matched transformations." << endl;
		
		//transforming the camera positions using ICP
		pcl::registration::TransformationEstimation<pcl::PointXYZRGB, pcl::PointXYZRGB>::Matrix4 tf_icp = runICPalignment(cloud_hexPos_FM, cloud_hexPos_MAVLink);
		
		//correcting old point cloud
		transformPtCloud(cloudrgb_FeatureMatched, cloudrgb_FeatureMatched, tf_icp);
		
		//correcting old tf_mats
		for (int i = 0; i < end_idx + 1; i++)
			t_FMVec[i] = tf_icp * t_FMVec[i];
		
		//fit FM camera positions to MAVLink camera positions using ICP and use the tf to correct point cloud
		transformPtCloud(cloud_hexPos_FM, cloud_hexPos_FM, tf_icp);
		
		//adding new points to point cloud
		cout << "Adding Point Cloud number/points ";
		log_file << "Adding Point Cloud number/points ";
		for (int i = start_idx; i < end_idx + 1; i++)
		{
			pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudrgb (new pcl::PointCloud<pcl::PointXYZRGB> ());
			pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformed_cloudrgb ( new pcl::PointCloud<pcl::PointXYZRGB>() );
			
			if(jump_pixels == 1)
				createPtCloud(i, cloudrgb);
			else
				createFeaturePtCloud(i, cloudrgb);
			//cout << "Created point cloud " << i << endl;
			
			transformPtCloud(cloudrgb, transformed_cloudrgb, t_FMVec[i]);
			
			//generating the bigger point cloud
			if (i == 0)
				copyPointCloud(*transformed_cloudrgb,*cloudrgb_FeatureMatched);
			else
				cloudrgb_FeatureMatched->insert(cloudrgb_FeatureMatched->end(),transformed_cloudrgb->begin(),transformed_cloudrgb->end());
			//cout << "Transformed and added." << endl;
		}
		log_file << endl;
		
		cout << "\nCycle time: " << ((getTickCount() - t) / getTickFrequency()) << " sec" << endl;
		log_file << "\nCycle time: " << ((getTickCount() - t) / getTickFrequency()) << " sec" << endl;
		
		//visualize
		if(preview)
		{
			pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudrgb_FeatureMatched_copy (new pcl::PointCloud<pcl::PointXYZRGB>());
			copyPointCloud(*cloudrgb_FeatureMatched, *cloudrgb_FeatureMatched_copy);
			
			if(cycle > 0)
				the_visualization_thread.join();
			
			the_visualization_thread = boost::thread(&Pose::displayPointCloudOnline, this, cloudrgb_FeatureMatched_copy, cloud_hexPos_FM, cloud_hexPos_MAVLink, cycle);
		}
	}
	
	cout << "\nFinished Pose Estimation, total time: " << ((getTickCount() - app_start_time) / getTickFrequency()) << " sec" << endl;
	log_file << "\nFinished Pose Estimation, total time: " << ((getTickCount() - app_start_time) / getTickFrequency()) << " sec" << endl;
	
	cout << "Saving point clouds..." << endl;
	cout << "Saving point clouds..." << endl;
	//read_PLY_filename0 = "cloudrgb_MAVLink_" + currentDateTimeStr + ".ply";
	//save_pt_cloud_to_PLY_File(cloudrgb_MAVLink, read_PLY_filename0);
	read_PLY_filename1 = folder + "cloud.ply";
	save_pt_cloud_to_PLY_File(cloudrgb_FeatureMatched, read_PLY_filename1);
	
	//downsampling
	cout << "downsampling..." << endl;
	log_file << "downsampling..." << endl;
	//pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudrgb_MAVLink_downsamp = downsamplePtCloud(cloudrgb_MAVLink);
	//read_PLY_filename0 = "downsampled_" + read_PLY_filename0;
	//save_pt_cloud_to_PLY_File(cloudrgb_MAVLink_downsamp, read_PLY_filename0);
	
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudrgb_FeatureMatched_downsamp = downsamplePtCloud(cloudrgb_FeatureMatched);
	//cloudrgb_FeatureMatched_downsamp = downsamplePtCloud(cloudrgb_FeatureMatched);
	read_PLY_filename1 = folder + "cloud_downsampled.ply";
	save_pt_cloud_to_PLY_File(cloudrgb_FeatureMatched_downsamp, read_PLY_filename1);
	
	cloud_hexPos_FM->insert(cloud_hexPos_FM->end(),cloud_hexPos_MAVLink->begin(),cloud_hexPos_MAVLink->end());
	string hexpos_filename = folder + "cloud_hexpos.ply";
	save_pt_cloud_to_PLY_File(cloud_hexPos_FM, hexpos_filename);
	
	//save settings for future reference
	string settings_filename = folder + "sttings.xml";
	cout << settings_filename << endl; 
	FileStorage fs(settings_filename, FileStorage::WRITE);
	fs << "range_width"  << range_width
	   << "voxel_size" << voxel_size
	   << "jump_pixels" << jump_pixels
	   << "focallength" << focallength
	   << "baseline" << baseline
	;
	fs.release();	// close Settings file
	log_file.close();
	
	if(preview)
		while (!viewer_online->wasStopped ()) { // Display the visualiser until 'q' key is pressed
			viewer_online->spinOnce();
		}
	//if(preview)
	//{
	//	wait_at_visualizer = true;
	//	displayCamPositions = true;
	//	hexPos_cloud = cloud_hexPos_FM;
	//	pcl::PolygonMesh mesh;
	//	//visualize_pt_cloud(true, cloud_hexPos_FM, false, mesh, "cloud_hexPos red:MAVLink green:FeatureMatched");
	//	visualize_pt_cloud(true, cloudrgb_FeatureMatched_downsamp, false, mesh, "cloudrgb_FM_Fitted_downsampled");
	//}
	
}

void Pose::displayPointCloudOnline(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudrgb_FeatureMatched_copy, 
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_hexPos_FM, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_hexPos_MAVLink, int cycle)
{
	wait_at_visualizer = false;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudrgb_FM_Fitted_downsampled_Online = downsamplePtCloud(cloudrgb_FeatureMatched_copy);
	displayCamPositions = true;
	pcl::PolygonMesh mesh;
	//visualize_pt_cloud(true, cloudrgb_FeatureMatched_downsampA, false, mesh, "cloudrgb_FM_Fitted_downsampledA");
	
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr hexPos_cloud_online (new pcl::PointCloud<pcl::PointXYZRGB>());
	hexPos_cloud_online->insert(hexPos_cloud_online->begin(),cloud_hexPos_FM->begin(),cloud_hexPos_FM->end());
	hexPos_cloud_online->insert(hexPos_cloud_online->end(),cloud_hexPos_MAVLink->begin(),cloud_hexPos_MAVLink->end());
	hexPos_cloud = hexPos_cloud_online;
	if (cycle == 0)
	{
		viewer_online = visualize_pt_cloud(true, cloudrgb_FM_Fitted_downsampled_Online, false, mesh, "cloudrgb_FM_Fitted_downsampled_Online");
	}
	else
	{
		visualize_pt_cloud_update(cloudrgb_FM_Fitted_downsampled_Online, "cloudrgb_FM_Fitted_downsampled_Online", viewer_online);
	}
}

pcl::registration::TransformationEstimation<pcl::PointXYZRGB, pcl::PointXYZRGB>::Matrix4 Pose::generate_tf_of_Matched_Keypoints_Point_Cloud
(int img_index, vector<pcl::registration::TransformationEstimation<pcl::PointXYZRGB, pcl::PointXYZRGB>::Matrix4> t_FMVec, 
pcl::registration::TransformationEstimation<pcl::PointXYZRGB, pcl::PointXYZRGB>::Matrix4 t_mat_MAVLink)
{
	cout << "matching img " << img_index << " with_img/matches";
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_current (new pcl::PointCloud<pcl::PointXYZRGB> ());
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_prior (new pcl::PointCloud<pcl::PointXYZRGB> ());
	cloud_current->is_dense = true;
	cloud_prior->is_dense = true;
	bool first_match = true;
	
	Mat disp_img_src;
	if(use_segment_labels)
		disp_img_src = double_disparity_images[img_index];
	else
		disp_img_src = disparity_images[img_index];
	//cout << "Read source disparity image." << endl;
	
	vector<KeyPoint> keypoints_src = features[img_index].keypoints;
	cuda::GpuMat descriptor_src(features[img_index].descriptors);
	//cout << "Read source keypoints." << endl;
	
	for (int dst_index = img_index-1; dst_index >= max(img_index - range_width,0); dst_index--)
	{
		//reference https://stackoverflow.com/questions/44988087/opencv-feature-matching-match-descriptors-to-knn-filtered-keypoints
		//reference https://github.com/opencv/opencv/issues/6130
		//reference http://study.marearts.com/2014/07/opencv-study-orb-gpu-feature-extraction.html
		//reference https://docs.opencv.org/3.1.0/d6/d1d/group__cudafeatures2d.html
		
		//cout << "image " << img_index << " to " << dst_index << endl;
		vector<vector<DMatch>> matches;
		cuda::GpuMat descriptor_dst(features[dst_index].descriptors);
		matcher->knnMatch(descriptor_src, descriptor_dst, matches, 2);
		vector<DMatch> good_matches;
		for(int k = 0; k < matches.size(); k++)
		{
			if(matches[k][0].distance < 0.5 * matches[k][1].distance && matches[k][0].distance < 40)
			{
				//cout << matches[k][0].distance << "/" << matches[k][1].distance << " " << 
				//matches[k][0].imgIdx << "/" << matches[k][1].imgIdx << " " << 
				//matches[k][0].queryIdx << "/" << matches[k][1].queryIdx << " " << 
				//matches[k][0].trainIdx << "/" << matches[k][1].trainIdx << endl;
				good_matches.push_back(matches[k][0]);
			}
		}
		if(good_matches.size() < 100)	//less number of matches.. don't bother working on this one. good matches are around 500-600
			continue;
		
		cout << " " << dst_index << "/" << good_matches.size();
		
		Mat disp_img_dst;
		if(use_segment_labels)
			disp_img_dst = double_disparity_images[dst_index];
		else
			disp_img_dst = disparity_images[dst_index];
		//cout << "Read destination disparity image." << endl;
		
		vector<KeyPoint> keypoints_dst = features[dst_index].keypoints;
		//cout << "Read destination keypoints." << endl;
		//using sequential matched points to estimate the rigid body transformation between matched 3D points
		for (int match_index = 0; match_index < good_matches.size(); match_index++)
		{
			DMatch match = good_matches[match_index];
			
			//define 3d points for all keypoints
			vector<Point3d> keypoints3D_src, keypoints3D_dst;
			vector<int> keypoints3D_2D_index_src, keypoints3D_2D_index_dst;
			
			//cout << "Converting 2D matches to 3D matches... match.trainIdx " << match.trainIdx << " match.queryIdx " << match.queryIdx << endl;
			int trainIdx = match.trainIdx;
			int queryIdx = match.queryIdx;
			
			//*3. convert corresponding features to 3D using disparity image information
			//cout << "keypoints_src[queryIdx].pt.y " << keypoints_src[queryIdx].pt.y << " keypoints_src[queryIdx].pt.x " << keypoints_src[queryIdx].pt.x << endl;
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
			//cout << "Read disparity value." << endl;
		
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
			pcl::registration::TransformationEstimation<pcl::PointXYZRGB, pcl::PointXYZRGB>::Matrix4 t_FM = t_FMVec[dst_index];
			//cout << "t_FMVec[" << dst_index << "]\n" << t_FM << endl;
			
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
		
	}
	
	cout << endl;
	
	//cout << "cloud_current->size() " << cloud_current->size() << endl;
	//cout << "cloud_prior->size() " << cloud_prior->size() << endl;
	
	//cout << "Finding Rigid Body Transformation..." << endl;
	
	pcl::registration::TransformationEstimationSVD<pcl::PointXYZRGB, pcl::PointXYZRGB> te2;
	pcl::registration::TransformationEstimation<pcl::PointXYZRGB, pcl::PointXYZRGB>::Matrix4 T_SVD_matched_pts;
	
	te2.estimateRigidTransformation(*cloud_current, *cloud_prior, T_SVD_matched_pts);
	//cout << "computed transformation between MATCHED KEYPOINTS T_SVD2 is\n" << T_SVD_matched_pts << endl;
	//log_file << "computed transformation between MATCHED KEYPOINTS T_SVD2 is\n" << T_SVD_matched_pts << endl;
	
	return T_SVD_matched_pts;
}
