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
	if (parseCmdArgs(argc, argv) == -1) return;
	
	if (visualize)
	{
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudrgb = read_PLY_File(read_PLY_filename0);
		if(displayUAVPositions)
			hexPos_cloud = read_PLY_File(read_PLY_filename1);
		pcl::PolygonMesh mesh;
		//visualize_pt_cloud(true, cloudrgb, false, mesh, read_PLY_filename0);
		visualize_pt_cloud(cloudrgb, read_PLY_filename0);
		return;
	}
	
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
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudrgb_FeatureMatched_big (new pcl::PointCloud<pcl::PointXYZRGB> ());
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_downsampled (new pcl::PointCloud<pcl::PointXYZRGB> ());
	cloud_downsampled->is_dense = true;
	//cloudrgb_MAVLink->is_dense = true;
	cloudrgb_FeatureMatched_big->is_dense = true;
	
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
		int64 t0 = getTickCount();
		
		start_idx = cycle * seq_len;
		
		if(seq_len > 0)
			end_idx = min((cycle + 1) * seq_len - 1, (int)(img_numbers.size()) - 1);
		else
			end_idx = img_numbers.size() - 1;
		
		cout << "\nImages " << start_idx << " to " << end_idx << endl;
		log_file << "\nImages " << start_idx << " to " << end_idx << endl;
		findFeatures();
		int64 t1 = getTickCount();
		cout << "\nFinding features, time: " << (t1 - t0) / getTickFrequency() << " sec\n" << endl;
		log_file << "\nFinding features, time: " << (t1 - t0) / getTickFrequency() << " sec\n" << endl;

		log_file << "\nrecorded hexacopter positions" << endl;
		for (int i = start_idx; i < end_idx + 1; i++)
		{
			//SEARCH PROCESS: get NSECS from images_times_data and search for corresponding or nearby entry in pose_data and heading_data
			int pose_index = data_index_finder(img_numbers[i]);
			pcl::PointXYZRGB hexPosMAVLink = addPointFromPoseFile(pose_index);
			cloud_hexPos_MAVLink->points.push_back(hexPosMAVLink);
			log_file << img_numbers[i] << "," << pose_data[pose_index][tx_ind] << "," << pose_data[pose_index][ty_ind] << "," << pose_data[pose_index][tz_ind] << endl;
			
			pcl::registration::TransformationEstimation<pcl::PointXYZRGB, pcl::PointXYZRGB>::Matrix4 t_mat = generateTmat(pose_data[pose_index]);
			t_matVec.push_back(t_mat);
			
			if (i == 0)
			{
				t_FMVec.push_back(t_mat);
				cloud_hexPos_FM->points.push_back(hexPosMAVLink);
				continue;
			}
			
			//Feature Matching Alignment
			//generate point clouds of matched keypoints and estimate rigid body transform between them
			pcl::registration::TransformationEstimation<pcl::PointXYZRGB, pcl::PointXYZRGB>::Matrix4 T_SVD_matched_pts = generate_tf_of_Matched_Keypoints_Point_Cloud(i, t_FMVec, t_mat);
			
			t_FMVec.push_back(T_SVD_matched_pts * t_mat);
			
			pcl::PointXYZRGB hexPosFM = transformPoint(hexPosMAVLink, T_SVD_matched_pts);
			cloud_hexPos_FM->points.push_back(hexPosFM);
		}
		int64 t2 = getTickCount();
		cout << "\nMatching features and finding transformations time: " << (t2 - t1) / getTickFrequency() << " sec\n" << endl;
		log_file << "\nMatching features and finding transformations time: " << (t2 - t1) / getTickFrequency() << " sec\n" << endl;
		
		//cout << "cloud_hexPos_FM: ";
		//findNormalOfPtCloud(cloud_hexPos_FM);
		//cout << "cloud_hexPos_MAVLink: ";
		//findNormalOfPtCloud(cloud_hexPos_MAVLink);
		
		log_file << "\nfeature matched hexacopter positions" << endl;
		for (int i = start_idx; i < end_idx + 1; i++)
			log_file << img_numbers[i] << "," << cloud_hexPos_FM->points[i].x << "," << cloud_hexPos_FM->points[i].y << "," << cloud_hexPos_FM->points[i].z << endl;
		
		//transforming the camera positions using ICP
		pcl::registration::TransformationEstimation<pcl::PointXYZRGB, pcl::PointXYZRGB>::Matrix4 tf_icp = runICPalignment(cloud_hexPos_FM, cloud_hexPos_MAVLink);
		
		//correcting old point cloud
		//transformPtCloud(cloudrgb_FeatureMatched, cloudrgb_FeatureMatched, tf_icp);
		transformPtCloud(cloud_downsampled, cloud_downsampled, tf_icp);
		
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudrgb_FeatureMatched (new pcl::PointCloud<pcl::PointXYZRGB> ());
		
		//correcting old tf_mats
		for (int i = 0; i < end_idx + 1; i++)
			t_FMVec[i] = tf_icp * t_FMVec[i];
		
		//fit FM camera positions to MAVLink camera positions using ICP and use the tf to correct point cloud
		transformPtCloud(cloud_hexPos_FM, cloud_hexPos_FM, tf_icp);
		
		log_file << "\ncorrected hexacopter positions" << endl;
		for (int i = start_idx; i < end_idx + 1; i++)
			log_file << img_numbers[i] << "," << cloud_hexPos_FM->points[i].x << "," << cloud_hexPos_FM->points[i].y << "," << cloud_hexPos_FM->points[i].z << endl;
		
		int64 t3 = getTickCount();
		cout << "\nICP alignment and point cloud correction time: " << (t3 - t2) / getTickFrequency() << " sec\n" << endl;
		log_file << "\nICP alignment and point cloud correction time: " << (t3 - t2) / getTickFrequency() << " sec\n" << endl;
		
		//adding new points to point cloud
		cout << "Adding Point Cloud number/points ";
		log_file << "Adding Point Cloud number/points ";
		//for (int i = start_idx; i < end_idx + 1; i++)
		voxel_size /= 10;
		int i = start_idx;
		while(i < end_idx + 1)
		{
			int i0 = i;
			pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformed_cloudrgb1 ( new pcl::PointCloud<pcl::PointXYZRGB>() );
			pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformed_cloudrgb2 ( new pcl::PointCloud<pcl::PointXYZRGB>() );
			pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformed_cloudrgb3 ( new pcl::PointCloud<pcl::PointXYZRGB>() );
			pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformed_cloudrgb4 ( new pcl::PointCloud<pcl::PointXYZRGB>() );
			pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformed_cloudrgb5 ( new pcl::PointCloud<pcl::PointXYZRGB>() );
			pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformed_cloudrgb6 ( new pcl::PointCloud<pcl::PointXYZRGB>() );
			pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformed_cloudrgb7 ( new pcl::PointCloud<pcl::PointXYZRGB>() );
			
			boost::thread pt_cloud_thread1, pt_cloud_thread2, pt_cloud_thread3, pt_cloud_thread4, pt_cloud_thread5, pt_cloud_thread6, pt_cloud_thread7;
			
			pt_cloud_thread1 = boost::thread(&Pose::createAndTransformPtCloud, this, i, t_FMVec, transformed_cloudrgb1);
			if(++i < end_idx + 1) pt_cloud_thread2 = boost::thread(&Pose::createAndTransformPtCloud, this, i, t_FMVec, transformed_cloudrgb2);
			if(++i < end_idx + 1) pt_cloud_thread3 = boost::thread(&Pose::createAndTransformPtCloud, this, i, t_FMVec, transformed_cloudrgb3);
			if(++i < end_idx + 1) pt_cloud_thread4 = boost::thread(&Pose::createAndTransformPtCloud, this, i, t_FMVec, transformed_cloudrgb4);
			if(++i < end_idx + 1) pt_cloud_thread5 = boost::thread(&Pose::createAndTransformPtCloud, this, i, t_FMVec, transformed_cloudrgb5);
			if(++i < end_idx + 1) pt_cloud_thread6 = boost::thread(&Pose::createAndTransformPtCloud, this, i, t_FMVec, transformed_cloudrgb6);
			if(++i < end_idx + 1) pt_cloud_thread7 = boost::thread(&Pose::createAndTransformPtCloud, this, i, t_FMVec, transformed_cloudrgb7);
			
			
			//generating the bigger point cloud
			pt_cloud_thread1.join();
			i = i0;
			if (i == 0)
				copyPointCloud(*transformed_cloudrgb1,*cloudrgb_FeatureMatched);
			else
				cloudrgb_FeatureMatched->insert(cloudrgb_FeatureMatched->end(),transformed_cloudrgb1->begin(),transformed_cloudrgb1->end());
			pt_cloud_thread2.join();
			if(++i < end_idx + 1) cloudrgb_FeatureMatched->insert(cloudrgb_FeatureMatched->end(),transformed_cloudrgb2->begin(),transformed_cloudrgb2->end());
			pt_cloud_thread3.join();
			if(++i < end_idx + 1) cloudrgb_FeatureMatched->insert(cloudrgb_FeatureMatched->end(),transformed_cloudrgb3->begin(),transformed_cloudrgb3->end());
			pt_cloud_thread4.join();
			if(++i < end_idx + 1) cloudrgb_FeatureMatched->insert(cloudrgb_FeatureMatched->end(),transformed_cloudrgb4->begin(),transformed_cloudrgb4->end());
			pt_cloud_thread5.join();
			if(++i < end_idx + 1) cloudrgb_FeatureMatched->insert(cloudrgb_FeatureMatched->end(),transformed_cloudrgb5->begin(),transformed_cloudrgb5->end());
			pt_cloud_thread6.join();
			if(++i < end_idx + 1) cloudrgb_FeatureMatched->insert(cloudrgb_FeatureMatched->end(),transformed_cloudrgb6->begin(),transformed_cloudrgb6->end());
			pt_cloud_thread7.join();
			if(++i < end_idx + 1) cloudrgb_FeatureMatched->insert(cloudrgb_FeatureMatched->end(),transformed_cloudrgb7->begin(),transformed_cloudrgb7->end());
			//cout << "Transformed and added." << endl;
		}
		voxel_size *= 10;
		log_file << endl;
		
		int64 t4 = getTickCount();
		cout << "\n\nPoint Cloud Creation time: " << (t4 - t3) / getTickFrequency() << " sec" << endl;
		log_file << "\n\nPoint Cloud Creation time: " << (t4 - t3) / getTickFrequency() << " sec" << endl;
		
		//downsample
		wait_at_visualizer = false;
		cout << "downsampling..." << endl;
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudrgb_FM_Fitted_downsampled_Online = downsamplePtCloud(cloudrgb_FeatureMatched);
		//adding the new downsampled points to old downsampled cloud
		cloud_downsampled->insert(cloud_downsampled->end(),cloudrgb_FM_Fitted_downsampled_Online->begin(),cloudrgb_FM_Fitted_downsampled_Online->end());
		cout << "downsampled." << endl;
		//downsampling again to clean joining areas
		cloud_downsampled = downsamplePtCloud(cloud_downsampled);
		cout << "clouds combined and downsampled again to clean edges." << endl;
		
		int64 t5 = getTickCount();
		
		cout << "\nDownsampling time: " << (t5 - t4) / getTickFrequency() << " sec" << endl;
		log_file << "\nDownsampling time: " << (t5 - t4) / getTickFrequency() << " sec" << endl;
		
		//visualize
		if(preview)
		{		
			pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_downsampled_copy (new pcl::PointCloud<pcl::PointXYZRGB>());
			copyPointCloud(*cloud_downsampled, *cloud_downsampled_copy);
			
			if(cycle > 0)
				the_visualization_thread.join();
			
			the_visualization_thread = boost::thread(&Pose::displayPointCloudOnline, this, cloud_downsampled_copy, cloud_hexPos_FM, cloud_hexPos_MAVLink, cycle, n_cycle);
		}
		
		int64 t6 = getTickCount();
		
		cout << "\nCycle time: " << (t6 - t0) / getTickFrequency() << " sec" << endl;
		log_file << "\nCycle time: " << (t6 - t0) / getTickFrequency() << " sec" << endl;
		
		if(seq_len == -1)
		{
			copyPointCloud(*cloudrgb_FeatureMatched,*cloudrgb_FeatureMatched_big);
			cout << "cloudrgb_FeatureMatched_big->size() " << cloudrgb_FeatureMatched_big->size() << endl;
		}
	}
	
	int64 tend = getTickCount();
	
	cout << "\nFinished Pose Estimation, total time: " << ((tend - app_start_time) / getTickFrequency()) << " sec for total of " 
		<< img_numbers.size() << " images with jump_pixels=" << jump_pixels << " and online visualization every " << seq_len << " images at " 
		<< 1.0*img_numbers.size()/((tend - app_start_time) / getTickFrequency()) << " fps\n" << endl;
	log_file << "\nFinished Pose Estimation, total time: " << ((tend - app_start_time) / getTickFrequency()) << " sec for total of " 
		<< img_numbers.size() << " images with jump_pixels=" << jump_pixels << " and online visualization every " << seq_len << " images at " 
		<< 1.0*img_numbers.size()/((tend - app_start_time) / getTickFrequency()) << " fps\n" << endl;
	
	cout << "Saving point clouds..." << endl;
	log_file << "Saving point clouds..." << endl;
	read_PLY_filename0 = folder + "cloud.ply";
	save_pt_cloud_to_PLY_File(cloud_downsampled, read_PLY_filename0);
	//read_PLY_filename0 = "cloudrgb_MAVLink_" + currentDateTimeStr + ".ply";
	//save_pt_cloud_to_PLY_File(cloudrgb_MAVLink, read_PLY_filename0);
	//read_PLY_filename1 = folder + "cloud_big.ply";
	//if(seq_len == -1)
	//	save_pt_cloud_to_PLY_File(cloudrgb_FeatureMatched_big, read_PLY_filename1);
	//boost::thread t1(&Pose::save_pt_cloud_to_PLY_File, this, cloudrgb_FeatureMatched, read_PLY_filename1);
	
	//downsampling
	//cout << "downsampling..." << endl;
	//log_file << "downsampling..." << endl;
	//pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudrgb_MAVLink_downsamp = downsamplePtCloud(cloudrgb_MAVLink);
	//read_PLY_filename0 = "downsampled_" + read_PLY_filename0;
	//save_pt_cloud_to_PLY_File(cloudrgb_MAVLink_downsamp, read_PLY_filename0);
	
	//pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudrgb_FeatureMatched_downsamp = downsamplePtCloud(cloudrgb_FeatureMatched);
	//cloudrgb_FeatureMatched_downsamp = downsamplePtCloud(cloudrgb_FeatureMatched);
	//read_PLY_filename1 = folder + "cloud_downsampled.ply";
	//save_pt_cloud_to_PLY_File(cloudrgb_FeatureMatched_downsamp, read_PLY_filename1);
	
	cloud_hexPos_FM->insert(cloud_hexPos_FM->end(),cloud_hexPos_MAVLink->begin(),cloud_hexPos_MAVLink->end());
	string hexpos_filename = folder + "cloud_uavpos.ply";
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
	//if(seq_len == -1)
	//	t1.join();
	
	if(preview)
		the_visualization_thread.join();
}

void Pose::findNormalOfPtCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud)
{
	// Create the normal estimation class, and pass the input dataset to it
	pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> ne;
	ne.setInputCloud (cloud);

	// Create an empty kdtree representation, and pass it to the normal estimation object.
	// Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
	pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB> ());
	ne.setSearchMethod (tree);

	// Output datasets
	pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);

	// Use all neighbors in a sphere of radius 3cm
	ne.setRadiusSearch (1000);

	// Compute the features
	ne.compute (*cloud_normals);

	// cloud_normals->points.size () should have the same size as the input cloud->points.size ()*
	cout << "cloud_normals->size() " << cloud_normals->size() << endl;
	for (int i = 0; i < cloud_normals->size(); i++)
	{
		cout << cloud_normals->points[i].normal_x << " " << cloud_normals->points[i].normal_y << " " << cloud_normals->points[i].normal_z << " " << endl;
	}
	
}

void Pose::createAndTransformPtCloud(int img_index, 
	vector<pcl::registration::TransformationEstimation<pcl::PointXYZRGB, pcl::PointXYZRGB>::Matrix4> t_FMVec, 
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr &downsampled_cloudrgb_return)
{
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudrgb (new pcl::PointCloud<pcl::PointXYZRGB> ());
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudrgb_transformed (new pcl::PointCloud<pcl::PointXYZRGB> ());
	
	if(jump_pixels == 1)
		createPtCloud(img_index, cloudrgb);
	else
		createFeaturePtCloud(img_index, cloudrgb);
	//cout << "Created point cloud " << i << endl;
	
	transformPtCloud(cloudrgb, cloudrgb_transformed, t_FMVec[img_index]);
	
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudrgb_downsampled = downsamplePtCloud(cloudrgb_transformed);
	
	copyPointCloud(*cloudrgb_downsampled, *downsampled_cloudrgb_return);
}

void Pose::displayPointCloudOnline(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudrgb_FM_Fitted_downsampled_Online, 
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_hexPos_FM, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_hexPos_MAVLink, int cycle, int n_cycle)
{
	wait_at_visualizer = false;
	//pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudrgb_FM_Fitted_downsampled_Online = downsamplePtCloud(cloudrgb_FeatureMatched_copy);
	
	displayUAVPositions = true;
	pcl::PolygonMesh mesh;
	//visualize_pt_cloud(true, cloudrgb_FeatureMatched_downsampA, false, mesh, "cloudrgb_FM_Fitted_downsampledA");
	
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr hexPos_cloud_online (new pcl::PointCloud<pcl::PointXYZRGB>());
	hexPos_cloud_online->insert(hexPos_cloud_online->begin(),cloud_hexPos_FM->begin(),cloud_hexPos_FM->end());
	hexPos_cloud_online->insert(hexPos_cloud_online->end(),cloud_hexPos_MAVLink->begin(),cloud_hexPos_MAVLink->end());
	hexPos_cloud = hexPos_cloud_online;
	if (cycle == 0)
	{
		viewer_online = visualize_pt_cloud(true, cloudrgb_FM_Fitted_downsampled_Online, false, mesh, "cloudrgb_visualization_Online");
	}
	else
	{
		visualize_pt_cloud_update(cloudrgb_FM_Fitted_downsampled_Online, "cloudrgb_visualization_Online", viewer_online);
	}
	if(cycle == n_cycle -1)
	{
		while (!viewer_online->wasStopped ()) { // Display the visualiser until 'q' key is pressed
			viewer_online->spinOnce();
		}
	}
}

