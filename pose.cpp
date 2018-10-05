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
#include <execinfo.h>
#include <signal.h>
#include <ucontext.h>

#include "pose.h"
#include <boost/filesystem.hpp>
#include "pose_functions.cpp"

#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_line.h>

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
	
	if (segment_map_only)
	{
		cout << "inside segment_map_only" << endl;
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudrgb = read_PLY_File(read_PLY_filename0);
		segmentCloud(cloudrgb);
		return;
	}
	
	if (smooth_surface)
	{
		smoothPtCloud();
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
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudrgb_filtered = downsamplePtCloud(cloudrgb, true);
		
		string writePath = "downsampled_" + read_PLY_filename0;
		save_pt_cloud_to_PLY_File(cloudrgb_filtered, writePath);
		
		pcl::PolygonMesh mesh;
		visualize_pt_cloud(true, cloudrgb_filtered, false, mesh, "Downsampled Point Cloud");
		
		cout << "Cya!" << endl;
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
	
	if(!run3d_reconstruction)
		return;
	
	populateData();
	
	//checks
	if(rows == 0 || cols == 0 || cols_start_aft_cutout == 0)
		throw "Exception: some important values not set! rows " + to_string(rows) + " cols " + to_string(cols) + " cols_start_aft_cutout " + to_string(cols_start_aft_cutout);
	
	//start program
	int64 app_start_time = getTickCount();
	
	//initialize some variables
	finder = makePtr<OrbFeaturesFinder>();
	
	//main point clouds
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_big (new pcl::PointCloud<pcl::PointXYZRGB> ());
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_small (new pcl::PointCloud<pcl::PointXYZRGB> ());
	cloud_big->is_dense = true;
	cloud_small->is_dense = true;
	
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_hexPos_MAVLink (new pcl::PointCloud<pcl::PointXYZRGB> ());
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_hexPos_FM (new pcl::PointCloud<pcl::PointXYZRGB> ());
	cloud_hexPos_MAVLink->is_dense = true;
	cloud_hexPos_FM->is_dense = true;
	
	int n_cycle;
	if(online)
		n_cycle = ceil(1.0 * rawImageDataVec.size() / seq_len);
	else
		n_cycle = 1;
	
	boost::thread the_visualization_thread;
	
	bool log_uav_positions = false;
	
	int current_idx = 0;
	int last_idx = rawImageDataVec.size() - 1;
	int cycle = 0;
	bool red_or_blue = true;
	
	while(current_idx <= last_idx)
	{
		int64 t0 = getTickCount();
		
		int cycle_start_idx = current_idx;
		
		cout << "\nCycle " << cycle << endl;
		log_file << "\nCycle " << cycle << endl;
		int images_in_cycle = 0;
		cout << "rawImageDataVec.size() " << rawImageDataVec.size() << endl;
		
		while(images_in_cycle < seq_len && current_idx <= last_idx)
		{
			double disp_img_var = getVariance(rawImageDataVec[current_idx].disparity_image, false);
			cout << rawImageDataVec[current_idx].img_num << " " << flush;
			log_file << rawImageDataVec[current_idx].img_num << " disp_img_var " << disp_img_var << "\t";
			if (disp_img_var > 5)
			{
				cout << " disp_img_var = " << disp_img_var << " > 5.\tRejected!" << endl;
				current_idx++;
				continue;
			}
			
			pcl::registration::TransformationEstimation<pcl::PointXYZRGB, pcl::PointXYZRGB>::Matrix4 t_mat_MAVLink = generateTmat(current_idx);
			//double z_normal = t_mat_MAVLink(0,2) + t_mat_MAVLink(1,2) + t_mat_MAVLink(2,2) + t_mat_MAVLink(3,2);
			//if(z_normal < -(1+z_threshold) || z_normal > -(1-z_threshold))
			//{
			//	cout << rawImageDataVec[current_idx].img_num << " z_normal " << z_normal << " > +-" << z_threshold*100 << "%. Rejected!" << endl;
			//	current_idx++;
			//	continue;
			//}
			
			pcl::PointXYZRGB hexPosMAVLink = createPCLPoint(current_idx);
			cloud_hexPos_MAVLink->points.push_back(hexPosMAVLink);
			
			//Find Features
			ImageData currentImageDataObj = findFeatures(current_idx);
			currentImageDataObj.t_mat_MAVLink = t_mat_MAVLink;
			
			if (current_idx > 0)
			{
				//Feature Matching Alignment
				//generate point clouds of matched keypoints and estimate rigid body transform between them
				bool acceptDecision = true;
				pcl::registration::TransformationEstimation<pcl::PointXYZRGB, pcl::PointXYZRGB>::Matrix4 T_SVD_matched_pts = generate_tf_of_Matched_Keypoints(currentImageDataObj, acceptDecision);
				
				if (!acceptDecision)
				{//rejected point -> no matches found
					cout << "\tLow Feature Matches.\tRejected!" << endl;
					cloud_hexPos_MAVLink->points.pop_back();
					current_idx++;
					continue;
				}
				else
				{
					cout << "\tAccepted!" << endl;
				}
				
				currentImageDataObj.t_mat_FeatureMatched = T_SVD_matched_pts * t_mat_MAVLink;
				pcl::PointXYZRGB hexPosFM = transformPoint(hexPosMAVLink, T_SVD_matched_pts);
				cloud_hexPos_FM->points.push_back(hexPosFM);
			}
			else
			{
				currentImageDataObj.t_mat_FeatureMatched = t_mat_MAVLink;
				cloud_hexPos_FM->points.push_back(hexPosMAVLink);
			}
			
			acceptedImageDataVec.push_back(currentImageDataObj);
			current_idx++;
			images_in_cycle++;
			//cout << "current_idx " << current_idx << " images_in_cycle " << images_in_cycle << endl;
			
		}
		
		cout << "Out of Feature Matching and Fitting Block" << endl;
		log_file << "Out of Feature Matching and Fitting Block" << endl;
		
		int64 t2 = getTickCount();
		cout << "\nMatching features and finding transformations time: " << (t2 - t0) / getTickFrequency() << " sec\n" << endl;
		log_file << "Matching features n transformations time:\t" << (t2 - t0) / getTickFrequency() << " sec" << endl;
		
		////finding normals of the hexPos
		//cout << "cloud_hexPos_FM: ";
		//findNormalOfPtCloud(cloud_hexPos_FM);
		//cout << "cloud_hexPos_MAVLink: ";
		//findNormalOfPtCloud(cloud_hexPos_MAVLink);
		
		//transforming the camera positions using ICP
		pcl::registration::TransformationEstimation<pcl::PointXYZRGB, pcl::PointXYZRGB>::Matrix4 tf_icp = runICPalignment(cloud_hexPos_FM, cloud_hexPos_MAVLink);
		//pcl::registration::TransformationEstimation<pcl::PointXYZRGB, pcl::PointXYZRGB>::Matrix4 tf_icp = runICPalignment(row12_FM_UAV_pos, row12_MAV_UAV_pos);
		
		//correcting old tf_mats
		for (int i = 0; i < acceptedImageDataVec.size(); i++)
			acceptedImageDataVec[i].t_mat_FeatureMatched = tf_icp * acceptedImageDataVec[i].t_mat_FeatureMatched;
		
		//fit FM camera positions to MAVLink camera positions using ICP and use the tf to correct point cloud
		transformPtCloud(cloud_hexPos_FM, cloud_hexPos_FM, tf_icp);
		
		//add additional correction for each individual UAV position -> translate back to MAVLink location by a percentage value
		//find distance and then translate
		//double error_x = 0, error_y = 0, error_z = 0;
		//for (int i = 0; i < current_idx + 1; i++)
		//{
		//	error_x += cloud_hexPos_MAVLink->points[i].x - cloud_hexPos_FM->points[i].x;
		//	error_y += cloud_hexPos_MAVLink->points[i].y - cloud_hexPos_FM->points[i].y;
		//	error_z += cloud_hexPos_MAVLink->points[i].z - cloud_hexPos_FM->points[i].z;
		//}
		//double mean_error_x = error_x/current_idx, mean_error_y = error_y/current_idx, mean_error_z = error_z/current_idx;
		//apply equivalent spread out correction to all positions
		//double K = 0.0;
		//pcl::registration::TransformationEstimation<pcl::PointXYZRGB, pcl::PointXYZRGB>::Matrix4 t_correct;
		//for (int i = 0; i < 4; i++)
		//	for (int j = 0; j < 4; j++)
		//		t_correct(i,j) = 0;
		//t_correct(0,0) = t_correct(1,1) = t_correct(2,2) = t_correct(3,3) = 1.0;
		//t_correct(0,3) = error_x/(cycle + 2)*K;
		//t_correct(1,3) = error_y/(cycle + 2)*K;
		////t_correct(2,3) = error_z;
		//for (int i = cloud_hexPos_FM->size() - row2_UAV_pos->size(); i < cloud_hexPos_FM->size() + 1; i++)
		//{
		//	t_FMVec[i] = t_correct * t_FMVec[i];
		//	cloud_hexPos_FM->points[i].x += error_x/(cycle + 2)*K;
		//	cloud_hexPos_FM->points[i].y += error_y/(cycle + 2)*K;
		//	//cloud_hexPos_FM->points[i].z += error_z;
		//}
		
		//correcting old point cloud
		transformPtCloud(cloud_big, cloud_big, tf_icp);
		
		int64 t3 = getTickCount();
		cout << "\nICP alignment and point cloud correction time: " << (t3 - t2) / getTickFrequency() << " sec\n" << endl;
		log_file << "ICP point cloud correction time:\t\t" << (t3 - t2) / getTickFrequency() << " sec" << endl;
		
		
		//adding new points to point cloud
		cout << "Adding Point Cloud number/points ";
		log_file << "Adding Point Cloud number/points ";
		
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudrgb_FeatureMatched (new pcl::PointCloud<pcl::PointXYZRGB> ());
		
		int i = seq_len * cycle;
		int acceptedImageDataVecSize = acceptedImageDataVec.size();
		//cout << "\nacceptedImageDataVec.size() " << acceptedImageDataVecSize << endl;
		//cout << "i from " << seq_len * cycle << " to " << min(seq_len * (cycle + 1), seq_len * cycle + images_in_cycle) << endl;
		while(i < min(seq_len * (cycle + 1), seq_len * cycle + images_in_cycle))
		{
			if(false)
			{//single threaded
				pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformed_cloudrgb1 ( new pcl::PointCloud<pcl::PointXYZRGB>() );
				createAndTransformPtCloud(i, transformed_cloudrgb1);
				//generating the bigger point cloud
				cloudrgb_FeatureMatched->insert(cloudrgb_FeatureMatched->end(),transformed_cloudrgb1->begin(),transformed_cloudrgb1->end());
				i++;
			}
			else
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
				
				//cout << "Start threads for creation of single image point clouds... i " << i << endl;
				for (int j = i; j < min(i+7, (int)acceptedImageDataVec.size()); j++)
					cout << acceptedImageDataVec[j].raw_img_data_ptr->img_num << " " << flush;
				cout << endl;
				pt_cloud_thread1 = boost::thread(&Pose::createAndTransformPtCloud, this, i, transformed_cloudrgb1); i++;
				if(i < acceptedImageDataVecSize) { pt_cloud_thread2 = boost::thread(&Pose::createAndTransformPtCloud, this, i, transformed_cloudrgb2); i++; }
				if(i < acceptedImageDataVecSize) { pt_cloud_thread3 = boost::thread(&Pose::createAndTransformPtCloud, this, i, transformed_cloudrgb3); i++; }
				if(i < acceptedImageDataVecSize) { pt_cloud_thread4 = boost::thread(&Pose::createAndTransformPtCloud, this, i, transformed_cloudrgb4); i++; }
				if(i < acceptedImageDataVecSize) { pt_cloud_thread5 = boost::thread(&Pose::createAndTransformPtCloud, this, i, transformed_cloudrgb5); i++; }
				if(i < acceptedImageDataVecSize) { pt_cloud_thread6 = boost::thread(&Pose::createAndTransformPtCloud, this, i, transformed_cloudrgb6); i++; }
				if(i < acceptedImageDataVecSize) { pt_cloud_thread7 = boost::thread(&Pose::createAndTransformPtCloud, this, i, transformed_cloudrgb7); i++; }
				//cout << "Created threads.." << endl;
				
				pt_cloud_thread1.join();
				pt_cloud_thread2.join();
				pt_cloud_thread3.join();
				pt_cloud_thread4.join();
				pt_cloud_thread5.join();
				pt_cloud_thread6.join();
				pt_cloud_thread7.join();
				
				//generating the bigger point cloud
				//cout << "\nPoint Cloud Creation and Downsampling done." << endl;
				i = i0;
				cloudrgb_FeatureMatched->insert(cloudrgb_FeatureMatched->end(),transformed_cloudrgb1->begin(),transformed_cloudrgb1->end()); i++;
				if(i < acceptedImageDataVecSize) { cloudrgb_FeatureMatched->insert(cloudrgb_FeatureMatched->end(),transformed_cloudrgb2->begin(),transformed_cloudrgb2->end()); i++; }
				if(i < acceptedImageDataVecSize) { cloudrgb_FeatureMatched->insert(cloudrgb_FeatureMatched->end(),transformed_cloudrgb3->begin(),transformed_cloudrgb3->end()); i++; }
				if(i < acceptedImageDataVecSize) { cloudrgb_FeatureMatched->insert(cloudrgb_FeatureMatched->end(),transformed_cloudrgb4->begin(),transformed_cloudrgb4->end()); i++; }
				if(i < acceptedImageDataVecSize) { cloudrgb_FeatureMatched->insert(cloudrgb_FeatureMatched->end(),transformed_cloudrgb5->begin(),transformed_cloudrgb5->end()); i++; }
				if(i < acceptedImageDataVecSize) { cloudrgb_FeatureMatched->insert(cloudrgb_FeatureMatched->end(),transformed_cloudrgb6->begin(),transformed_cloudrgb6->end()); i++; }
				if(i < acceptedImageDataVecSize) { cloudrgb_FeatureMatched->insert(cloudrgb_FeatureMatched->end(),transformed_cloudrgb7->begin(),transformed_cloudrgb7->end()); i++; }
				//cout << "Point Clouds Added. i " << i << endl;
			}
		}
		
		int64 t4 = getTickCount();
		cout << "\n\nPoint Cloud Creation time: " << (t4 - t3) / getTickFrequency() << " sec" << endl;
		log_file << "Point Cloud Creation time:\t\t\t" << (t4 - t3) / getTickFrequency() << " sec" << endl;
		
		if(online)
		{
			cout << "joining..." << endl;
			//adding the new downsampled points to old downsampled cloud
			cloud_big->insert(cloud_big->end(),cloudrgb_FeatureMatched->begin(),cloudrgb_FeatureMatched->end());
		}
		else
		{
			//downsample
			wait_at_visualizer = false;
			cout << "downsampling..." << endl;
			cloud_small = downsamplePtCloud(cloudrgb_FeatureMatched, true);
			//adding the new downsampled points to old downsampled cloud
			cloud_big->insert(cloud_big->end(),cloudrgb_FeatureMatched->begin(),cloudrgb_FeatureMatched->end());
		}
		
		int64 t5 = getTickCount();
		
		cout << "\nDownsampling/Joining time: " << (t5 - t4) / getTickFrequency() << " sec" << endl;
		log_file << "Downsampling/Joining time:\t\t\t" << (t5 - t4) / getTickFrequency() << " sec" << endl;
		
		//visualize
		if(preview)
		{
			bool last_cycle = current_idx > last_idx;
			if (online)
			{
				pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_big_copy (new pcl::PointCloud<pcl::PointXYZRGB>());
				copyPointCloud(*cloud_big, *cloud_big_copy);
				
				if(cycle > 0)
					the_visualization_thread.join();
				
				the_visualization_thread = boost::thread(&Pose::displayPointCloudOnline, this, cloud_big_copy, cloud_hexPos_FM, cloud_hexPos_MAVLink, cycle, last_cycle);
			}
			else
			{
				the_visualization_thread = boost::thread(&Pose::displayPointCloudOnline, this, cloud_small, cloud_hexPos_FM, cloud_hexPos_MAVLink, cycle, last_cycle);
			}
		}
		
		finder->collectGarbage();
		//increment cycle
		cycle++;
		
		int64 t6 = getTickCount();
		
		cout << "\nCycle time: " << (t6 - t0) / getTickFrequency() << " sec" << endl;
		log_file << "Cycle time:\t\t\t\t\t" << (t6 - t0) / getTickFrequency() << " sec" << endl;
	}
	
	int64 tend = getTickCount();
	
	cout << "\nFinished Pose Estimation, total time: " << ((tend - app_start_time) / getTickFrequency()) << " sec at " << 1.0*img_numbers.size()/((tend - app_start_time) / getTickFrequency()) << " fps" 
		<< "\nimages " << img_numbers.size()
		<< "\njump_pixels " << jump_pixels
		<< "\nseq_len " << seq_len
		<< "\nrange_width " << range_width
		<< "\nblur_kernel " << blur_kernel
		<< "\nvoxel_size " << voxel_size
		<< "\nmin_points_per_voxel " << min_points_per_voxel
		<< "\ndist_nearby " << dist_nearby
		<< "\ngood_matched_imgs " << good_matched_imgs
		<< endl;
	log_file << "\nFinished Pose Estimation, total time: " << ((tend - app_start_time) / getTickFrequency()) << " sec at " << 1.0*img_numbers.size()/((tend - app_start_time) / getTickFrequency()) << " fps" 
		<< "\nimages " << img_numbers.size()
		<< "\njump_pixels " << jump_pixels
		<< "\nseq_len " << seq_len
		<< "\nrange_width " << range_width
		<< "\nblur_kernel " << blur_kernel
		<< "\nvoxel_size " << voxel_size
		<< "\nmin_points_per_voxel " << min_points_per_voxel
		<< "\ndist_nearby " << dist_nearby
		<< "\ngood_matched_imgs " << good_matched_imgs
		<< endl;
	
	log_file << "\nMAVLink hexacopter positions" << endl;
	for (int i = 0; i <= last_idx; i++)
		log_file << img_numbers[i] << "," << cloud_hexPos_MAVLink->points[i].x << "," << cloud_hexPos_MAVLink->points[i].y << "," << cloud_hexPos_MAVLink->points[i].z << endl;
	
	log_file << "\nFeature Matched and ICP corrected hexacopter positions" << endl;
	for (int i = 0; i <= last_idx; i++)
		log_file << img_numbers[i] << "," << cloud_hexPos_FM->points[i].x << "," << cloud_hexPos_FM->points[i].y << "," << cloud_hexPos_FM->points[i].z << endl;
	
	log_file << "\nerror in uav positions" << endl;
	for (int i = 0; i <= last_idx; i++)
		log_file << img_numbers[i] << "," << cloud_hexPos_MAVLink->points[i].x - cloud_hexPos_FM->points[i].x << "," << cloud_hexPos_MAVLink->points[i].y - cloud_hexPos_FM->points[i].y << "," << cloud_hexPos_MAVLink->points[i].z - cloud_hexPos_FM->points[i].z << endl;
	
	double error_x = 0, error_y = 0, error_z = 0;
	for (int i = 0; i <= last_idx; i++)
	{
		error_x += cloud_hexPos_MAVLink->points[i].x - cloud_hexPos_FM->points[i].x;
		error_y += cloud_hexPos_MAVLink->points[i].y - cloud_hexPos_FM->points[i].y;
		error_z += cloud_hexPos_MAVLink->points[i].z - cloud_hexPos_FM->points[i].z;
	}
	double mean_error_x = error_x/(last_idx + 1), mean_error_y = error_y/(last_idx + 1), mean_error_z = error_z/(last_idx + 1);
	double var_error_x = 0, var_error_y = 0, var_error_z = 0;
	for (int i = 0; i <= last_idx; i++)
	{
		error_x += cloud_hexPos_MAVLink->points[i].x - cloud_hexPos_FM->points[i].x;
		error_y += cloud_hexPos_MAVLink->points[i].y - cloud_hexPos_FM->points[i].y;
		error_z += cloud_hexPos_MAVLink->points[i].z - cloud_hexPos_FM->points[i].z;
		
		var_error_x += (error_x - mean_error_x) * (error_x - mean_error_x);
		var_error_y += (error_y - mean_error_y) * (error_y - mean_error_y);
		var_error_z += (error_z - mean_error_z) * (error_z - mean_error_z);
	}
	double stddev_error_x = sqrt(var_error_x / (last_idx * last_idx)), 
		stddev_error_y = sqrt(var_error_y / (last_idx * last_idx)), 
		stddev_error_z = sqrt(var_error_z / (last_idx * last_idx));
	//cout << "total localization errors in x " << error_x << " y " << error_y << " z " << error_z << endl;
	cout << "\navg UAV localization error (m) in x " << mean_error_x << " y " << mean_error_y << " z " << mean_error_z << endl;
	log_file << "\navg UAV localization error (m) in x " << mean_error_x << " y " << mean_error_y << " z " << mean_error_z << endl;
	cout << "std deviation in UAV localization error in x " << stddev_error_x << " y " << stddev_error_y << " z " << stddev_error_z << endl << endl;
	log_file << "std deviation in UAV localization error in x " << stddev_error_x << " y " << stddev_error_y << " z " << stddev_error_z << endl << endl;
	
	if(online)
	{
		cout << "downsample before saving..." << endl;
		cloud_small = downsamplePtCloud(cloud_big, true);
		cout << "downsampled." << endl;
	}
	
	cout << "Saving point clouds..." << endl;
	read_PLY_filename0 = folder + "cloud.ply";
	save_pt_cloud_to_PLY_File(cloud_small, read_PLY_filename0);
	//read_PLY_filename0 = "cloudrgb_MAVLink_" + currentDateTimeStr + ".ply";
	//save_pt_cloud_to_PLY_File(cloudrgb_MAVLink, read_PLY_filename0);
	//read_PLY_filename1 = folder + "cloud_big.ply";
	//save_pt_cloud_to_PLY_File(cloud_big, read_PLY_filename1);
	
	//downsampling
	//cout << "downsampling..." << endl;
	//log_file << "downsampling..." << endl;
	//pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudrgb_MAVLink_downsamp = downsamplePtCloud(cloudrgb_MAVLink, false);
	//read_PLY_filename0 = "downsampled_" + read_PLY_filename0;
	//save_pt_cloud_to_PLY_File(cloudrgb_MAVLink_downsamp, read_PLY_filename0);
	
	cloud_hexPos_FM->insert(cloud_hexPos_FM->end(),cloud_hexPos_MAVLink->begin(),cloud_hexPos_MAVLink->end());
	string hexpos_filename = folder + "cloud_uavpos.ply";
	save_pt_cloud_to_PLY_File(cloud_hexPos_FM, hexpos_filename);
	
	if(preview)
		the_visualization_thread.join();
	
	if (segment_map)
	{
		segmentCloud(cloud_small);
	}
	
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

void Pose::createAndTransformPtCloud(int accepted_img_index, pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloudrgb_return)
{
	try
	{
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudrgb (new pcl::PointCloud<pcl::PointXYZRGB> ());
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudrgb_transformed (new pcl::PointCloud<pcl::PointXYZRGB> ());
		
		//if(jump_pixels == 1)
		//	createPtCloud(img_index, cloudrgb);
		//else
			createSingleImgPtCloud(accepted_img_index, cloudrgb);
		//cout << "Created point cloud " << img_index << endl;
		
		pcl::registration::TransformationEstimation<pcl::PointXYZRGB, pcl::PointXYZRGB>::Matrix4 t_mat_FeatureMatched = acceptedImageDataVec[accepted_img_index].t_mat_FeatureMatched;
		transformPtCloud(cloudrgb, cloudrgb_transformed, t_mat_FeatureMatched);
		//cout << "transformed point cloud " << img_index << endl;
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudrgb_downsampled = downsamplePtCloud(cloudrgb_transformed, false);
		//cout << "Downsampled point cloud " << img_index << endl;
		copyPointCloud(*cloudrgb_downsampled, *cloudrgb_return);
			
	}
	catch (const std::runtime_error& e)
	{
		// this executes if f() throws std::underflow_error (base class rule)
		cout << "std::underflow_error Exception caught in thread with accepted_img_index=" << accepted_img_index << endl;
		cout << e.what() << endl;
	}
	catch (exception& e)
	{
		cout << "Exception caught in thread with accepted_img_index=" << accepted_img_index << endl;
		cout << e.what() << endl;
	}
	catch (...)
	{
		// this executes if f() throws std::string or int or any other unrelated type
		cout << "General Exception caught in thread with accepted_img_index=" << accepted_img_index << endl;
	}
}

void Pose::displayPointCloudOnline(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud_combined_copy, 
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud_hexPos_FM, pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud_hexPos_MAVLink, int cycle, bool last_cycle)
{
	wait_at_visualizer = false;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudrgb (new pcl::PointCloud<pcl::PointXYZRGB> ());
	if(online)
	{
		cloudrgb = downsamplePtCloud(cloud_combined_copy, true);
	}
	else
	{
		cloudrgb = cloud_combined_copy;
	}
	
	displayUAVPositions = true;
	pcl::PolygonMesh mesh;
	//visualize_pt_cloud(true, cloudrgb_FeatureMatched_downsampA, false, mesh, "cloudrgb_FM_Fitted_downsampledA");
	
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr hexPos_cloud_online (new pcl::PointCloud<pcl::PointXYZRGB>());
	hexPos_cloud_online->insert(hexPos_cloud_online->begin(),cloud_hexPos_FM->begin(),cloud_hexPos_FM->end());
	hexPos_cloud_online->insert(hexPos_cloud_online->end(),cloud_hexPos_MAVLink->begin(),cloud_hexPos_MAVLink->end());
	hexPos_cloud = hexPos_cloud_online;
	if (cycle == 0)
	{
		viewer_online = visualize_pt_cloud(true, cloudrgb, false, mesh, "cloudrgb_visualization_Online");
	}
	else
	{
		visualize_pt_cloud_update(cloudrgb, "cloudrgb_visualization_Online", viewer_online);
	}
	if(last_cycle)
	{
		while (!viewer_online->wasStopped ()) { // Display the visualiser until 'q' key is pressed
			viewer_online->spinOnce();
		}
	}
}

// stack trace reference https://stackoverflow.com/questions/77005/how-to-automatically-generate-a-stacktrace-when-my-program-crashes
/* This structure mirrors the one found in /usr/include/asm/ucontext.h */
typedef struct _sig_ucontext {
 unsigned long     uc_flags;
 struct ucontext   *uc_link;
 stack_t           uc_stack;
 struct sigcontext uc_mcontext;
 sigset_t          uc_sigmask;
} sig_ucontext_t;

void crit_err_hdlr(int sig_num, siginfo_t * info, void * ucontext)
{
 void *             array[50];
 void *             caller_address;
 char **            messages;
 int                size, i;
 sig_ucontext_t *   uc;

 uc = (sig_ucontext_t *)ucontext;

 /* Get the address at the time the signal was raised */
#if defined(__i386__) // gcc specific
 caller_address = (void *) uc->uc_mcontext.eip; // EIP: x86 specific
#elif defined(__x86_64__) // gcc specific
 caller_address = (void *) uc->uc_mcontext.rip; // RIP: x86_64 specific
#else
#error Unsupported architecture. // TODO: Add support for other arch.
#endif

 fprintf(stderr, "signal %d (%s), address is %p from %p\n", 
  sig_num, strsignal(sig_num), info->si_addr, 
  (void *)caller_address);

 size = backtrace(array, 50);

 /* overwrite sigaction with caller's address */
 array[1] = caller_address;

 messages = backtrace_symbols(array, size);

 /* skip first stack frame (points here) */
 for (i = 1; i < size && messages != NULL; ++i)
 {
  fprintf(stderr, "[bt]: (%d) %s\n", i, messages[i]);
 }

 free(messages);

 exit(EXIT_FAILURE);
}

int main(int argc, char* argv[])
{
	cout << setprecision(3) << 
		  "\n**********   Unmanned Systems Lab    **********"
		"\n\n********** 3D Reconstruction Program **********"
		"\n\nAuthor: Prashant Kumar"
		"\n\nHelp  ./pose --help"
		"\n"
		<< endl;
	
	//plotMatches("images/1248.png","images/1251.png");
	//plotMatches("images/1248.png","images/1258.png");
	
	//struct sigaction sigact;
    //
	//sigact.sa_sigaction = crit_err_hdlr;
	//sigact.sa_flags = SA_RESTART | SA_SIGINFO;
    //
	//if (sigaction(SIGSEGV, &sigact, (struct sigaction *)NULL) != 0)
	//{
	//	fprintf(stderr, "error setting signal handler for %d (%s)\n",
	//	SIGSEGV, strsignal(SIGSEGV));
    //
	//	exit(EXIT_FAILURE);
	//}
	
	try
	{
		Pose pose(argc, argv);
	}
	catch (exception& e)
	{
		cout << e.what() << '\n';
	}
	catch (const char* msg)
	{
		cerr << msg << endl;
	}
	
	return 0;
}

