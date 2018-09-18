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
	
	if(test_bad_data_rejection)
	{
		cout << "*** inside test_bad_data_rejection to test rejection of tilted UAV positions and bad disparity images ***\n" << endl;
		
		readPoseFile();
		
		//namedWindow("dispImg", CV_WINDOW_AUTOSIZE);
		
		cout << "rows " << rows << " cols " << cols << " cols_start_aft_cutout " << cols_start_aft_cutout << endl;
		//test load an image to fix rows, cols and cols_start_aft_cutout
		Mat test_load_img = imread(disparityPrefix + to_string(1141) + ".png", CV_LOAD_IMAGE_GRAYSCALE);
		rows = test_load_img.rows;
		cols = test_load_img.cols;
		cols_start_aft_cutout = (int)(cols/cutout_ratio);
		cout << "rows " << rows << " cols " << cols << " cols_start_aft_cutout " << cols_start_aft_cutout << endl;
		cout << fixed;
		cout << setprecision(5);
		cout << endl;
		
		for (int img_id = 1141; img_id < 1600; img_id++)
		{
			int pose_index = data_index_finder(img_id);
			pcl::registration::TransformationEstimation<pcl::PointXYZRGB, pcl::PointXYZRGB>::Matrix4 t_mat = generateTmat(pose_data[pose_index]);
			double z_normal = t_mat(0,2) + t_mat(1,2) + t_mat(2,2) + t_mat(3,2);
			cout << img_id << " z_normal " << z_normal;
			
			string disp_img_name = disparityPrefix + to_string(img_id) + ".png";
			Mat disp_img = imread(disp_img_name,CV_LOAD_IMAGE_GRAYSCALE);
			if(disp_img.empty())
			{
				cout << "disparity img_id " << img_id << " not present!" << endl;
				continue;
			}
			//imshow( "dispImg", disp_img );                   // Show our image inside it.
			//waitKey(1);                                          // Wait for a keystroke in the window
			
			double disp_img_var = getVariance(disp_img, false);
			cout << "\tdisp_img_var " << disp_img_var;
			if (disp_img_var > 5)
				cout << "\tvariance > 5!";
			
			if(z_normal < -1.05 || z_normal > -0.95)
				cout << "\tz_normal > +-5%!";
			
			cout << endl;
		}
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
	
	readCalibFile();
	readPoseFile();
	populateData();
	
	//checks
	if(rows == 0 || cols == 0 || cols_start_aft_cutout == 0)
		throw "Exception: some important values not set! rows " + to_string(rows) + " cols " + to_string(cols) + " cols_start_aft_cutout " + to_string(cols_start_aft_cutout);
	
	//start program
	int64 app_start_time = getTickCount();
	
	//initialize some variables
	finder = makePtr<OrbFeaturesFinder>();
	
	features = vector<ImageFeatures>(img_numbers.size());
	for (int i = 0; i < img_numbers.size(); i++)
	{
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr keypoints3dptcloud (new pcl::PointCloud<pcl::PointXYZRGB> ());
		keypoints3dptcloud->is_dense = true;
		keypoints3DVec.push_back(keypoints3dptcloud);
	}
	
	//main point clouds
	//pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudrgb_MAVLink (new pcl::PointCloud<pcl::PointXYZRGB> ());
	//pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudrgb_FeatureMatched_big (new pcl::PointCloud<pcl::PointXYZRGB> ());
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_big (new pcl::PointCloud<pcl::PointXYZRGB> ());
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_small (new pcl::PointCloud<pcl::PointXYZRGB> ());
	cloud_big->is_dense = true;
	cloud_small->is_dense = true;
	//cloudrgb_MAVLink->is_dense = true;
	//cloudrgb_FeatureMatched_big->is_dense = true;
	
	//vectors to store transformations of point clouds
	vector<pcl::registration::TransformationEstimation<pcl::PointXYZRGB, pcl::PointXYZRGB>::Matrix4> t_matVec;
	vector<pcl::registration::TransformationEstimation<pcl::PointXYZRGB, pcl::PointXYZRGB>::Matrix4> t_FMVec;
	
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_hexPos_MAVLink (new pcl::PointCloud<pcl::PointXYZRGB> ());
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_hexPos_FM (new pcl::PointCloud<pcl::PointXYZRGB> ());
	cloud_hexPos_MAVLink->is_dense = true;
	cloud_hexPos_FM->is_dense = true;
	
	int n_cycle;
	if(online)
		n_cycle = ceil(1.0 * img_numbers.size() / seq_len);
	else
		n_cycle = 1;
	
	boost::thread the_visualization_thread;
	
	bool log_uav_positions = false;
	
	int current_idx = 0;
	int last_idx = img_numbers.size() - 1;
	int cycle = 0;
	bool row1_done = false;
	bool row2_done = false;
	vector<int> row1_UAV_pos_idx, row2_UAV_pos_idx;
	bool red_or_blue = true;
	
	Eigen::VectorXf model_coefficients;		//coeffs of line fitting
	
	while(current_idx <= last_idx)
	{
		int64 t0 = getTickCount();
		
		int cycle_start_idx = current_idx;
		int row_start_idx = current_idx;
		
		cout << "\nCycle " << cycle << endl;
		log_file << "\nCycle " << cycle << endl;
		
		while(current_idx <= last_idx)
		{
			//SEARCH PROCESS: get NSECS from images_times_data and search for corresponding or nearby entry in pose_data and heading_data
			int pose_index = data_index_finder(img_numbers[current_idx]);
			pcl::PointXYZRGB hexPosMAVLink = addPointFromPoseFile(pose_index, red_or_blue);
			
			//line fitting
			if (current_idx - row_start_idx >= min_uav_positions_for_line_fitting)
			{
				//row line fitting
				// created RandomSampleConsensus object and compute the appropriated model
				pcl::PointCloud<pcl::PointXYZRGB>::Ptr row_UAV_pos(new pcl::PointCloud<pcl::PointXYZRGB> ());
				for (int i = row_start_idx; i < current_idx; i++)
					row_UAV_pos->points.push_back(cloud_hexPos_MAVLink->points[i]);
				
				pcl::SampleConsensusModelLine<pcl::PointXYZRGB>::Ptr model_l (new pcl::SampleConsensusModelLine<pcl::PointXYZRGB> (row_UAV_pos));
				pcl::RandomSampleConsensus<pcl::PointXYZRGB> ransac (model_l);
				ransac.setDistanceThreshold (uav_line_creation_dist_threshold);
				
				ransac.computeModel();
				ransac.getModelCoefficients (model_coefficients);
				
				if(current_idx - row_start_idx == min_uav_positions_for_line_fitting)
				{
					//std::vector<int> inliers;
					//ransac.getInliers(inliers);
					//if (row_UAV_pos->size() != inliers.size())
					//	cout << "***** row_UAV_pos->size() and inliers.size() mismatch! *****" << endl;
					std::vector<double> distances;
					model_l->getDistancesToModel(model_coefficients, distances);
					
					cout << "\nUAV dist from fitted line: ";
					log_file << "\nUAV dist from fitted line: ";
					for (int a = 0; a < distances.size(); a++)
					{
						cout << distances[a] << " " << flush;
						log_file << distances[a] << " " << flush;
					}
				}
			
				//check whether new UAV position lies on this row or not
				
				// Obtain the line point and direction
				Eigen::Vector4f line_pt  (model_coefficients[0], model_coefficients[1], model_coefficients[2], 0);
				Eigen::Vector4f line_dir (model_coefficients[3], model_coefficients[4], model_coefficients[5], 0);
				line_dir.normalize ();
				Eigen::Vector4f new_pt  (hexPosMAVLink.x, hexPosMAVLink.y, hexPosMAVLink.z, 0);
				
				double distance_new_uav_pos = sqrt ((line_pt - new_pt).cross3 (line_dir).squaredNorm ());

				cout << " dist_new_uav_pos: " << distance_new_uav_pos << endl;
				log_file << " dist_new_uav_pos: " << distance_new_uav_pos << endl;
				//cout << "current_idx:" << current_idx<< " last_idx:" << last_idx<< endl;
				if (distance_new_uav_pos > uav_new_row_dist_threshold)
				{//new row
					red_or_blue = !red_or_blue;
					hexPosMAVLink = addPointFromPoseFile(pose_index, red_or_blue);
					if (!row1_done)
					{
						row1_done = true;
						row_start_idx = current_idx;
						cout << "New Row!" << endl;
						cout << "row1_UAV_pos_idx->size():" << row1_UAV_pos_idx.size() << endl;
						log_file << "New Row!" << endl;
						log_file << "row1_UAV_pos_idx->size():" << row1_UAV_pos_idx.size() << endl;
					}
					else
					{
						row2_done = true;
						cout << "New Row and Cycle!" << endl;
						cout << "row2_UAV_pos_idx->size():" << row2_UAV_pos_idx.size() << endl;
						log_file << "New Row and Cycle!" << endl;
						log_file << "row2_UAV_pos_idx->size():" << row2_UAV_pos_idx.size() << endl;
						break;
					}
				}
				else if (current_idx == last_idx)	//special handling of last row ending
				{
					if (!row1_done)
					{
						cout << "Row End!" << endl;
						log_file << "Row End!" << endl;
						row1_done = true;
						row1_UAV_pos_idx.push_back(current_idx);
						cout << "row1_UAV_pos_idx->size():" << row1_UAV_pos_idx.size() << endl;
						log_file << "row1_UAV_pos_idx->size():" << row1_UAV_pos_idx.size() << endl;
					}
					else
					{
						cout << "Row and Cycle End!" << endl;
						log_file << "Row and Cycle End!" << endl;
						row2_done = true;
						row2_UAV_pos_idx.push_back(current_idx);
						cout << "row2_UAV_pos_idx->size():" << row2_UAV_pos_idx.size() << endl;
						log_file << "row2_UAV_pos_idx->size():" << row2_UAV_pos_idx.size() << endl;
					}
				}
			}
			else
				std::cout << std::endl;
			
			if (!row1_done)
				row1_UAV_pos_idx.push_back(current_idx);
			else
				row2_UAV_pos_idx.push_back(current_idx);
			
			cloud_hexPos_MAVLink->points.push_back(hexPosMAVLink);
			
			pcl::registration::TransformationEstimation<pcl::PointXYZRGB, pcl::PointXYZRGB>::Matrix4 t_mat = generateTmat(pose_data[pose_index]);
			t_matVec.push_back(t_mat);
			
			//Find Features
			findFeatures(current_idx);
			
			if (current_idx == 0)
			{
				t_FMVec.push_back(t_mat);
				cloud_hexPos_FM->points.push_back(hexPosMAVLink);
				
				current_idx++;
				continue;
			}
			
			//Feature Matching Alignment
			//generate point clouds of matched keypoints and estimate rigid body transform between them
			pcl::registration::TransformationEstimation<pcl::PointXYZRGB, pcl::PointXYZRGB>::Matrix4 T_SVD_matched_pts = generate_tf_of_Matched_Keypoints_Point_Cloud(current_idx, t_FMVec, t_mat, row1_UAV_pos_idx, row2_UAV_pos_idx, cloud_hexPos_MAVLink);
			
			t_FMVec.push_back(T_SVD_matched_pts * t_mat);
			
			pcl::PointXYZRGB hexPosFM = transformPoint(hexPosMAVLink, T_SVD_matched_pts);
			cloud_hexPos_FM->points.push_back(hexPosFM);
			
			current_idx++;
			//cout << " row12_FM_UAV_pos->size():" << row12_FM_UAV_pos->size() << endl;
		}
		
		cout << "out of cycle loop" << endl;
		log_file << "out of cycle loop" << endl;
		
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
		for (int i = 0; i < current_idx; i++)
			t_FMVec[i] = tf_icp * t_FMVec[i];
		
		//fit FM camera positions to MAVLink camera positions using ICP and use the tf to correct point cloud
		transformPtCloud(cloud_hexPos_FM, cloud_hexPos_FM, tf_icp);
		
		cout << "last row fitted line model_coefficients:\n" << model_coefficients << endl;
		log_file << "last row fitted line model_coefficients:\n" << model_coefficients << endl;
		
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
		
		//log_file << "Adding Point Cloud number/points ";
		int i = cycle_start_idx;
		while(i < current_idx)
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
			if(++i < current_idx) pt_cloud_thread2 = boost::thread(&Pose::createAndTransformPtCloud, this, i, t_FMVec, transformed_cloudrgb2);
			if(++i < current_idx) pt_cloud_thread3 = boost::thread(&Pose::createAndTransformPtCloud, this, i, t_FMVec, transformed_cloudrgb3);
			if(++i < current_idx) pt_cloud_thread4 = boost::thread(&Pose::createAndTransformPtCloud, this, i, t_FMVec, transformed_cloudrgb4);
			if(++i < current_idx) pt_cloud_thread5 = boost::thread(&Pose::createAndTransformPtCloud, this, i, t_FMVec, transformed_cloudrgb5);
			if(++i < current_idx) pt_cloud_thread6 = boost::thread(&Pose::createAndTransformPtCloud, this, i, t_FMVec, transformed_cloudrgb6);
			if(++i < current_idx) pt_cloud_thread7 = boost::thread(&Pose::createAndTransformPtCloud, this, i, t_FMVec, transformed_cloudrgb7);
			
			
			//generating the bigger point cloud
			pt_cloud_thread1.join();
			i = i0;
			cloudrgb_FeatureMatched->insert(cloudrgb_FeatureMatched->end(),transformed_cloudrgb1->begin(),transformed_cloudrgb1->end());
			pt_cloud_thread2.join();
			if(++i < current_idx) cloudrgb_FeatureMatched->insert(cloudrgb_FeatureMatched->end(),transformed_cloudrgb2->begin(),transformed_cloudrgb2->end());
			pt_cloud_thread3.join();
			if(++i < current_idx) cloudrgb_FeatureMatched->insert(cloudrgb_FeatureMatched->end(),transformed_cloudrgb3->begin(),transformed_cloudrgb3->end());
			pt_cloud_thread4.join();
			if(++i < current_idx) cloudrgb_FeatureMatched->insert(cloudrgb_FeatureMatched->end(),transformed_cloudrgb4->begin(),transformed_cloudrgb4->end());
			pt_cloud_thread5.join();
			if(++i < current_idx) cloudrgb_FeatureMatched->insert(cloudrgb_FeatureMatched->end(),transformed_cloudrgb5->begin(),transformed_cloudrgb5->end());
			pt_cloud_thread6.join();
			if(++i < current_idx) cloudrgb_FeatureMatched->insert(cloudrgb_FeatureMatched->end(),transformed_cloudrgb6->begin(),transformed_cloudrgb6->end());
			pt_cloud_thread7.join();
			if(++i < current_idx) cloudrgb_FeatureMatched->insert(cloudrgb_FeatureMatched->end(),transformed_cloudrgb7->begin(),transformed_cloudrgb7->end());
			//cout << "Transformed and added." << endl;
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
		//from now onwards row and cycle start index will be same
		//only first cycle has two rows.
		cycle_start_idx = current_idx;
		row_start_idx = current_idx;
		//copy row2 to row1.
		//row1_UAV_pos = row2_UAV_pos;
		cout << row1_UAV_pos_idx.size() << " " << row2_UAV_pos_idx.size();
		row1_UAV_pos_idx = row2_UAV_pos_idx;
		row2_UAV_pos_idx.clear();
		cout << " " << row1_UAV_pos_idx.size() << " " << row2_UAV_pos_idx.size() << endl;
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
		//<< "\nmax_depth " << max_depth
		//<< "\nmax_height " << max_height
		<< "\nmin_points_per_voxel " << min_points_per_voxel
		<< "\ndist_nearby " << dist_nearby
		<< "\ngood_matched_imgs " << good_matched_imgs
		<< "\nuav_line_creation_dist_threshold " << uav_line_creation_dist_threshold
		<< "\nuav_new_row_dist_threshold " << uav_new_row_dist_threshold
		<< "\nmin_uav_positions_for_line_fitting " << min_uav_positions_for_line_fitting
		<< endl;
	log_file << "\nFinished Pose Estimation, total time: " << ((tend - app_start_time) / getTickFrequency()) << " sec at " << 1.0*img_numbers.size()/((tend - app_start_time) / getTickFrequency()) << " fps" 
		<< "\nimages " << img_numbers.size()
		<< "\njump_pixels " << jump_pixels
		<< "\nseq_len " << seq_len
		<< "\nrange_width " << range_width
		<< "\nblur_kernel " << blur_kernel
		<< "\nvoxel_size " << voxel_size
		//<< "\nmax_depth " << max_depth
		//<< "\nmax_height " << max_height
		<< "\nmin_points_per_voxel " << min_points_per_voxel
		<< "\ndist_nearby " << dist_nearby
		<< "\ngood_matched_imgs " << good_matched_imgs
		<< "\nuav_line_creation_dist_threshold " << uav_line_creation_dist_threshold
		<< "\nuav_new_row_dist_threshold " << uav_new_row_dist_threshold
		<< "\nmin_uav_positions_for_line_fitting " << min_uav_positions_for_line_fitting
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

void Pose::createAndTransformPtCloud(int img_index, 
	vector<pcl::registration::TransformationEstimation<pcl::PointXYZRGB, pcl::PointXYZRGB>::Matrix4> &t_FMVec, 
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloudrgb_return)
{
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudrgb (new pcl::PointCloud<pcl::PointXYZRGB> ());
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudrgb_transformed (new pcl::PointCloud<pcl::PointXYZRGB> ());
	
	//if(jump_pixels == 1)
	//	createPtCloud(img_index, cloudrgb);
	//else
		createSingleImgPtCloud(img_index, cloudrgb);
	//cout << "Created point cloud " << i << endl;
	
	transformPtCloud(cloudrgb, cloudrgb_transformed, t_FMVec[img_index]);
	
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudrgb_downsampled = downsamplePtCloud(cloudrgb_transformed, false);
	
	copyPointCloud(*cloudrgb_downsampled, *cloudrgb_return);
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
	
	try
	{
		Pose pose(argc, argv);
	}
	catch (const char* msg)
	{
		cerr << msg << endl;
	}
	
	return 0;
}

