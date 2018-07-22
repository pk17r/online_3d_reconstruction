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
	declareDependentVariables();
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
	if(preview)
	{
		//int img_index = 0;
		pcl::visualization::PCLVisualizer viewer ("3d reconstruction");
		
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud0 (new pcl::PointCloud<pcl::PointXYZRGB> ());
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud1 (new pcl::PointCloud<pcl::PointXYZRGB> ());
		
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformed_cloud0( new pcl::PointCloud<pcl::PointXYZRGB>() );
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformed_cloud1( new pcl::PointCloud<pcl::PointXYZRGB>() );
		
		createPtCloud(0, cloud0, transformed_cloud0);
		createPtCloud(1, cloud1, transformed_cloud1);
		
		pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> transformed_cloud_color_handler0 (transformed_cloud0, 230, 20, 20); // Red
		pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> transformed_cloud_color_handler1 (transformed_cloud1, 20, 230, 20); // Green
		
		viewer.addPointCloud<pcl::PointXYZRGB> (transformed_cloud0, transformed_cloud_color_handler0, "transformed_cloud" + to_string(0));
		viewer.addPointCloud<pcl::PointXYZRGB> (transformed_cloud1, transformed_cloud_color_handler1, "transformed_cloud" + to_string(1));
		
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

	cout << "Converting 2D matches to 3D matches, time: " << ((getTickCount() - t) / getTickFrequency()) << " sec" << endl;
	
	cout << "Finding 3D transformation and Eular Angles..." << endl;
	t = getTickCount();
	
	//*4. find transformation between corresponding 3D points using estimateAffine3D: Output 3D affine transformation matrix  3 x 4
	if(log_stuff)
		f << "Q:\n" << Q << endl;

	Mat affineTransformationMatrix;
	std::vector<uchar> inliers3D;
	double ransacThreshold = 3, confidence = 0.99;
	estimateAffine3D(keypoints3D_src, keypoints3D_dst, affineTransformationMatrix, inliers3D, ransacThreshold, confidence);
	cout << "affineTransformationMatrix:\n" << affineTransformationMatrix << endl;
	
	int inlierCount = 0;
	for (int i = 0; i < inliers3D.size(); i++)
	{
		if (inliers3D[i] == 1)
			inlierCount++;
		f << (int)inliers3D[i] << ", ";
	}
	cout << "inlierCount: " << inlierCount << "/" << inliers3D.size() << endl;
	
	//* 5. use decomposeProjectionMatrix to get rotation or Euler angles
	Mat cameraMatrix, rotMatrix, transVect, rotMatrixX, rotMatrixY, rotMatrixZ, eulerAngles;
	decomposeProjectionMatrix(affineTransformationMatrix, cameraMatrix, rotMatrix, transVect, rotMatrixX, rotMatrixY, rotMatrixZ, eulerAngles);
	cout << "EulerAngles :\n" << eulerAngles << endl;
	
	cout << "Finding 3D transformation and Eular Angles, time: " << ((getTickCount() - t) / getTickFrequency()) << " sec" << endl;
	cout << "Finished Pose Estimation, total time: " << ((getTickCount() - app_start_time) / getTickFrequency()) << " sec" << endl;
	
	if(log_stuff)
	{
		f << "affineTransformationMatrix:\n" << affineTransformationMatrix << endl;
		f << "ransacThreshold: " << ransacThreshold << " confidence: " << confidence << endl;
		f << "inliers3D:\n[";
		f << "]" << endl;
		f << "inlierCount: " << inlierCount << "/" << inliers3D.size() << endl;
		f << "cameraMatrix:\n" << cameraMatrix << "\nrotMatrix:\n" << rotMatrix << "\ntransVect :\n" << transVect << "\nrotMatrixX :\n" << rotMatrixX << "\nrotMatrixY :\n" << rotMatrixY << "\nrotMatrixZ :\n" << rotMatrixZ << "\nEulerAngles :\n" << eulerAngles << endl;
	}
	
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
			log_stuff = true;
		}
		else if (string(argv[i]) == "--preview")
		{
			preview = true;
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

inline void declareDependentVariables()
{
	images = vector<Mat>(img_numbers.size());
	full_images = vector<Mat>(img_numbers.size());
	disparity_images = vector<Mat>(img_numbers.size());
	full_img_sizes = vector<Size>(img_numbers.size());
	save_graph_to = "output/graph";
	//logging stuff
	if(log_stuff)
		f.open(save_graph_to.c_str(), ios::out);

}

inline void readImages()
{
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
		//read disparity image
		Mat disp_img(rows,cols, CV_8UC1);
		disp_img = imread(disparityPrefix + to_string(img_numbers[i]) + ".png",CV_LOAD_IMAGE_GRAYSCALE);
		disparity_images[i] = disp_img;
		
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

void createPtCloud(int img_index, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformed_cloud)
{
	cout << "Image index: " << img_index << endl;
	cloud->width    = 20;
	cloud->height   = 15;
	cloud->is_dense = false;
	cloud->points.resize (cloud->width * cloud->height);
	
	int point_clout_pts = 0;
	cv::Mat_<double> vec_tmp(4,1);
	
	Mat disp_img = disparity_images[img_index];
	
	for (int y = boundingBox; y < rows - boundingBox; ++y)
	{
		for (int x = cols_start_aft_cutout; x < cols - boundingBox; ++x)
		{
			double disp_val = (double)disp_img.at<uchar>(y,x);
			
			if (disp_val > minDisparity)
			{
				//reference: https://stackoverflow.com/questions/22418846/reprojectimageto3d-in-opencv
				vec_tmp(0)=x; vec_tmp(1)=y; vec_tmp(2)=disp_val; vec_tmp(3)=1;
				vec_tmp = Q*vec_tmp;
				vec_tmp /= vec_tmp(3);
				
				point_clout_pts++;
				
				pcl::PointXYZRGB pt_3d;
				pt_3d.x = (float)vec_tmp(0);
				pt_3d.y = (float)vec_tmp(1);
				pt_3d.z = -(float)vec_tmp(2);
				Vec3b color = full_images[0].at<Vec3b>(Point(x, y));
				uint32_t rgb = ((uint32_t)color[2] << 16 | (uint32_t)color[1] << 8 | (uint32_t)color[0]);
				pt_3d.rgb = *reinterpret_cast<float*>(&rgb);
				
				cloud->points.push_back(pt_3d);
				//cout << pt_3d << endl;
			}
		}
	}
	
	cout << "pose_seq_to_find " << img_numbers[img_index] << endl;
	int found_index = binary_search_find_index(pose_sequence, img_numbers[img_index]);
	cout << "found_index " << found_index << endl;
	
	Eigen::Affine3f transform_2 = Eigen::Affine3f::Identity();
	// Define a translation of 2.5 meters on the x axis.
	transform_2.translation() << pose_data[found_index][1], pose_data[found_index][2], pose_data[found_index][3];

	// The same rotation matrix as before; theta radians around Z axis
	double theta = 0;
	transform_2.rotate (Eigen::AngleAxisf (theta, Eigen::Vector3f::UnitZ()));

	// Print the transformation
	printf ("Correcting 3D pt cloud using an Affine3f\n");
	std::cout << transform_2.matrix() << std::endl;
	
	// Executing the transformation
	//pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformed_cloud (new pcl::PointCloud<pcl::PointXYZRGB> ());
	pcl::transformPointCloud(*cloud, *transformed_cloud, transform_2);
	// Define R,G,B colors for the point cloud
	//pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> transformed_cloud_color_handler (transformed_cloud, 230, 20, 20); // Red
	
	cout << "point cloud created!" << endl;
	
	// Define R,G,B colors for the point cloud
	//pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb (cloud0);
	
	// We add the point cloud to the viewer and pass the color handler
	//viewer.addPointCloud<pcl::PointXYZRGB> (cloud0, rgb, "original_cloud" + to_string(img_index));
	//viewer.addPointCloud<pcl::PointXYZRGB> (transformed_cloud, transformed_cloud_color_handler, "transformed_cloud" + to_string(img_index));
	
	//viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "original_cloud" + to_string(img_index));
	//viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "transformed_cloud" + to_string(img_index));
	
	cout << "point_clout_pts: " << point_clout_pts << endl;
	if(log_stuff)
		f << "point_clout_pts: " << point_clout_pts << endl;
	
}
