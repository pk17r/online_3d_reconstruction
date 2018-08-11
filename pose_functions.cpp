/* STEPS:
*
* Defining functions here
*
*
*
* */

#include "pose.h"


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

void Pose::printUsage()
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

string Pose::type2str(int type)
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

int Pose::parseCmdArgs(int argc, char** argv)
{
	if (argc == 1)
	{
		printUsage();
		return -1;
	}
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
				throw "Exception: Bad --try_cuda flag value!";
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
				throw "Exception: Incorrect refinement mask length!";
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
				throw "Exception: Bad --wave_correct flag value!";
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
				throw "Exception: Bad exposure compensation method!";
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
				throw "Exception: Bad seam finding method!";
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
				throw "Exception: Bad blending method!";
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
				throw "Exception: Bad timelapse method!";
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
		throw "Exception: Number of images needs to be greater than two!";
	
	return 0;
}

// A recursive binary search function. It returns 
// location of x in given array arr[l..r] is present, 
// otherwise -1
int Pose::binarySearchImageTime(int l, int r, double x)
{
	if (r >= l)
	{
        int mid = l + (r - l)/2;
        //cout << "mid " << mid << " images_times_data[mid][0] " << images_times_data[mid][0] << " x-images_times_data[mid][0] " << x - images_times_data[mid][0] << endl;
 
        // If the element is present at the middle 
        // itself
        if(images_times_data[mid][0] == x)
			return mid;
		
        // If element is smaller than mid, then 
        // it can only be present in left subarray
        if (images_times_data[mid][0] > x)
            return binarySearchImageTime(l, mid-1, x);
 
        // Else the element can only be present
        // in right subarray
        return binarySearchImageTime(mid+1, r, x);
   }
 
   // We reach here when element is not 
   // present in array
   throw "Exception: binarySearchImageTime: unsuccessful search!";
   return -1;
}

// A recursive binary search function. It returns 
// location of x in given array arr[l..r] is present, 
// otherwise -1
int Pose::binarySearch(vector<double> seq, int l, int r, double x)
{
	if (r >= l)
	{
        int mid = l + (r - l)/2;
        //cout << "mid " << mid << " seq[mid] " << seq[mid] << " x-seq[mid] " << x - seq[mid] << endl;
 
        // If the element is present at the middle 
        // itself
        if(mid > 0 && mid < seq.size()-1)
        {
			if (seq[mid-1] < x && seq[mid+1] > x)	//if (arr[mid] == x)
				return mid;
		}
		else if(mid == 0)
		{
			return 0;
		}
		else if(mid == seq.size()-1)
		{
			return seq.size()-1;
		}
		else
		{
			throw "Exception: binarySearch: This should not be hit!";
		}
		
        // If element is smaller than mid, then 
        // it can only be present in left subarray
        if (seq[mid] > x)
            return binarySearch(seq, l, mid-1, x);
 
        // Else the element can only be present
        // in right subarray
        return binarySearch(seq, mid+1, r, x);
   }
 
   // We reach here when element is not 
   // present in array
   throw "Exception: binarySearch: unsuccessful search!";
   return -1;
}

void Pose::readCalibFile()
{
	cv::FileStorage fs(dataFilesPrefix + calib_file, cv::FileStorage::READ);
	fs["Q"] >> Q;
	cout << "Q: " << Q << endl;
	if(Q.empty())
		throw "Exception: could not read Q matrix";
	fs.release();
	cout << "read calib file." << endl;
}

void Pose::readPoseFile()
{
	//read pose file
	// Here is the file containing the data. Read it into data.
	//pose_data
	ifstream data_file;
	data_file.open(dataFilesPrefix + pose_file);
	data_file >> pose_data;
	// Complain if something went wrong.
	if (!data_file.eof())
		throw "Exception: Could not open pose_data file!";
	data_file.close();
	
	for (int i = 0; i < pose_data.size(); i++)
		pose_times_seq.push_back(pose_data[i][2]);
	
	//images_times_data
	data_file.open(dataFilesPrefix + images_times_file);
	data_file >> images_times_data;
	// Complain if something went wrong.
	if (!data_file.eof())
		throw "Exception: Could not open images_times_data file!";
	data_file.close();
	
	for (int i = 0; i < images_times_data.size(); i++)
		images_times_seq.push_back(images_times_data[i][2]);
	
	//heading_data
	data_file.open(dataFilesPrefix + heading_data_file);
	data_file >> heading_data;
	// Complain if something went wrong.
	if (!data_file.eof())
		throw "Exception: Could not open heading_data file!";
	data_file.close();
	
	for (int i = 0; i < heading_data.size(); i++)
		heading_times_seq.push_back(heading_data[i][2]);
	
	cout << "Your images_times file contains " << images_times_data.size() << " records.\n";
	cout << "Your pose_data file contains " << pose_data.size() << " records.\n";
	cout << "Your heading_data file contains " << heading_data.size() << " records.\n";
	
	//SEARCH PROCESS: get NSECS from images_times_data and search for corresponding or nearby entry in pose_data and heading_data
	data_index_finder(1248);
}

int* Pose::data_index_finder(int image_number)
{
	//SEARCH PROCESS: get NSECS from images_times_data and search for corresponding or nearby entry in pose_data and heading_data
	int image_time_index = binarySearchImageTime(0, images_times_seq.size()-1, (double)image_number);
	cout << fixed <<  "image_number: " << image_number << " image_time_index: " << image_time_index << " time: " << images_times_seq[image_time_index] << endl;
	int pose_index = binarySearch(pose_times_seq, 0, pose_times_seq.size()-1, images_times_seq[image_time_index]);
	(pose_index == -1)? printf("pose_index is not found\n") : printf("pose_index: %d\n", pose_index);
	int heading_index = binarySearch(heading_times_seq, 0, heading_times_seq.size()-1, images_times_seq[image_time_index]);
	(heading_index == -1)? printf("heading_index is not found\n") : printf("heading_index: %d\n", heading_index);
	
	cout << "pose: ";
	for (int i = 0; i < 9; i++)
		cout << fixed << " " << pose_data[pose_index][i];
	cout << "\nheading: ";
	for (int i = 0; i < 3; i++)
		cout << fixed << " " << heading_data[heading_index][i];
	cout << "\n";
	int arr[2] = {pose_index, heading_index};
	int* arr_ptr = &(arr[0]);
	return arr_ptr;
}

void Pose::readImages()
{
	images = vector<Mat>(img_numbers.size());
	full_images = vector<Mat>(img_numbers.size());
	disparity_images = vector<Mat>(img_numbers.size());
	full_img_sizes = vector<Size>(img_numbers.size());
	//namedWindow( "Display window", WINDOW_AUTOSIZE );// Create a window for display.
	if(use_segment_labels)
	{
		segment_maps = vector<Mat>(img_numbers.size());
	}
	save_graph_to = "output/graph";
	//logging stuff
	if(log_stuff)
		f.open(save_graph_to.c_str(), ios::out);
	
	for (int i = 0; i < img_numbers.size(); ++i)
	{
		cout << img_numbers[i] << endl;
		full_img = imread(imagePrefix + to_string(img_numbers[i]) + ".png");
		if(full_img.empty())
			throw "Exception: cannot read full_img!";
		
		//imshow( "Display window", full_img );                   // Show our image inside it.
		//waitKey(0);                                          // Wait for a keystroke in the window
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
			//read segmentation map
			Mat segment_img(rows,cols, CV_8UC1);
			segment_img = imread(segmentlblPrefix + to_string(img_numbers[i]) + ".png",CV_LOAD_IMAGE_GRAYSCALE);
			segment_maps[i] = segment_img;
			if(segment_img.empty())
				throw "Exception: cannot read segment map!";
			
			//read disparity image
			Mat disp_img(rows,cols, CV_8UC1);
			disp_img = imread(disparityPrefix + to_string(img_numbers[i]) + ".png",CV_LOAD_IMAGE_GRAYSCALE);
			disparity_images[i] = disp_img;
			if(disp_img.empty())
				throw "Exception: Cannot read disparity image!";
			//imshow( "Display window", disp_img );                   // Show our image inside it.
			//waitKey(0);                                          // Wait for a keystroke in the window
		}
		else
		{
			//read disparity image
			Mat disp_img(rows,cols, CV_8UC1);
			disp_img = imread(disparityPrefix + to_string(img_numbers[i]) + ".png",CV_LOAD_IMAGE_GRAYSCALE);
			disparity_images[i] = disp_img;
		}
		
		
		if (full_img.empty())
			throw "Exception: Can't read image!";
		
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

void Pose::findFeatures()
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
		throw "Exception: Unknown 2D features type!";
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

void Pose::pairWiseMatching()
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

void Pose::createPlaneFittedDisparityImages()
{
	for (int i = 0; i < segment_maps.size(); i++)
	{
		//cout << "Image" << i << endl;
		Mat segment_img = segment_maps[i];
		
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
			//cout << "cluster" << cluster << " size:" << Xc.size();
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
			//cout << " Accepted points: " << Xp.size() << endl;
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

void Pose::createPtCloud(int img_index, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudrgb, pcl::PointCloud<pcl::PointXYZ>::Ptr cloudxyz)
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
			//cout << "y " << y << " x " << x << " disp_img.at<uint16_t>(y,x) " << disp_img.at<uint16_t>(y,x) << endl;
			//cout << " disp_img.at<double>(y,x) " << disp_img.at<double>(y,x) << endl;
			if(use_segment_labels)
				disp_val = disp_img.at<double>(y,x);		//disp_val = (double)disp_img.at<uint16_t>(y,x) / 200.0;
			else
				disp_val = (double)disp_img.at<uchar>(y,x);
			//cout << "disp_val " << disp_val << endl;
			
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
				//cout << (uint32_t)color[2] << " " << (uint32_t)color[1] << " " << (uint32_t)color[0] << endl;
				
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

pcl::registration::TransformationEstimation<pcl::PointXYZRGB, pcl::PointXYZRGB>::Matrix4 Pose::generateTmat(record_t pose)
{
	double tx = pose[tx_ind];
	double ty = pose[ty_ind];
	double tz = pose[tz_ind];
	double qx = pose[qx_ind];
	double qy = pose[qy_ind];
	double qz = pose[qz_ind];
	double qw = pose[qw_ind];
	
	double sqw = qw*qw;
	double sqx = qx*qx;
	double sqy = qy*qy;
	double sqz = qz*qz;
	
	if(sqw + sqx + sqy + sqz < 0.99 || sqw + sqx + sqy + sqz > 1.01)
		throw "Exception: Sum of squares of quaternion values should be 1! i.e., quaternion should be homogeneous!";
	
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
	
	pcl::registration::TransformationEstimation<pcl::PointXYZRGB, pcl::PointXYZRGB>::Matrix4 r_wh;
	
	for (int i = 0; i < 3; i++)
		for (int j = 0; j < 3; j++)
			r_wh(i,j) = rot.at<double>(i,j);
	r_wh(3,0) = r_wh(3,1) = r_wh(3,2) = r_wh(0,3) = r_wh(1,3) = r_wh(2,3) = 0.0;
	r_wh(3,3) = 1.0;
	//t_wh(0,3) = tx - tx * t_wh(0,0) - ty * t_wh(0,1) - tz * t_wh(0,2);
	//t_wh(1,3) = ty - tx * t_wh(1,0) - ty * t_wh(1,1) - tz * t_wh(1,2);
	//t_wh(2,3) = tz - tx * t_wh(2,0) - ty * t_wh(2,1) - tz * t_wh(2,2);
	
	pcl::registration::TransformationEstimation<pcl::PointXYZRGB, pcl::PointXYZRGB>::Matrix4 t_wh;
	for (int i = 0; i < 4; i++)
		for (int j = 0; j < 4; j++)
			t_wh(i,j) = 0;
	t_wh(0,0) = t_wh(1,1) = t_wh(2,2) = 1.0;
	t_wh(0,3) = tx;
	t_wh(1,3) = ty;
	t_wh(2,3) = tz;
	t_wh(3,3) = 1.0;
	
	pcl::registration::TransformationEstimation<pcl::PointXYZRGB, pcl::PointXYZRGB>::Matrix4 r_xi;
	for (int i = 0; i < 4; i++)
		for (int j = 0; j < 4; j++)
			r_xi(i,j) = 0;
	r_xi(0,0) = r_xi(1,1) = r_xi(2,2) = 1.0;
	r_xi(3,3) = 1.0;
	r_xi(1,1) = cos(-theta_xi);
	r_xi(1,2) = -sin(-theta_xi);
	r_xi(2,1) = sin(-theta_xi);
	r_xi(2,2) = cos(-theta_xi);
	
	pcl::registration::TransformationEstimation<pcl::PointXYZRGB, pcl::PointXYZRGB>::Matrix4 r_yi;
	for (int i = 0; i < 4; i++)
		for (int j = 0; j < 4; j++)
			r_yi(i,j) = 0;
	r_yi(0,0) = r_yi(1,1) = r_yi(2,2) = 1.0;
	r_yi(3,3) = 1.0;
	r_yi(0,0) = cos(-theta_yi);
	r_yi(0,2) = sin(-theta_yi);
	r_yi(2,0) = -sin(-theta_yi);
	r_yi(2,2) = cos(-theta_yi);
	
	pcl::registration::TransformationEstimation<pcl::PointXYZRGB, pcl::PointXYZRGB>::Matrix4 r_invert_i;
	for (int i = 0; i < 4; i++)
		for (int j = 0; j < 4; j++)
			r_invert_i(i,j) = 0;
	r_invert_i(3,3) = 1.0;
	//invert y and z
	r_invert_i(0,0) = 1.0;	//x
	r_invert_i(1,1) = -1.0;	//y
	r_invert_i(2,2) = -1.0;	//z
	
	pcl::registration::TransformationEstimation<pcl::PointXYZRGB, pcl::PointXYZRGB>::Matrix4 t_hi;
	for (int i = 0; i < 4; i++)
		for (int j = 0; j < 4; j++)
			t_hi(i,j) = 0;
	t_hi(0,0) = t_hi(1,1) = t_hi(2,2) = 1.0;
	t_hi(0,3) = -trans_x_hi;
	t_hi(1,3) = -trans_y_hi;
	t_hi(2,3) = -trans_z_hi;
	t_hi(3,3) = 1.0;
	
	pcl::registration::TransformationEstimation<pcl::PointXYZRGB, pcl::PointXYZRGB>::Matrix4 r_flip_xy;
	for (int i = 0; i < 4; i++)
		for (int j = 0; j < 4; j++)
			r_flip_xy(i,j) = 0;
	r_flip_xy(3,3) = 1.0;
	//flip x and y
	r_flip_xy(1,0) = 1.0;	//x
	r_flip_xy(0,1) = 1.0;	//y
	r_flip_xy(2,2) = 1.0;	//z
	
	pcl::registration::TransformationEstimation<pcl::PointXYZRGB, pcl::PointXYZRGB>::Matrix4 r_invert_y;
	for (int i = 0; i < 4; i++)
		for (int j = 0; j < 4; j++)
			r_invert_y(i,j) = 0;
	r_invert_y(3,3) = 1.0;
	//invert y
	r_invert_y(0,0) = 1.0;	//x
	r_invert_y(1,1) = -1.0;	//y
	r_invert_y(2,2) = 1.0;	//z
	
	pcl::registration::TransformationEstimation<pcl::PointXYZRGB, pcl::PointXYZRGB>::Matrix4 t_mat = t_wh * r_wh * r_invert_y * r_flip_xy * t_hi * r_invert_i * r_yi * r_xi;
	
	cout << "r_xi:\n" << r_xi << endl;
	cout << "r_yi:\n" << r_yi << endl;
	cout << "r_invert_i:\n" << r_invert_i << endl;
	cout << "t_hi:\n" << t_hi << endl;
	cout << "r_flip_xy:\n" << r_flip_xy << endl;
	cout << "r_invert_y:\n" << r_invert_y << endl;
	cout << "r_wh:\n" << r_wh << endl;
	cout << "t_wh:\n" << t_wh << endl;
	cout << "t_mat:\n" << t_mat << endl;
	
	return t_mat;
}

void Pose::transformPtCloud2(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudrgb, pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformed_cloudrgb, pcl::registration::TransformationEstimation<pcl::PointXYZRGB, pcl::PointXYZRGB>::Matrix4 transform)
{
	// Executing the transformation
	pcl::transformPointCloud(*cloudrgb, *transformed_cloudrgb, transform);
}

void Pose::transformPtCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudrgb, pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformed_cloudrgb, Eigen::Affine3f transform_2)
{
	// Executing the transformation
	pcl::transformPointCloud(*cloudrgb, *transformed_cloudrgb, transform_2);
}
