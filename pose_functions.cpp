//#include "pose.h"

void Pose::printUsage()
{
	cout <<
		"Process:"
		"\n  1. extract 2D features from images"
		"\n  2. find pairwise feature correspondence"
		"\n  3. convert corresponding features to 3D using disparity image information"
		"\n  4. find transformation between corresponding 3D points using rigid body transformation over multiple images"
		"\n  5. overlay point clouds using the calculated transformation"
		"\n  6. do a correction step by fitting calculated camera positions over recorded camera positions using ICP"
		"\n  7. use this transformation to transform the point cloud to make it near to ground truth"
		"\n\nNotes:"
		"\n  - image numbers to be supplied either using command line or in the file " << imageNumbersFile <<
		"\n  - Usage to supply images from command line ./pose img1 img2 [...imgN] [flags]"
		"\n  - camera calib file 'cam13calib.yml' needs to be supplied at data_files folder"
		"\n  - image trigger times file needs to be kept at data_files/images.txt"
		"\n  - pose information file needs to be kept at data_files/pose.txt"
		"\n  - left cam images will be read from " << imagePrefix <<
		"\n  - disparity images will be read from " << disparityPrefix <<
		"\n  - segmented label maps will be read from " << segmentlblPrefix <<
		"\n\nFlags:"
		"\n  --use_segment_labels"
		"\n      Use pre-made segmented labels for every image to improve resolution of disparity images"
		"\n  --jump_pixels [int]"
		"\n      number of pixels to jump on in each direction to make semi-dense 3D reconstruction"
		"\n      to get sparse 3D reconstruction put this as 0"
		"\n  --seq_len [int]"
		"\n      number of images to consider during online visualization cycles. Enter -1 to have a single cycle run"
		"\n  --max_depth [double]"
		"\n      max depth in field. Will remove points below this or outliers. Enter positive depth value."
		"\n  --max_height [double]"
		"\n      max height in field. Will remove points above this or outliers."
		"\n  --range_width [int]"
		"\n      Number of nearby images to do pairwise matching on for every image"
		"\n  --dist_nearby [double]"
		"\n      Max allowable distance of hex position image to check for pairwise matching"
		"\n  --preview"
		"\n      visualize generated point cloud at the end"
		"\n  --visualize [Pt Cloud filename]"
		"\n      Visualize a given point cloud"
		"\n  --segment_map"
		"\n      To create segmented map with excluded obstacles and area convex hull"
		"\n  --segment_map_details [segment_dist_threashold_float] [convexhull_dist_threshold_float] [convexhull_alpha_float]"
		"\n      To create segmented map with excluded obstacles and area convex hull"
		"\n  --displayUAVPositions [Pt Cloud filename]"
		"\n      Display hexacopter positions along with point cloud during visualization"
		"\n  --align_point_cloud [Pt Cloud 1] [Pt Cloud 2]"
		"\n      Align point clouds using ICP algorithm"
		"\n  --voxel_size [float]"
		"\n      Voxel size in m to find average value of points for downsampling"
		"\n  --min_points_per_voxel [int]"
		"\n      Set minimum number of points required during downsampling a voxel of combined point cloud, not single image point cloud"
		"\n  --blur_kernel [int]"
		"\n      Blur kernel size to blur disparity image to reject outliers. Current implementation is of either median blur or bilateral blur"
		"\n  --downsample [Pt Cloud file name] (optional)--voxel_size [float]"
		"\n      Downsample a point cloud along with optional voxel size in meters"
		"\n  --smooth_surface [Pt Cloud file name] (optional)--search_radius [float]"
		"\n      Smooth surface"
		"\n  --mesh_surface [Pt Cloud file name]"
		"\n      Mesh surface using triangulation"
		"\n  --log 0/1"
		"\n      log most things in log.txt file. Default true. Enter 0 to stop logging."
		<< endl;
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
		else if (string(argv[i]) == "--visualize")
		{
			visualize = true;
			n_imgs = 1;		//just to stop program from reading images.txt file
			read_PLY_filename0 = string(argv[i + 1]);
			cout << "Visualize " << read_PLY_filename0 << endl;
			i++;
		}
		else if (string(argv[i]) == "--align_point_cloud")
		{
			align_point_cloud = true;
			n_imgs = 1;		//just to stop program from reading images.txt file
			read_PLY_filename0 = string(argv[++i]);
			read_PLY_filename1 = string(argv[++i]);
			cout << "Align " << read_PLY_filename0 << " and " << read_PLY_filename1 << " using ICP." << endl;
		}
		else if (string(argv[i]) == "--smooth_surface")
		{
			smooth_surface = true;
			n_imgs = 1;		//just to stop program from reading images.txt file
			read_PLY_filename0 = string(argv[i + 1]);
			cout << "smooth_surface " << read_PLY_filename0 << endl;
			i++;
		}
		else if (string(argv[i]) == "--mesh_surface")
		{
			mesh_surface = true;
			n_imgs = 1;		//just to stop program from reading images.txt file
			read_PLY_filename0 = string(argv[i + 1]);
			cout << "mesh_surface " << read_PLY_filename0 << endl;
			i++;
		}
		else if (string(argv[i]) == "--downsample")
		{
			downsample = true;
			n_imgs = 1;		//just to stop program from reading images.txt file
			read_PLY_filename0 = string(argv[i + 1]);
			cout << "Downsample " << read_PLY_filename0 << endl;
			i++;
		}
		else if (string(argv[i]) == "--displayUAVPositions")
		{
			displayUAVPositions = true;
			n_imgs = 1;		//just to stop program from reading images.txt file
			read_PLY_filename1 = string(argv[i + 1]);
			i++;
			cout << "displayUAVPositions " << read_PLY_filename1 << endl;
		}
		else if (string(argv[i]) == "--voxel_size")
		{
			voxel_size = atof(argv[i + 1]);
			cout << "voxel_size " << voxel_size << endl;
			i++;
		}
		else if (string(argv[i]) == "--min_points_per_voxel")
		{
			min_points_per_voxel = atoi(argv[i + 1]);
			cout << "min_points_per_voxel " << min_points_per_voxel << endl;
			i++;
		}
		else if (string(argv[i]) == "--dist_nearby")
		{
			dist_nearby = atof(argv[i + 1]);
			cout << "dist_nearby " << dist_nearby << endl;
			i++;
		}
		else if (string(argv[i]) == "--blur_kernel")
		{
			blur_kernel = atoi(argv[i + 1]);
			cout << "blur_kernel " << blur_kernel << endl;
			i++;
		}
		else if (string(argv[i]) == "--segment_map")
		{
			segment_map = true;
			cout << "segment_map" << endl;
		}
		else if (string(argv[i]) == "--segment_map_details")
		{
			segment_map = true;
			segment_dist_threashold = atof(argv[++i]);
			convexhull_dist_threshold = atof(argv[++i]);
			convexhull_alpha = atof(argv[++i]);
			cout << "segment_map" << endl;
			cout << "segment_dist_threashold " << segment_dist_threashold << endl;
			cout << "convexhull_dist_threshold " << convexhull_dist_threshold << endl;
			cout << "convexhull_alpha " << convexhull_alpha << endl;
		}
		else if (string(argv[i]) == "--search_radius")
		{
			search_radius = atof(argv[i + 1]);
			cout << "search_radius " << search_radius << endl;
			i++;
		}
		else if (string(argv[i]) == "--seq_len")
		{
			seq_len = atoi(argv[i + 1]);
			cout << "seq_len " << seq_len << endl;
			if (seq_len == 0 || seq_len < -1)
				throw "Exception: invalid seq_len value!";
			
			if(seq_len != -1)
			{
				online = true;
				cout << "online 3D reconstruction" << endl;
			}
			i++;
		}
		else if (string(argv[i]) == "--jump_pixels")
		{
			jump_pixels = atoi(argv[i + 1]);
			cout << "jump_pixels " << jump_pixels << endl;
			i++;
		}
		else if (string(argv[i]) == "--test")
		{
			release = false;
			cout << "test version, don't check variance of disparity images to speed up development" << endl;
		}
		else if (string(argv[i]) == "--range_width")
		{
			range_width = atoi(argv[i + 1]);
			cout << "range_width " << range_width << endl;
			i++;
		}
		else if (string(argv[i]) == "--log")
		{
			int log_val = atoi(argv[i + 1]);
			i++;
			if (log_val == 0) log_stuff = false;
			else log_stuff = true;
			cout << "log " << log_stuff << endl;
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
		else if (string(argv[i]) == "--features")
		{
			features_type = argv[i + 1];
			if (features_type == "orb")
				match_conf = 0.3f;
			i++;
		}
		else
		{
			img_numbers.push_back(atoi(argv[i]));
			cout << atoi(argv[i]) << endl;
			++n_imgs;
		}
	}
	if (n_imgs == 0)
	{
		ifstream images_file;
		images_file.open(imageNumbersFile);
		if(images_file.is_open())
		{
			string line;
			while (getline( images_file, line ))
			{
				stringstream fs( line );
				int img_num = 0;  // (default value is 0.0)
				fs >> img_num;
				img_numbers.push_back(img_num);
				++n_imgs;
				cout << img_num << " ";
			}
			images_file.close();
			cout << "\nRead " << n_imgs << " image numbers from " << imageNumbersFile << endl;
		}
		else
			throw "Exception: Unable to open imageNumbersFile!";
	}
	
	return 0;
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

// Get current date/time, format is YYYY-MM-DD.HH:mm:ss
const string Pose::currentDateTime()
{
    time_t     now = time(0);
    struct tm  tstruct;
    char       buf[80];
    tstruct = *localtime(&now);
    // Visit http://en.cppreference.com/w/cpp/chrono/c/strftime
    // for more information about date/time format
    strftime(buf, sizeof(buf), "%Y-%m-%d.%X", &tstruct);

    return buf;
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
		data.push_back( record );
	
	// Again, return the argument stream as required for this kind of input stream overload.
	return ins;  
}

// A recursive binary search function. It returns 
// location of x in given array arr[l..r] is present, 
// otherwise -1
int Pose::binarySearchImageTime(int l, int r, int imageNumber)
{
	if (r >= l)
	{
        int mid = l + (r - l)/2;
 
        // If the element is present at the middle
        if((int)images_times_data[mid][0] == imageNumber)
			return mid;
		
        // If element is smaller than mid, then it can only be present in left subarray
        if ((int)images_times_data[mid][0] > imageNumber)
            return binarySearchImageTime(l, mid-1, imageNumber);
 
        // Else the element can only be present in right subarray
        return binarySearchImageTime(mid+1, r, imageNumber);
   }
 
   // We reach here when element is not present in array
   throw "Exception: binarySearchImageTime: unsuccessful search!";
   return -1;
}

// A recursive binary search function. It returns 
// location of x in given array arr[l..r] is present, 
// otherwise -1
int Pose::binarySearchUsingTime(vector<double> seq, int l, int r, double time)
{
	if (r >= l)
	{
        int mid = l + (r - l)/2;
 
        // If the element is present at the middle
        if(mid > 0 && mid < seq.size()-1)
        {
			if (seq[mid-1] < time && seq[mid+1] > time)
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
			throw "Exception: binarySearchUsingTime: This should not be hit!";
		}
		
        // If element is smaller than mid, then it can only be present in left subarray
        if (seq[mid] > time)
            return binarySearchUsingTime(seq, l, mid-1, time);
 
        // Else the element can only be present in right subarray
        return binarySearchUsingTime(seq, mid+1, r, time);
   }
 
   // We reach here when element is not present in array
   throw "Exception: binarySearchUsingTime: unsuccessful search!";
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
	
	cout << "Your images_times file contains " << images_times_data.size() << " records.\n";
	cout << "Your pose_data file contains " << pose_data.size() << " records.\n";
	//cout << "Your heading_data file contains " << heading_data.size() << " records.\n";
}

int Pose::data_index_finder(int image_number)
{
	//SEARCH PROCESS: get time NSECS from images_times_data and search for corresponding or nearby entry in pose_data and heading_data
	int image_time_index = binarySearchImageTime(0, images_times_seq.size()-1, image_number);
	//cout << fixed <<  "image_number: " << image_number << " image_time_index: " << image_time_index << " time: " << images_times_seq[image_time_index] << endl;
	
	int pose_index = binarySearchUsingTime(pose_times_seq, 0, pose_times_seq.size()-1, images_times_seq[image_time_index]);
	//(pose_index == -1)? printf("pose_index is not found\n") : printf("pose_index: %d\n", pose_index);
	
	//cout << "pose: ";
	//for (int i = 0; i < 9; i++)
	//	cout << fixed << " " << pose_data[pose_index][i];
	return pose_index;
}

void Pose::readImage(int i)
{
	full_images[i] = imread(imagePrefix + to_string(img_numbers[i]) + ".png");
	if(full_images[i].empty())
		throw "Exception: cannot read full_img " + to_string(img_numbers[i]) + "!";
	//full_images[i] = full_image;
	cout << " i" << to_string(img_numbers[i]) << " " << std::flush;
}

void Pose::populateImages(int start_index, int end_index)
{
	for (int i = start_index; i <= end_index; i++)
	{
		readImage(i);
	}
}

void Pose::readDisparityImage(int i)
{
	Mat disp_img = imread(disparityPrefix + to_string(img_numbers[i]) + ".png",CV_LOAD_IMAGE_GRAYSCALE);
	if(disp_img.empty())
		throw "Exception: cannot read disp_image " + to_string(img_numbers[i]) + "!";
	
	if(blur_kernel > 1)
	{
		//blur the disparity image to remove noise
		Mat disp_img_blurred;
		//bilateralFilter ( disp_img, disp_img_blurred, blur_kernel, blur_kernel*2, blur_kernel/2 );
		medianBlur ( disp_img, disp_img_blurred, blur_kernel );
		disparity_images[i] = disp_img_blurred;
	}
	else
	{
		disparity_images[i] = disp_img;
	}
	
	if (release)
	{
		double disp_img_var = getVariance(disparity_images[i], false);
		//cout << img_numbers[i] << " disp_img_var " << disp_img_var << "\t";
		//log_file << img_numbers[i] << " disp_img_var " << disp_img_var << "\t";
		if (disp_img_var > 5)
			throw "Exception: disp_image " + to_string(img_numbers[i]) + " > 5. Unacceptable disparity image.";
	}
	
	cout << " d" << to_string(img_numbers[i]) << " " << std::flush;
}

void Pose::populateDisparityImages(int start_index, int end_index)
{
	for (int i = start_index; i <= end_index; i++)
	{
		readDisparityImage(i);
	}
}

void Pose::readSegmentLabelMap(int i)
{
	segment_maps[i] = imread(segmentlblPrefix + to_string(img_numbers[i]) + ".png",CV_LOAD_IMAGE_GRAYSCALE);
	if(segment_maps[i].empty())
		throw "Exception: cannot read segment_label_map " + to_string(img_numbers[i]) + "!";
	
	cout << " s" << to_string(img_numbers[i]) << " " << std::flush;
}

void Pose::populateSegmentLabelMaps(int start_index, int end_index)
{
	for (int i = start_index; i <= end_index; i++)
	{
		readSegmentLabelMap(i);
	}
}

void Pose::populateDoubleDispImages(int start_index, int end_index)
{
	for (int i = start_index; i <= end_index; i++)
	{
		createPlaneFittedDisparityImages(i);
	}
}

void Pose::populateData()
{
	full_images = vector<Mat>(img_numbers.size());
	disparity_images = vector<Mat>(img_numbers.size());
	
	//logging stuff
	if(log_stuff)
		log_file.open(save_log_to.c_str(), ios::out);
	
	//test load an image to fix rows, cols and cols_start_aft_cutout
	Mat test_load_img = imread(imagePrefix + to_string(img_numbers[0]) + ".png");
	rows = test_load_img.rows;
	cols = test_load_img.cols;
	cols_start_aft_cutout = (int)(cols/cutout_ratio);
	
	//4 cores, 2 threads per core, total 8 threads, 1 thread being used by main program, 7 addditional threads can be used here
	const int divisions = 7;
	const int imgs_per_division = ceil(1.0 * img_numbers.size() / divisions);
	int i = 0;
	cout << "\nMulti-threading sequences:" << endl;
	cout << "divisions " << divisions << " imgs_per_division " << imgs_per_division << endl;
	cout << i+1 << " : " << i * imgs_per_division << " to " << (i+1) * imgs_per_division - 1 << endl; i++;
	cout << i+1 << " : " << i * imgs_per_division << " to " << (i+1) * imgs_per_division - 1 << endl; i++;
	cout << i+1 << " : " << i * imgs_per_division << " to " << (i+1) * imgs_per_division - 1 << endl; i++;
	cout << i+1 << " : " << i * imgs_per_division << " to " << (i+1) * imgs_per_division - 1 << endl; i++;
	cout << i+1 << " : " << i * imgs_per_division << " to " << (i+1) * imgs_per_division - 1 << endl; i++;
	cout << i+1 << " : " << i * imgs_per_division << " to " << (i+1) * imgs_per_division - 1 << endl; i++;
	cout << i+1 << " : " << i * imgs_per_division << " to " << min((i+1) * imgs_per_division - 1, (int)(img_numbers.size()) - 1) << endl;
	
	cout << "\nReading images using multithreading" << endl;
	i = 0;
	boost::thread img_thread1(&Pose::populateImages, this, i * imgs_per_division, (i+1) * imgs_per_division - 1); i++;
	boost::thread img_thread2(&Pose::populateImages, this, i * imgs_per_division, (i+1) * imgs_per_division - 1); i++;
	boost::thread img_thread3(&Pose::populateImages, this, i * imgs_per_division, (i+1) * imgs_per_division - 1); i++;
	boost::thread img_thread4(&Pose::populateImages, this, i * imgs_per_division, (i+1) * imgs_per_division - 1); i++;
	boost::thread img_thread5(&Pose::populateImages, this, i * imgs_per_division, (i+1) * imgs_per_division - 1); i++;
	boost::thread img_thread6(&Pose::populateImages, this, i * imgs_per_division, (i+1) * imgs_per_division - 1); i++;
	boost::thread img_thread7(&Pose::populateImages, this, i * imgs_per_division, min((i+1) * imgs_per_division - 1, (int)(img_numbers.size()) - 1));
	
	//cout << "\nReading disparity images using multithreading" << endl;
	i = 0;
	boost::thread disp_thread1(&Pose::populateDisparityImages, this, i * imgs_per_division, (i+1) * imgs_per_division - 1); i++;
	boost::thread disp_thread2(&Pose::populateDisparityImages, this, i * imgs_per_division, (i+1) * imgs_per_division - 1); i++;
	boost::thread disp_thread3(&Pose::populateDisparityImages, this, i * imgs_per_division, (i+1) * imgs_per_division - 1); i++;
	boost::thread disp_thread4(&Pose::populateDisparityImages, this, i * imgs_per_division, (i+1) * imgs_per_division - 1); i++;
	boost::thread disp_thread5(&Pose::populateDisparityImages, this, i * imgs_per_division, (i+1) * imgs_per_division - 1); i++;
	boost::thread disp_thread6(&Pose::populateDisparityImages, this, i * imgs_per_division, (i+1) * imgs_per_division - 1); i++;
	boost::thread disp_thread7(&Pose::populateDisparityImages, this, i * imgs_per_division, min((i+1) * imgs_per_division - 1, (int)(img_numbers.size()) - 1));
	
	if(use_segment_labels)
	{
		segment_maps = vector<Mat>(img_numbers.size());
		//cout << "\nReading segment label map images using multithreading" << endl;
		i = 0;
		boost::thread segment_map_thread1(&Pose::populateSegmentLabelMaps, this, i * imgs_per_division, (i+1) * imgs_per_division - 1); i++;
		boost::thread segment_map_thread2(&Pose::populateSegmentLabelMaps, this, i * imgs_per_division, (i+1) * imgs_per_division - 1); i++;
		boost::thread segment_map_thread3(&Pose::populateSegmentLabelMaps, this, i * imgs_per_division, (i+1) * imgs_per_division - 1); i++;
		boost::thread segment_map_thread4(&Pose::populateSegmentLabelMaps, this, i * imgs_per_division, (i+1) * imgs_per_division - 1); i++;
		boost::thread segment_map_thread5(&Pose::populateSegmentLabelMaps, this, i * imgs_per_division, (i+1) * imgs_per_division - 1); i++;
		boost::thread segment_map_thread6(&Pose::populateSegmentLabelMaps, this, i * imgs_per_division, (i+1) * imgs_per_division - 1); i++;
		boost::thread segment_map_thread7(&Pose::populateSegmentLabelMaps, this, i * imgs_per_division, min((i+1) * imgs_per_division - 1, (int)(img_numbers.size()) - 1));
		segment_map_thread1.join();
		segment_map_thread2.join();
		segment_map_thread3.join();
		segment_map_thread4.join();
		segment_map_thread5.join();
		segment_map_thread6.join();
		segment_map_thread7.join();
	}
	img_thread1.join();
	img_thread2.join();
	img_thread3.join();
	img_thread4.join();
	img_thread5.join();
	img_thread6.join();
	img_thread7.join();
	disp_thread1.join();
	disp_thread2.join();
	disp_thread3.join();
	disp_thread4.join();
	disp_thread5.join();
	disp_thread6.join();
	disp_thread7.join();
	
	if(use_segment_labels)
	{
		double_disparity_images = vector<Mat>(img_numbers.size());
		cout << "\n\nPopulating Double Disp Images using multithreading" << endl;
		i = 0;
		boost::thread double_disp_thread1(&Pose::populateDoubleDispImages, this, i * imgs_per_division, (i+1) * imgs_per_division - 1); i++;
		boost::thread double_disp_thread2(&Pose::populateDoubleDispImages, this, i * imgs_per_division, (i+1) * imgs_per_division - 1); i++;
		boost::thread double_disp_thread3(&Pose::populateDoubleDispImages, this, i * imgs_per_division, (i+1) * imgs_per_division - 1); i++;
		boost::thread double_disp_thread4(&Pose::populateDoubleDispImages, this, i * imgs_per_division, (i+1) * imgs_per_division - 1); i++;
		boost::thread double_disp_thread5(&Pose::populateDoubleDispImages, this, i * imgs_per_division, (i+1) * imgs_per_division - 1); i++;
		boost::thread double_disp_thread6(&Pose::populateDoubleDispImages, this, i * imgs_per_division, (i+1) * imgs_per_division - 1); i++;
		boost::thread double_disp_thread7(&Pose::populateDoubleDispImages, this, i * imgs_per_division, min((i+1) * imgs_per_division - 1, (int)(img_numbers.size()) - 1)); i++;
		double_disp_thread1.join();
		double_disp_thread2.join();
		double_disp_thread3.join();
		double_disp_thread4.join();
		double_disp_thread5.join();
		double_disp_thread6.join();
		double_disp_thread7.join();
		cout << endl;
		
		//cout << "min((i+1) * imgs_per_division - 1, (int)(img_numbers.size()) - 1) "  << min((i+1) * imgs_per_division - 1, (int)(img_numbers.size()) - 1) << endl;
	}
	
	//for (int i = 0; i < 2; i++)
	//{
	//	imshow( "full_images", full_images[i] );                   // Show our image inside it.
	//	waitKey(0);                                          // Wait for a keystroke in the window
	//	imshow( "disparity_images", disparity_images[i] );                   // Show our image inside it.
	//	waitKey(0);                                          // Wait for a keystroke in the window
	//	imshow( "segment_maps", segment_maps[i] );                   // Show our image inside it.
	//	waitKey(0);                                          // Wait for a keystroke in the window
	//	imshow( "double_disparity_images", double_disparity_images[i] );                   // Show our image inside it.
	//	waitKey(0);                                          // Wait for a keystroke in the window
	//}
}

void Pose::findFeatures()
{
	cout << "\nFeatures";
	//int i = start_idx;
	//while(i < end_idx + 1)
	//{
	//	boost::thread featureFinder_thread1, featureFinder_thread2, featureFinder_thread3, featureFinder_thread4, featureFinder_thread5, featureFinder_thread6, featureFinder_thread7;
	//	//cout << "d0" << endl;
	//	featureFinder_thread1 = boost::thread(&Pose::findFeaturesActual, this, i, 0);
	//	//cout << "d1" << endl;
	//	if(++i < end_idx + 1) featureFinder_thread2 = boost::thread(&Pose::findFeaturesActual, this, i, 1);
	//	//cout << "d2" << endl;
	//	if(++i < end_idx + 1) featureFinder_thread3 = boost::thread(&Pose::findFeaturesActual, this, i, 2);
	//	//cout << "d3" << endl;
	//	if(++i < end_idx + 1) featureFinder_thread4 = boost::thread(&Pose::findFeaturesActual, this, i, 3);
	//	//cout << "d4" << endl;
	//	if(++i < end_idx + 1) featureFinder_thread5 = boost::thread(&Pose::findFeaturesActual, this, i, 4);
	//	//cout << "d5" << endl;
	//	if(++i < end_idx + 1) featureFinder_thread6 = boost::thread(&Pose::findFeaturesActual, this, i, 5);
	//	//cout << "d6" << endl;
	//	if(++i < end_idx + 1) featureFinder_thread7 = boost::thread(&Pose::findFeaturesActual, this, i, 6);
	//	//cout << "d7" << endl;
	//	//sleep(5);
	//	//cout << "slept enough" << endl;
	//	featureFinder_thread1.join();
	//	featureFinder_thread2.join();
	//	featureFinder_thread3.join();
	//	featureFinder_thread4.join();
	//	featureFinder_thread5.join();
	//	featureFinder_thread6.join();
	//	featureFinder_thread7.join();
	//}
	//for (int i = start_idx; i < end_idx + 1; ++i)
	//{
	//	//findFeaturesActual(i, 0);
	//	//(*finder)(full_images[i], features[i]);
	//	////vector<KeyPoint> keypointsD;
	//	////detector->detect(img,keypointsD,Mat());
	//	//features[i].img_idx = i;
	//	////features[i].keypoints = keypointsD;
	//	//cout << " " << img_numbers[i] << "/" << features[i].keypoints.size() << std::flush;
	//}
	//cout << endl;
	//for (int i = 0; i < 6; i++)
	//{
	//	(finderVec[i])->collectGarbage();
	//}
	finder->collectGarbage();
}

void Pose::findFeatures(int img_idx)
{
	//Ptr<FeaturesFinder> finder = makePtr<OrbFeaturesFinder>();
	(*finder)(full_images[img_idx], features[img_idx]);
	features[img_idx].img_idx = img_idx;
	cout << img_numbers[img_idx] << " features " << features[img_idx].keypoints.size();
	
	cuda::GpuMat descriptor(features[img_idx].descriptors);
	descriptorsVec.push_back(descriptor);
	
	//convert keypoints to 3d for easier estimation of rigid body transform later during pairwise matching
	Mat disp_img;
	if(use_segment_labels)
		disp_img = double_disparity_images[img_idx];
	else
		disp_img = disparity_images[img_idx];
	
	vector<KeyPoint> keypoints = features[img_idx].keypoints;
	
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_current_temp = keypoints3DVec[img_idx];
	vector<bool> goodness;
	
	//int good = 0, bad = 0;
	for (int i = 0; i < keypoints.size(); i++)
	{
		double disp_value;
		if(use_segment_labels)
			disp_value = disp_img.at<double>(keypoints[i].pt.y, keypoints[i].pt.x);
		else
			disp_value = (double)disp_img.at<char>(keypoints[i].pt.y, keypoints[i].pt.x);
	
		cv::Mat_<double> vec_src(4, 1);

		double xs = keypoints[i].pt.x;
		double ys = keypoints[i].pt.y;
		
		vec_src(0) = xs; vec_src(1) = ys; vec_src(2) = disp_value; vec_src(3) = 1;
		vec_src = Q * vec_src;
		vec_src /= vec_src(3);
		
		pcl::PointXYZRGB pt_3d_src;
		
		pt_3d_src.x = vec_src(0);
		pt_3d_src.y = vec_src(1);
		pt_3d_src.z = vec_src(2);

		cloud_current_temp->points.push_back(pt_3d_src);
		
		if (disp_value > minDisparity && keypoints[i].pt.x >= cols_start_aft_cutout)
		{
			//good++;
			goodness.push_back(true);
		}
		else
		{
			//bad++;
			goodness.push_back(false);
		}
	}
	//cout << " g" << good << "/b" << bad << flush;
	keypoints3DGoodnessVec.push_back(goodness);
}

void Pose::pairWiseMatching()
{
	if (range_width == -1)
	{
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
		cout << "\nPairwise matches:" << endl;
		log_file << "\nPairwise matches:" << endl;
		for (int i = 0; i < pairwise_matches.size(); i++)
		{
			log_file << "i" << i << " src " << pairwise_matches[i].src_img_idx << " dst " << pairwise_matches[i].dst_img_idx << " confidence " << pairwise_matches[i].confidence << " inliers " << pairwise_matches[i].inliers_mask.size() << " matches " << pairwise_matches[i].matches.size() << endl;
		}
	}
	
}

void Pose::orbcudaPairwiseMatching()
{
	//unsigned long t_AAtime=0, t_BBtime=0, t_CCtime=0;
	//float t_pt;
	//float t_fpt;
	//t_AAtime = getTickCount(); 
	//cout << "\nOrb cuda Features Finding start.." << endl;
	//
	//Ptr<cuda::ORB> orb = cuda::ORB::create();
	//
	//for (int i = 0; i < img_numbers.size(); i++)
	//{
	//	cv::Mat grayImg;
	//	cv::cvtColor(full_images[i], grayImg, CV_BGR2GRAY);
	//	cuda::GpuMat grayImg_gpu(grayImg);
	//	
	//	vector<KeyPoint> keypoints;
	//	cuda::GpuMat descriptors;
	//	orb->detectAndCompute(grayImg_gpu, cuda::GpuMat(), keypoints, descriptors);
	//	
	//	keypointsVec.push_back(keypoints);
	//	descriptorsVec.push_back(descriptors);
	//	cout << " " << keypoints.size() << std::flush;
	//}
	//cout << endl;
	//
	//t_BBtime = getTickCount();
	//t_pt = (t_BBtime - t_AAtime)/getTickFrequency();
	//t_fpt = img_numbers.size()/t_pt;
	//printf("orb cuda features %.4lf sec/ %.4lf fps\n",  t_pt, t_fpt );
	//
	//cout << "\ncuda pairwise matching start..." ;
	//vector<vector<vector<DMatch>>> good_matchesVecVec;
	//for (int i = (range_width > 0 ? range_width : 0); i < img_numbers.size(); i++)
	//{
	//	vector<vector<DMatch>> good_matchesVec;
	//	for (int j = 0; j < (range_width > 0 ? img_numbers.size() - range_width : img_numbers.size()); j++)
	//	{
	//		if(i == j)
	//		{
	//			cout << " " << i << std::flush;
	//		}
	//		else
	//		{
	//			vector<vector<DMatch>> matches;
	//			matcher->knnMatch(descriptorsVec[i], descriptorsVec[j], matches, 2);
	//			vector<DMatch> good_matches;
	//			for(int k = 0; k < matches.size(); k++)
	//			{
	//				if(matches[k][0].distance < 0.5 * matches[k][1].distance && matches[k][0].distance < 40)
	//				{
	//					//cout << matches[k][0].distance << "/" << matches[k][1].distance << " " << matches[k][0].imgIdx << "/" << matches[k][1].imgIdx << " " << matches[k][0].queryIdx << "/" << matches[k][1].queryIdx << " " << matches[k][0].trainIdx << "/" << matches[k][1].trainIdx << endl;
	//					good_matches.push_back(matches[k][0]);
	//				}
	//			}
	//			good_matchesVec.push_back(good_matches);
	//		}
	//	}
	//	good_matchesVecVec.push_back(good_matchesVec);
	//}
	//cout << endl;
	//
	//t_CCtime = getTickCount();
	//t_pt = (t_CCtime - t_BBtime)/getTickFrequency();
	//t_fpt = img_numbers.size()/t_pt;
	//printf("cuda pairwise matching %.4lf sec/ %.4lf fps\n\n",  t_pt, t_fpt );
	
}

void Pose::createPlaneFittedDisparityImages(int i)
{
	//cout << "Image" << i << endl;
	Mat segment_img = segment_maps[i];
	Mat disp_img = disparity_images[i];
	Mat new_disp_img = Mat::zeros(disp_img.rows,disp_img.cols, CV_64F);
	
	for (int cluster = 1; cluster < 1024; cluster++)
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
	double_disparity_images[i] = new_disp_img;
	
	if (release)
	{
		double plane_fitted_disp_img_var = getVariance(new_disp_img, true);
		//cout << img_numbers[i] << " plane_fitted_disp_img_var " << plane_fitted_disp_img_var << endl;
		//log_file << img_numbers[i] << " plane_fitted_disp_img_var " << plane_fitted_disp_img_var << endl;
		if (plane_fitted_disp_img_var > 3)
			throw "Exception: plane_fitted_disp_img_var " + to_string(i) + " > 3. Unacceptable disparity image.";
	}
	cout << " dd" << img_numbers[i] << std::flush;
}

double Pose::getMean(Mat disp_img, bool planeFitted)
{
	double sum = 0.0;
	for (int y = boundingBox; y < rows - boundingBox; ++y)
	{
		for (int x = cols_start_aft_cutout; x < cols - boundingBox; ++x)
		{
			double disp_val = 0;
			if(planeFitted)
				disp_val = disp_img.at<double>(y,x);
			else
				disp_val = (double)disp_img.at<uchar>(y,x);
			
			if (disp_val > minDisparity)
				sum += disp_val;
		}
	}
	return sum/((rows - 2 * boundingBox )*(cols - boundingBox - cols_start_aft_cutout));
}

double Pose::getVariance(Mat disp_img, bool planeFitted)
{
	double mean = getMean(disp_img, planeFitted);
	double temp = 0;

	for (int y = boundingBox; y < rows - boundingBox; ++y)
	{
		for (int x = cols_start_aft_cutout; x < cols - boundingBox; ++x)
		{
			double disp_val = 0;
			if(planeFitted)
				disp_val = disp_img.at<double>(y,x);
			else
				disp_val = (double)disp_img.at<uchar>(y,x);
			
			if (disp_val > minDisparity)
				temp += (disp_val-mean)*(disp_val-mean);
		}
	}
	
	return temp/((rows - 2 * boundingBox )*(cols - boundingBox - cols_start_aft_cutout) - 1);
}

void Pose::createPtCloud(int img_index, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudrgb)
{
	//cout << "Pt Cloud #" << img_index;
	cloudrgb->is_dense = true;
	
	cv::Mat_<double> vec_tmp(4,1);
	
	Mat disp_img;
	if(use_segment_labels)
		disp_img = double_disparity_images[img_index];
	else
		disp_img = disparity_images[img_index];
	//cout << "disp_img.type(): " << type2str(disp_img.type()) << endl;;
	
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
				
				pcl::PointXYZRGB pt_3drgb;
				pt_3drgb.x = (float)vec_tmp(0);
				pt_3drgb.y = (float)vec_tmp(1);
				pt_3drgb.z = (float)vec_tmp(2);
				Vec3b color = full_images[img_index].at<Vec3b>(Point(x, y));
				
				uint32_t rgb = ((uint32_t)color[2] << 16 | (uint32_t)color[1] << 8 | (uint32_t)color[0]);
				pt_3drgb.rgb = *reinterpret_cast<float*>(&rgb);
				
				cloudrgb->points.push_back(pt_3drgb);
				//cout << pt_3d << endl;
			}
		}
	}
	cout << " " << img_numbers[img_index] << "/" << cloudrgb->points.size() << std::flush;
	//if(log_stuff)
	//	log_file << " " << img_numbers[img_index] << "/" << cloudrgb->points.size();
}

void Pose::createSingleImgPtCloud(int img_index, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudrgb)
{
	//cout << "Pt Cloud #" << img_index;
	cloudrgb->is_dense = true;
	
	vector<KeyPoint> keypoints = features[img_index].keypoints;
	//vector<KeyPoint> keypoints = keypointsVec[img_index];
	
	cv::Mat_<double> vec_tmp(4,1);
	
	Mat disp_img;
	if(use_segment_labels)
		disp_img = double_disparity_images[img_index];
	else
		disp_img = disparity_images[img_index];
	
	if (jump_pixels != 1)
	{
		for (int i = 0; i < keypoints.size(); i++)
		{
			double disp_val = 0;
			if(use_segment_labels)
				disp_val = disp_img.at<double>(keypoints[i].pt.y, keypoints[i].pt.x);
			else
				disp_val = (double)disp_img.at<uchar>(keypoints[i].pt.y, keypoints[i].pt.x);
			
			if (disp_val > minDisparity)
			{
				//reference: https://stackoverflow.com/questions/22418846/reprojectimageto3d-in-opencv
				vec_tmp(0)=keypoints[i].pt.x; vec_tmp(1)=keypoints[i].pt.y; vec_tmp(2)=disp_val; vec_tmp(3)=1;
				vec_tmp = Q*vec_tmp;
				vec_tmp /= vec_tmp(3);
				
				pcl::PointXYZRGB pt_3drgb;
				pt_3drgb.x = (float)vec_tmp(0);
				pt_3drgb.y = (float)vec_tmp(1);
				pt_3drgb.z = (float)vec_tmp(2);
				Vec3b color = full_images[img_index].at<Vec3b>(Point(keypoints[i].pt.x, keypoints[i].pt.y));
				
				uint32_t rgb = ((uint32_t)color[2] << 16 | (uint32_t)color[1] << 8 | (uint32_t)color[0]);
				pt_3drgb.rgb = *reinterpret_cast<float*>(&rgb);
				
				cloudrgb->points.push_back(pt_3drgb);
				//cout << pt_3d << endl;
			}
		}
	}
	if (jump_pixels > 0)
	{
		for (int y = boundingBox; y < rows - boundingBox;)
		{
			for (int x = cols_start_aft_cutout; x < cols - boundingBox;)
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
					
					pcl::PointXYZRGB pt_3drgb;
					pt_3drgb.x = (float)vec_tmp(0);
					pt_3drgb.y = (float)vec_tmp(1);
					pt_3drgb.z = (float)vec_tmp(2);
					Vec3b color = full_images[img_index].at<Vec3b>(Point(x, y));
					
					uint32_t rgb = ((uint32_t)color[2] << 16 | (uint32_t)color[1] << 8 | (uint32_t)color[0]);
					pt_3drgb.rgb = *reinterpret_cast<float*>(&rgb);
					
					cloudrgb->points.push_back(pt_3drgb);
					//cout << pt_3d << endl;
				}
				x += jump_pixels;
			}
			y += jump_pixels;
		}
	}
	cout << " " << img_numbers[img_index] << "/" << cloudrgb->points.size() << std::flush;
	//if(log_stuff)
	//	log_file << " " << img_numbers[img_index] << "/" << cloudrgb->points.size();
}

//kernel to create point cloud
//reference:
//https://stackoverflow.com/questions/24613637/custom-kernel-gpumat-with-float
//__global__
//void createSingleImgPtCloudGPU(int img_index, GpuMat disp_img, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudrgb)
//{
//	cloudrgb->is_dense = true;
//	
//	cv::Mat_<double> vec_tmp(4,1);
//	
//	for (int y = boundingBox; y < rows - boundingBox;)
//	{
//		for (int x = cols_start_aft_cutout; x < cols - boundingBox;)
//		{
//			double disp_val = (double)disp_img.at<uchar>(y,x);
//			
//			if (disp_val > minDisparity)
//			{
//				//reference: https://stackoverflow.com/questions/22418846/reprojectimageto3d-in-opencv
//				vec_tmp(0)=x; vec_tmp(1)=y; vec_tmp(2)=disp_val; vec_tmp(3)=1;
//				vec_tmp = Q*vec_tmp;
//				vec_tmp /= vec_tmp(3);
//				
//				pcl::PointXYZRGB pt_3drgb;
//				pt_3drgb.x = (float)vec_tmp(0);
//				pt_3drgb.y = (float)vec_tmp(1);
//				pt_3drgb.z = (float)vec_tmp(2);
//				Vec3b color = full_images[img_index].at<Vec3b>(Point(x, y));
//				
//				uint32_t rgb = ((uint32_t)color[2] << 16 | (uint32_t)color[1] << 8 | (uint32_t)color[0]);
//				pt_3drgb.rgb = *reinterpret_cast<float*>(&rgb);
//				
//				cloudrgb->points.push_back(pt_3drgb);
//				//cout << pt_3d << endl;
//			}
//			x += jump_pixels;
//		}
//		y += jump_pixels;
//	}
//	cout << " " << img_numbers[img_index] << "/" << cloudrgb->points.size() << std::flush;
//}

pcl::registration::TransformationEstimation<pcl::PointXYZRGB, pcl::PointXYZRGB>::Matrix4 Pose::generateTmat(record_t pose)
{
	//rotation of image plane to account for camera pitch -> x axis is towards east and y axis is towards south of image
	pcl::registration::TransformationEstimation<pcl::PointXYZRGB, pcl::PointXYZRGB>::Matrix4 r_xi;
	for (int i = 0; i < 4; i++)
		for (int j = 0; j < 4; j++)
			r_xi(i,j) = 0;
	r_xi(0,0) = r_xi(1,1) = r_xi(2,2) = 1.0;
	r_xi(3,3) = 1.0;
	r_xi(1,1) = cos(theta_xi);
	r_xi(1,2) = -sin(theta_xi);
	r_xi(2,1) = sin(theta_xi);
	r_xi(2,2) = cos(theta_xi);
	
	//rotation of image plane to account for camera roll-> x axis is towards east and y axis is towards south of image
	pcl::registration::TransformationEstimation<pcl::PointXYZRGB, pcl::PointXYZRGB>::Matrix4 r_yi;
	for (int i = 0; i < 4; i++)
		for (int j = 0; j < 4; j++)
			r_yi(i,j) = 0;
	r_yi(0,0) = r_yi(1,1) = r_yi(2,2) = 1.0;
	r_yi(3,3) = 1.0;
	r_yi(0,0) = cos(theta_yi);
	r_yi(0,2) = sin(theta_yi);
	r_yi(2,0) = -sin(theta_yi);
	r_yi(2,2) = cos(theta_yi);
	
	//rotation of image plane to invert y axis and designate all z points negative -> now x axis is towards east and y axis is towards north of image and image
	pcl::registration::TransformationEstimation<pcl::PointXYZRGB, pcl::PointXYZRGB>::Matrix4 r_invert_i;
	for (int i = 0; i < 4; i++)
		for (int j = 0; j < 4; j++)
			r_invert_i(i,j) = 0;
	r_invert_i(3,3) = 1.0;
	////invert y and z
	r_invert_i(0,0) = 1.0;	//x
	r_invert_i(1,1) = -1.0;	//y
	r_invert_i(2,2) = -1.0;	//z
	pcl::registration::TransformationEstimation<pcl::PointXYZRGB, pcl::PointXYZRGB>::Matrix4 r_invert_y;
	for (int i = 0; i < 4; i++)
		for (int j = 0; j < 4; j++)
			r_invert_y(i,j) = 0;
	r_invert_y(3,3) = 1.0;
	r_invert_y(0,0) = 1.0;	//x
	r_invert_y(1,1) = -1.0;	//y
	r_invert_y(2,2) = 1.0;	//z
	pcl::registration::TransformationEstimation<pcl::PointXYZRGB, pcl::PointXYZRGB>::Matrix4 r_invert_z;
	for (int i = 0; i < 4; i++)
		for (int j = 0; j < 4; j++)
			r_invert_z(i,j) = 0;
	r_invert_z(3,3) = 1.0;
	r_invert_z(0,0) = 1.0;	//x
	r_invert_z(1,1) = 1.0;	//y
	r_invert_z(2,2) = -1.0;	//z
	
	//translation image plane to hexacopter
	pcl::registration::TransformationEstimation<pcl::PointXYZRGB, pcl::PointXYZRGB>::Matrix4 t_hi;
	for (int i = 0; i < 4; i++)
		for (int j = 0; j < 4; j++)
			t_hi(i,j) = 0;
	t_hi(0,0) = t_hi(1,1) = t_hi(2,2) = 1.0;
	t_hi(0,3) = trans_x_hi;
	t_hi(1,3) = trans_y_hi;
	t_hi(2,3) = trans_z_hi;
	t_hi(3,3) = 1.0;
	
	//rotation to flip x and y axis-> now x axis is towards north and y axis is towards east of hexacopter
	pcl::registration::TransformationEstimation<pcl::PointXYZRGB, pcl::PointXYZRGB>::Matrix4 r_flip_xy;
	for (int i = 0; i < 4; i++)
		for (int j = 0; j < 4; j++)
			r_flip_xy(i,j) = 0;
	r_flip_xy(3,3) = 1.0;
	//flip x and y
	r_flip_xy(1,0) = 1.0;	//x
	r_flip_xy(0,1) = 1.0;	//y
	r_flip_xy(2,2) = 1.0;	//z
	
	////rotate to invert y axis I dont know why this correction works!
	//pcl::registration::TransformationEstimation<pcl::PointXYZRGB, pcl::PointXYZRGB>::Matrix4 r_invert_y;
	//for (int i = 0; i < 4; i++)
	//	for (int j = 0; j < 4; j++)
	//		r_invert_y(i,j) = 0;
	//r_invert_y(3,3) = 1.0;
	////invert y
	//r_invert_y(0,0) = 1.0;	//x
	//r_invert_y(1,1) = -1.0;	//y
	//r_invert_y(2,2) = 1.0;	//z
	
	
	//converting hexacopter quaternion to rotation matrix
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
	
	//rotation to orient hexacopter coordinates to world coordinates
	pcl::registration::TransformationEstimation<pcl::PointXYZRGB, pcl::PointXYZRGB>::Matrix4 r_wh;
	for (int i = 0; i < 3; i++)
		for (int j = 0; j < 3; j++)
			r_wh(i,j) = rot.at<double>(i,j);
	r_wh(3,0) = r_wh(3,1) = r_wh(3,2) = r_wh(0,3) = r_wh(1,3) = r_wh(2,3) = 0.0;
	r_wh(3,3) = 1.0;
	//t_wh(0,3) = tx - tx * t_wh(0,0) - ty * t_wh(0,1) - tz * t_wh(0,2);
	//t_wh(1,3) = ty - tx * t_wh(1,0) - ty * t_wh(1,1) - tz * t_wh(1,2);
	//t_wh(2,3) = tz - tx * t_wh(2,0) - ty * t_wh(2,1) - tz * t_wh(2,2);
	
	//translation to translate hexacopter coordinates to world coordinates
	pcl::registration::TransformationEstimation<pcl::PointXYZRGB, pcl::PointXYZRGB>::Matrix4 t_wh;
	for (int i = 0; i < 4; i++)
		for (int j = 0; j < 4; j++)
			t_wh(i,j) = 0;
	t_wh(0,0) = t_wh(1,1) = t_wh(2,2) = 1.0;
	t_wh(0,3) = tx;
	t_wh(1,3) = ty;
	t_wh(2,3) = tz;
	t_wh(3,3) = 1.0;
	
	//translate hexacopter take off position to base station antenna location origin : Translation: [1.946, 6.634, -1.006]
	const double antenna_takeoff_offset_x = 1.946;
	const double antenna_takeoff_offset_y = 6.634;
	const double antenna_takeoff_offset_z = -1.006;
	pcl::registration::TransformationEstimation<pcl::PointXYZRGB, pcl::PointXYZRGB>::Matrix4 t_ow;
	for (int i = 0; i < 4; i++)
		for (int j = 0; j < 4; j++)
			t_ow(i,j) = 0;
	t_ow(0,0) = t_ow(1,1) = t_ow(2,2) = 1.0;
	t_ow(0,3) = - antenna_takeoff_offset_x;
	t_ow(1,3) = - antenna_takeoff_offset_y;
	t_ow(2,3) = - antenna_takeoff_offset_z;
	t_ow(3,3) = 1.0;
		
	pcl::registration::TransformationEstimation<pcl::PointXYZRGB, pcl::PointXYZRGB>::Matrix4 t_mat = t_wh * r_wh * r_invert_y * r_flip_xy * t_hi * r_invert_i * r_yi * r_xi;
	//pcl::registration::TransformationEstimation<pcl::PointXYZRGB, pcl::PointXYZRGB>::Matrix4 t_mat = t_ow * t_wh * r_wh * r_flip_xy * t_hi * r_invert_i * r_yi * r_xi;
	//pcl::registration::TransformationEstimation<pcl::PointXYZRGB, pcl::PointXYZRGB>::Matrix4 t_mat = t_wh * r_wh * r_invert_y * r_flip_xy * t_hi * r_invert_y * r_invert_z * r_yi * r_xi;
	
	//cout << "r_xi:\n" << r_xi << endl;
	//cout << "r_yi:\n" << r_yi << endl;
	//cout << "r_invert_i:\n" << r_invert_i << endl;
	//cout << "t_hi:\n" << t_hi << endl;
	//cout << "r_flip_xy:\n" << r_flip_xy << endl;
	//cout << "r_invert_y:\n" << r_invert_y << endl;
	//cout << "r_wh:\n" << r_wh << endl;
	//cout << "t_wh:\n" << t_wh << endl;
	//cout << "t_mat:\n" << t_mat << endl;
	
	return t_mat;
}

void Pose::transformPtCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudrgb, pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformed_cloudrgb, pcl::registration::TransformationEstimation<pcl::PointXYZRGB, pcl::PointXYZRGB>::Matrix4 transform)
{
	// Executing the transformation
	pcl::transformPointCloud(*cloudrgb, *transformed_cloudrgb, transform);
}

// struct that will contain point cloud and indices 
struct CloudandIndices 
{ 
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_ptr; 
	pcl::PointIndices::Ptr point_indicies; 
}; 

double x_last_pt = 0;
double y_last_pt = 0;
double z_last_pt = 0;
bool calc_height = false;
//This gets all of the indices that you box out.   
void area_picking_get_points (const pcl::visualization::AreaPickingEvent &event, void* cloudStruct)
{
	cout << "inside area_picking_get_points" << endl;
	struct CloudandIndices *cloudInfoStruct = (struct CloudandIndices*) cloudStruct;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr tempCloud = cloudInfoStruct->cloud_ptr;
	pcl::PointIndices::Ptr point_indices_ = cloudInfoStruct->point_indicies;
	//cout << "points in tempCloud " << tempCloud->size() << endl;
	//cout << "points in point_indices_ " << point_indices_->indices.size() << endl;
	
	if (event.getPointsIndices (point_indices_->indices)) 
	{ 
		cout <<"picked "<< point_indices_->indices.size() << " points,";
		double x = 0, y = 0, z = 0, var_x = 0, var_y = 0, var_z = 0;
		for (unsigned int i = 0; i < point_indices_->indices.size(); i++)
		{
			int index = point_indices_->indices[i];
			//cout <<"point " << index << " (" << tempCloud->points[index].x << "," << tempCloud->points[index].y << "," << tempCloud->points[index].z << ")" << endl;
			x += tempCloud->points[index].x;
			y += tempCloud->points[index].y;
			z += tempCloud->points[index].z;
		}
		x /= point_indices_->indices.size();
		y /= point_indices_->indices.size();
		z /= point_indices_->indices.size();
		
		for (unsigned int i = 0; i < point_indices_->indices.size(); i++)
		{
			int index = point_indices_->indices[i];
			//cout <<"point " << index << " (" << tempCloud->points[index].x << "," << tempCloud->points[index].y << "," << tempCloud->points[index].z << ")" << endl;
			var_x += (tempCloud->points[index].x - x)*(tempCloud->points[index].x - x);
			var_y += (tempCloud->points[index].y - y)*(tempCloud->points[index].y - y);
			var_z += (tempCloud->points[index].z - z)*(tempCloud->points[index].z - z);
		}
		var_x /= (point_indices_->indices.size() - 1);
		var_y /= (point_indices_->indices.size() - 1);
		var_z /= (point_indices_->indices.size() - 1);
		
		cout << " mean (" << x << "," << y << "," << z << ")" << " std (" << sqrt(var_x) << "," << sqrt(var_y) << "," << sqrt(var_z) << ")" << endl;
		
		if (calc_height)
		{
			double height = z - z_last_pt;
			double distange = sqrt((x - x_last_pt)*(x - x_last_pt) + (y - y_last_pt)*(y - y_last_pt));
			cout << "difference between last two points: height " << height << " distance: " << distange << endl;
			calc_height = false;
		}
		else
		{
			x_last_pt = x;
			y_last_pt = y;
			z_last_pt = z;
			calc_height = true;
		}
		
		
	}
	else 
		cout<<"No valid points selected!"<<std::endl; 
}

boost::shared_ptr<pcl::visualization::PCLVisualizer> Pose::visualize_pt_cloud(bool showcloud, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudrgb, bool showmesh, pcl::PolygonMesh &mesh, string pt_cloud_name)
{
	//cout << "Starting Visualization..." << endl;
	//cout << "- use h for help" << endl;
	//cout << "visualizing cloud with " << cloudrgb->size() << " points" << endl;
	
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer " + pt_cloud_name));
	//pcl::visualization::PCLVisualizer viewer ("3d visualizer " + pt_cloud_name);
	//pcl::visualization::PCLVisualizer *viewerPtr = &viewer;
	if(showcloud)
	{
		pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb (cloudrgb);
		viewer->addPointCloud<pcl::PointXYZRGB> (cloudrgb, rgb, pt_cloud_name);
	}
	if(showmesh)
	{
		viewer->addPolygonMesh(mesh,"meshes",0);
	}
	
	if(displayUAVPositions)
	{
		last_hexPos_cloud_points = hexPos_cloud->size();
		for (int i = 0; i < hexPos_cloud->size(); i++)
		{
			if(hexPos_cloud->points[i].r == 255)
				viewer->addSphere(hexPos_cloud->points[i], 0.1, 255, 0, 0, "hexPos"+to_string(i), 0);
			else if(hexPos_cloud->points[i].g == 255)
				viewer->addSphere(hexPos_cloud->points[i], 0.1, 0, 255, 0, "hexPos"+to_string(i), 0);
			else if(hexPos_cloud->points[i].b == 255)
				viewer->addSphere(hexPos_cloud->points[i], 0.1, 0, 0, 255, "hexPos"+to_string(i), 0);
			else
				viewer->addSphere(hexPos_cloud->points[i], 0.1, "hexPos"+to_string(i), 0);
		}
	}
	
	viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, pt_cloud_name);

	viewer->addCoordinateSystem (1.0, 0, 0, 0);
	viewer->setBackgroundColor(0.05, 0.05, 0.05, 0); // Setting background to a dark grey
	viewer->setPosition(0, 540); // Setting visualiser window position
	
	if(wait_at_visualizer)
	{
		cout << "*** Display the visualiser until 'q' key is pressed ***" << endl;
		while (!viewer->wasStopped ()) { // Display the visualiser until 'q' key is pressed
			viewer->spinOnce();
		}
		cout << "Cya!" << endl;
	}
	else
	{
		viewer->spinOnce(1,true);
	}
	
	return (viewer);
}

//another visualization function made just for noting height difference and location
void Pose::visualize_pt_cloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudrgb, string pt_cloud_name)
{
	cout << "Starting Visualization..." << endl;
	cout << "- use h for help" << endl;
	cout << "- use x to toggle between area selection and pan/rotate/move" << endl;
	cout << "- use SHIFT + LEFT MOUSE to select area, it will give mean values of pixels along with std div" << endl;
	cout << "visualizing cloud with " << cloudrgb->size() << " points" << endl;
	
	//boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer " + pt_cloud_name));
	pcl::visualization::PCLVisualizer viewer ("3d visualizer " + pt_cloud_name);
	//pcl::visualization::PCLVisualizer *viewerPtr = &viewer;
	pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb (cloudrgb);
	viewer.addPointCloud<pcl::PointXYZRGB> (cloudrgb, rgb, pt_cloud_name);
	if(displayUAVPositions)
	{
		for (int i = 0; i < hexPos_cloud->size(); i++)
		{
			if(hexPos_cloud->points[i].r == 255)
				viewer.addSphere(hexPos_cloud->points[i], 0.1, 255, 0, 0, "hexPos"+to_string(i), 0);
			else if(hexPos_cloud->points[i].g == 255)
				viewer.addSphere(hexPos_cloud->points[i], 0.1, 0, 255, 0, "hexPos"+to_string(i), 0);
			else if(hexPos_cloud->points[i].b == 255)
				viewer.addSphere(hexPos_cloud->points[i], 0.1, 0, 0, 255, "hexPos"+to_string(i), 0);
			else
				viewer.addSphere(hexPos_cloud->points[i], 0.1, "hexPos"+to_string(i), 0);
		}
	}
	viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, pt_cloud_name);

	cout << "*** Display the visualiser until 'q' key is pressed ***" << endl;
	
	viewer.addCoordinateSystem (1.0, 0, 0, 0);
	viewer.setBackgroundColor(0.05, 0.05, 0.05, 0); // Setting background to a dark grey
	viewer.setPosition(0, 540); // Setting visualiser window position
	
	// Struct Pointers for Passing Cloud to Events/Callbacks ----------- some of this may be redundant 
	pcl::PointIndices::Ptr point_indicies (new pcl::PointIndices());
	struct CloudandIndices pointSelectors;
	pointSelectors.cloud_ptr = cloudrgb;
	pointSelectors.point_indicies = point_indicies;
	CloudandIndices *pointSelectorsPtr = &pointSelectors;
	//reference http://www.pcl-users.org/Select-set-of-points-using-mouse-td3424113.html
	viewer.registerAreaPickingCallback (area_picking_get_points, (void*)pointSelectorsPtr);
	cout << "registered viewer" << endl;

	while (!viewer.wasStopped ()) { // Display the visualiser until 'q' key is pressed
		viewer.spinOnce();
	}
}

void Pose::visualize_pt_cloud_update(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudrgb, string pt_cloud_name, boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer)
{
	//pcl::visualization::PCLVisualizer viewer = *viewerPtr;
	
	pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb (cloudrgb);
	viewer->updatePointCloud<pcl::PointXYZRGB> (cloudrgb, rgb, pt_cloud_name);
	
	if(displayUAVPositions)
	{
		//update old points
		for (int i = 0; i < last_hexPos_cloud_points; i++)
		{
			if(hexPos_cloud->points[i].r == 255)
				viewer->updateSphere(hexPos_cloud->points[i], 0.1, 255, 0, 0, "hexPos"+to_string(i));
			else if(hexPos_cloud->points[i].g == 255)
				viewer->updateSphere(hexPos_cloud->points[i], 0.1, 0, 255, 0, "hexPos"+to_string(i));
			else if(hexPos_cloud->points[i].b == 255)
				viewer->updateSphere(hexPos_cloud->points[i], 0.1, 0, 0, 255, "hexPos"+to_string(i));
			//else
			//	viewer->updateSphere(hexPos_cloud->points[i], 0.1, "FMFittedaa"+to_string(i));
		}
		//add new points
		for (int i = last_hexPos_cloud_points; i < hexPos_cloud->size(); i++)
		{
			if(hexPos_cloud->points[i].r == 255)
				viewer->addSphere(hexPos_cloud->points[i], 0.1, 255, 0, 0, "hexPos"+to_string(i), 0);
			else if(hexPos_cloud->points[i].g == 255)
				viewer->addSphere(hexPos_cloud->points[i], 0.1, 0, 255, 0, "hexPos"+to_string(i), 0);
			else if(hexPos_cloud->points[i].b == 255)
				viewer->addSphere(hexPos_cloud->points[i], 0.1, 0, 0, 255, "hexPos"+to_string(i), 0);
			else
				viewer->addSphere(hexPos_cloud->points[i], 0.1, "hexPos"+to_string(i), 0);
		}
		//update point counts
		last_hexPos_cloud_points = hexPos_cloud->size();
	}
	viewer->spinOnce(1,true);
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr Pose::read_PLY_File(string point_cloud_filename)
{
	cout << "Reading PLY file..." << endl;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudrgb (new pcl::PointCloud<pcl::PointXYZRGB> ());
	pcl::PLYReader Reader;
	Reader.read(point_cloud_filename, *cloudrgb);
	cout << "Read PLY file!" << endl;
	return cloudrgb;
}

void Pose::save_pt_cloud_to_PLY_File(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudrgb, string &writePath)
{
	pcl::io::savePLYFileBinary(writePath, *cloudrgb);
	std::cerr << "Saved Point Cloud with " << cloudrgb->points.size () << " data points to " << writePath << endl;
}

pcl::registration::TransformationEstimation<pcl::PointXYZRGB, pcl::PointXYZRGB>::Matrix4 Pose::runICPalignment(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_out)
{
	cout << "Running ICP to align point clouds..." << endl;
	pcl::IterativeClosestPoint<pcl::PointXYZRGB, pcl::PointXYZRGB> icp;
	icp.setInputSource(cloud_in);
	icp.setInputTarget(cloud_out);
	pcl::PointCloud<pcl::PointXYZRGB> Final;
	icp.align(Final);
	cout << "has converged: " << icp.hasConverged() << " score: " << icp.getFitnessScore() << endl;
	Eigen::Matrix4f icp_tf = icp.getFinalTransformation();
	cout << icp_tf << endl;
	
	pcl::registration::TransformationEstimation<pcl::PointXYZRGB, pcl::PointXYZRGB>::Matrix4 tf_icp_main;
	for (int i = 0; i < 4; i++)
		for (int j = 0; j < 4; j++)
			tf_icp_main(i,j) = icp_tf(i,j);
	
	return tf_icp_main;
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr Pose::downsamplePtCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloudrgb, bool combinedPtCloud)
{
	//cerr << "PointCloud before filtering: " << cloudrgb->width * cloudrgb->height 
	//	<< " data points (" << pcl::getFieldsList (*cloudrgb) << ")." << endl;
	
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudrgb_outlier_removed (new pcl::PointCloud<pcl::PointXYZRGB> ());
	int j = 0;
	for (int i = 0; i < cloudrgb->size(); i++)
	{
		//if(cloudrgb->points[i].z > -max_depth && cloudrgb->points[i].z < max_height)
		{
			cloudrgb_outlier_removed->points.push_back(cloudrgb->points[i]);
			if(combinedPtCloud)
				cloudrgb_outlier_removed->points[j].z += 500;	//increasing height to place all points at center of voxel of size 1000 m
			j++;
		}
	}
	
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudrgb_filtered (new pcl::PointCloud<pcl::PointXYZRGB> ());
	
	if (!combinedPtCloud && jump_pixels > 0)
	{
		//cout << " before:" << cloudrgb_outlier_removed->size();
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudrgb_filtered_stat (new pcl::PointCloud<pcl::PointXYZRGB> ());
		//use statistical outlier remover to remove outliers from single image point cloud
		// Create the filtering object
		pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor0;
		sor0.setInputCloud (cloudrgb_outlier_removed);
		sor0.setMeanK (50);
		sor0.setStddevMulThresh (1.0);
		sor0.filter (*cloudrgb_filtered_stat);
		cloudrgb_outlier_removed = cloudrgb_filtered_stat;
		//cout << " after:" << cloudrgb_outlier_removed->size() << " ";
	}
	
	// Create the filtering object
	pcl::VoxelGrid<pcl::PointXYZRGB> sor;
	sor.setInputCloud (cloudrgb_outlier_removed);
	if (combinedPtCloud)
	{
		sor.setMinimumPointsNumberPerVoxel(min_points_per_voxel);
		sor.setLeafSize (voxel_size,voxel_size,1000);
	}
	else
	{	//single image point cloud -> go for higher resolution to better create combinedPtCloud later
		sor.setLeafSize (voxel_size/5,voxel_size/5,voxel_size/5);
	}
	sor.filter (*cloudrgb_filtered);
	
	if(combinedPtCloud)
		for (int i = 0; i < cloudrgb_filtered->size(); i++)
			cloudrgb_filtered->points[i].z -= 500;	//changing back height to original place
	
	//cerr << "\nPointCloud after filtering: " << cloudrgb_filtered->width * cloudrgb_filtered->height 
	//	<< " data points (" << pcl::getFieldsList (*cloudrgb_filtered) << ")." << endl;
	
	return cloudrgb_filtered;
}

void Pose::smoothPtCloud()
{
	int64 t0 = getTickCount();
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudrgb = read_PLY_File(read_PLY_filename0);
	// Create a KD-Tree
	pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB>);

	// Output has the PointNormal type in order to store the normals calculated by MLS
	pcl::PointCloud<pcl::PointXYZRGB> mls_points;

	// Init object (second point type is for the normals, even if unused)
	pcl::MovingLeastSquares<pcl::PointXYZRGB, pcl::PointXYZRGB> mls;

	mls.setComputeNormals (true);

	// Set parameters
	mls.setInputCloud (cloudrgb);
	mls.setPolynomialOrder (true);
	mls.setSearchMethod (tree);
	mls.setSearchRadius (search_radius);
	
	// Reconstruct
	mls.process (mls_points);
	
	cout << "Smoothing surface, time: " << ((getTickCount() - t0) / getTickFrequency()) << " sec" << endl;
	
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (&mls_points);
	
	string writePath = "smoothed_" + read_PLY_filename0;
	save_pt_cloud_to_PLY_File(cloud, writePath);
	
	pcl::PolygonMesh mesh;
	visualize_pt_cloud(true, cloud, false, mesh, writePath);
	cout << "Cya!" << endl;
}

void Pose::meshSurface()
{
	int64 t0 = getTickCount();
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudrgb = read_PLY_File(read_PLY_filename0);
	
	cout << "convert to PointXYZ" << endl;
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ> ());
	pcl::copyPointCloud(*cloudrgb, *cloud);
	
	// Normal estimation*
	cout << "Normal estimation" << endl;
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n;
	pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
	tree->setInputCloud (cloud);
	n.setInputCloud (cloud);
	n.setSearchMethod (tree);
	n.setKSearch (20);
	n.compute (*normals);
	//* normals should not contain the point normals + surface curvatures
	
	// Concatenate the XYZ and normal fields*
	cout << "Concatenate the XYZ and normal fields" << endl;
	pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals (new pcl::PointCloud<pcl::PointNormal>);
	pcl::concatenateFields (*cloud, *normals, *cloud_with_normals);
	//* cloud_with_normals = cloud + normals
	
	// Create search tree*
	cout << "Create search tree" << endl;
	pcl::search::KdTree<pcl::PointNormal>::Ptr tree2 (new pcl::search::KdTree<pcl::PointNormal>);
	tree2->setInputCloud (cloud_with_normals);
	
	// Initialize objects
	cout << "Initialize objects and set values" << endl;
	pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;
	pcl::PolygonMesh triangles;
	
	// Set the maximum distance between connected points (maximum edge length)
	gp3.setSearchRadius (0.05);
	
	// Set typical values for the parameters
	gp3.setMu (10);
	gp3.setMaximumNearestNeighbors (500);
	gp3.setMaximumSurfaceAngle(M_PI/2); // 90 degrees
	gp3.setMinimumAngle(M_PI/18); // 10 degrees
	gp3.setMaximumAngle(2*M_PI/3); // 120 degrees
	gp3.setNormalConsistency(false);
	
	// Get result
	cout << "Get result" << endl;
	gp3.setInputCloud (cloud_with_normals);
	gp3.setSearchMethod (tree2);
	gp3.reconstruct (triangles);
	
	cout << "Meshing surface, time: " << ((getTickCount() - t0) / getTickFrequency()) << " sec" << endl;
	
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr nullCloud;
	
	string writePath = "meshed_" + read_PLY_filename0;
	pcl::io::savePLYFileBinary(writePath, triangles);
	std::cerr << "Saved Mesh to " << writePath << endl;
	
	visualize_pt_cloud(false, nullCloud, true, triangles, writePath);
	
	cout << "Cya!" << endl;
}

pcl::PointXYZRGB Pose::addPointFromPoseFile(int pose_index, bool red_or_blue)
{
	pcl::PointXYZRGB position;
	position.x = pose_data[pose_index][tx_ind];
	position.y = pose_data[pose_index][ty_ind];
	position.z = pose_data[pose_index][tz_ind];
	uint32_t rgb;
	if(red_or_blue)
		rgb = (uint32_t)255 << 16;	//red
	else
		rgb = (uint32_t)255;		//blue
	position.rgb = *reinterpret_cast<float*>(&rgb);
	
	return position;
}

pcl::PointXYZRGB Pose::transformPoint(pcl::PointXYZRGB hexPosMAVLink, pcl::registration::TransformationEstimation<pcl::PointXYZRGB, pcl::PointXYZRGB>::Matrix4 T_SVD_matched_pts)
{
	pcl::PointXYZRGB hexPosFM;// = pcl::transformPoint(hexPosMAVLink, T_SVD_matched_pts);
	hexPosFM.x = static_cast<float> (T_SVD_matched_pts (0, 0) * hexPosMAVLink.x + T_SVD_matched_pts (0, 1) * hexPosMAVLink.y + T_SVD_matched_pts (0, 2) * hexPosMAVLink.z + T_SVD_matched_pts (0, 3));
	hexPosFM.y = static_cast<float> (T_SVD_matched_pts (1, 0) * hexPosMAVLink.x + T_SVD_matched_pts (1, 1) * hexPosMAVLink.y + T_SVD_matched_pts (1, 2) * hexPosMAVLink.z + T_SVD_matched_pts (1, 3));
	hexPosFM.z = static_cast<float> (T_SVD_matched_pts (2, 0) * hexPosMAVLink.x + T_SVD_matched_pts (2, 1) * hexPosMAVLink.y + T_SVD_matched_pts (2, 2) * hexPosMAVLink.z + T_SVD_matched_pts (2, 3));
	uint32_t rgbFM = (uint32_t)255 << 8;	//green
	hexPosFM.rgb = *reinterpret_cast<float*>(&rgbFM);
	
	return hexPosFM;
}

pcl::registration::TransformationEstimation<pcl::PointXYZRGB, pcl::PointXYZRGB>::Matrix4 Pose::generate_tf_of_Matched_Keypoints_Point_Cloud
(int img_index, vector<pcl::registration::TransformationEstimation<pcl::PointXYZRGB, pcl::PointXYZRGB>::Matrix4> t_FMVec, 
pcl::registration::TransformationEstimation<pcl::PointXYZRGB, pcl::PointXYZRGB>::Matrix4 t_mat_MAVLink,
vector<int> &row1_UAV_pos_idx, vector<int> &row2_UAV_pos_idx, pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud_hexPos_MAVLink)
{
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_current (new pcl::PointCloud<pcl::PointXYZRGB> ());
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_prior (new pcl::PointCloud<pcl::PointXYZRGB> ());
	cloud_current->is_dense = true;
	cloud_prior->is_dense = true;
	bool first_match = true;
	
	//cout << "Read source disparity image." << endl;
	
	int pose_index_src = data_index_finder(img_numbers[img_index]);
	//pose_data[pose_index_src][tx_ind] << "," << pose_data[pose_index_src][ty_ind]
	
	//find matches and create matched point clouds
	int good_matches_count = generate_Matched_Keypoints_Point_Cloud(img_index, t_FMVec, t_mat_MAVLink, cloud_current, cloud_prior, pose_index_src, row1_UAV_pos_idx, row2_UAV_pos_idx, cloud_hexPos_MAVLink);
	if (good_matches_count <= 500)
	{
		dist_nearby *= 2;
		cout << "\t** LOW MATCHES ** dist_nearby " << dist_nearby << " retrying.." << endl;
		cout << img_numbers[img_index];
		log_file << "\t** LOW MATCHES ** dist_nearby " << dist_nearby << " retrying.." << endl;
		log_file << img_numbers[img_index];
		cloud_current->clear();
		cloud_prior->clear();
		generate_Matched_Keypoints_Point_Cloud(img_index, t_FMVec, t_mat_MAVLink, cloud_current, cloud_prior, pose_index_src, row1_UAV_pos_idx, row2_UAV_pos_idx, cloud_hexPos_MAVLink);
		dist_nearby /= 2;
	}
	
	pcl::registration::TransformationEstimationSVD<pcl::PointXYZRGB, pcl::PointXYZRGB> te2;
	pcl::registration::TransformationEstimation<pcl::PointXYZRGB, pcl::PointXYZRGB>::Matrix4 T_SVD_matched_pts;
	
	//cout << " cloud_current->size():" << cloud_current->size() << " cloud_prior->size():" << cloud_prior->size() << endl;
	te2.estimateRigidTransformation(*cloud_current, *cloud_prior, T_SVD_matched_pts);
	//cout << "computed transformation between MATCHED KEYPOINTS T_SVD2 is\n" << T_SVD_matched_pts << endl;
	//log_file << "computed transformation between MATCHED KEYPOINTS T_SVD2 is\n" << T_SVD_matched_pts << endl;
	
	return T_SVD_matched_pts;
}



int Pose::generate_Matched_Keypoints_Point_Cloud
(int img_index, vector<pcl::registration::TransformationEstimation<pcl::PointXYZRGB, pcl::PointXYZRGB>::Matrix4> t_FMVec, 
pcl::registration::TransformationEstimation<pcl::PointXYZRGB, pcl::PointXYZRGB>::Matrix4 t_mat_MAVLink,
pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud_current, pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud_prior, int pose_index_src,
vector<int> &row1_UAV_pos_idx, vector<int> &row2_UAV_pos_idx, pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud_hexPos_MAVLink)
{
	cout << " matched with_imgs/matches";
	log_file << " matched with_imgs/matches";
	
	Mat disp_img_src;
	if(use_segment_labels)
		disp_img_src = double_disparity_images[img_index];
	else
		disp_img_src = disparity_images[img_index];
	
	int good_matched_imgs_this_src = 0;
	int good_matches_count = 0;
	
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr keypoints3D_src = keypoints3DVec[img_index];
	vector<bool> goodness_src = keypoints3DGoodnessVec[img_index];
	
	for (int dst_index = img_index-1; dst_index >= max(img_index - range_width,0); dst_index--)
	{
		//check for only with nearby images
		int pose_index_dst = data_index_finder(img_numbers[dst_index]);
		double dist = sqrt((pose_data[pose_index_src][tx_ind] - pose_data[pose_index_dst][tx_ind]) * (pose_data[pose_index_src][tx_ind] - pose_data[pose_index_dst][tx_ind])
			+ (pose_data[pose_index_src][ty_ind] - pose_data[pose_index_dst][ty_ind]) * (pose_data[pose_index_src][ty_ind] - pose_data[pose_index_dst][ty_ind]));
		if(dist > dist_nearby)
			continue;
		
		//reference https://stackoverflow.com/questions/44988087/opencv-feature-matching-match-descriptors-to-knn-filtered-keypoints
		//reference https://github.com/opencv/opencv/issues/6130
		//reference http://study.marearts.com/2014/07/opencv-study-orb-gpu-feature-extraction.html
		//reference https://docs.opencv.org/3.1.0/d6/d1d/group__cudafeatures2d.html
		
		//cout << "image " << img_index << " to " << dst_index << endl;
		vector<vector<DMatch>> matches;
		
		matcher->knnMatch(descriptorsVec[img_index], descriptorsVec[dst_index], matches, 2);
		
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
		
		//cout << " " << dist << "/" << good_matches.size();
		
		if(good_matches.size() < 75)	//less number of matches.. don't bother working on this one. good matches are around 500-600
			continue;
		
		good_matched_imgs++;
		good_matched_imgs_this_src++;
		good_matches_count += good_matches.size();
		
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr keypoints3D_dst = keypoints3DVec[dst_index];
		vector<bool> goodness_dst = keypoints3DGoodnessVec[dst_index];
		
		//using sequential matched points to estimate the rigid body transformation between matched 3D points
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_current_temp (new pcl::PointCloud<pcl::PointXYZRGB> ());
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_prior_temp (new pcl::PointCloud<pcl::PointXYZRGB> ());
		cloud_current_temp->is_dense = true;
		cloud_prior_temp->is_dense = true;
		
		for (int match_index = 0; match_index < good_matches.size(); match_index++)
		{
			DMatch match = good_matches[match_index];
			
			int dst_Idx = match.trainIdx;	//dst img
			int src_Idx = match.queryIdx;	//src img
			
			if(goodness_src[src_Idx] == true && goodness_dst[dst_Idx] == true)
			{
				cloud_current_temp->points.push_back(keypoints3D_src->points[src_Idx]);
				cloud_prior_temp->points.push_back(keypoints3D_dst->points[dst_Idx]);
			}
		}
		
		pcl::registration::TransformationEstimation<pcl::PointXYZRGB, pcl::PointXYZRGB>::Matrix4 t_FM = t_FMVec[dst_index];
		
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_current_t_temp (new pcl::PointCloud<pcl::PointXYZRGB> ());
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_prior_t_temp (new pcl::PointCloud<pcl::PointXYZRGB> ());
		
		pcl::transformPointCloud(*cloud_current_temp, *cloud_current_t_temp, t_mat_MAVLink);
		pcl::transformPointCloud(*cloud_prior_temp, *cloud_prior_t_temp, t_FM);
		
		//cout << " cloud_current_t_temp->size():" << cloud_current_t_temp->size() << " cloud_prior_t_temp->size():" << cloud_prior_t_temp->size() << flush;
		
		cloud_current->insert(cloud_current->end(),cloud_current_t_temp->begin(),cloud_current_t_temp->end());
		cloud_prior->insert(cloud_prior->end(),cloud_prior_t_temp->begin(),cloud_prior_t_temp->end());
	}
	cout << " " << good_matched_imgs_this_src << "/" << good_matches_count;
	log_file << " " << good_matched_imgs_this_src << "/" << good_matches_count;
	
	return good_matches_count;
}

/*	the above two functions with distance being taken into account from row positions


pcl::PointXYZRGB Pose::transformPoint(pcl::PointXYZRGB hexPosMAVLink, pcl::registration::TransformationEstimation<pcl::PointXYZRGB, pcl::PointXYZRGB>::Matrix4 T_SVD_matched_pts)
{
	pcl::PointXYZRGB hexPosFM;// = pcl::transformPoint(hexPosMAVLink, T_SVD_matched_pts);
	hexPosFM.x = static_cast<float> (T_SVD_matched_pts (0, 0) * hexPosMAVLink.x + T_SVD_matched_pts (0, 1) * hexPosMAVLink.y + T_SVD_matched_pts (0, 2) * hexPosMAVLink.z + T_SVD_matched_pts (0, 3));
	hexPosFM.y = static_cast<float> (T_SVD_matched_pts (1, 0) * hexPosMAVLink.x + T_SVD_matched_pts (1, 1) * hexPosMAVLink.y + T_SVD_matched_pts (1, 2) * hexPosMAVLink.z + T_SVD_matched_pts (1, 3));
	hexPosFM.z = static_cast<float> (T_SVD_matched_pts (2, 0) * hexPosMAVLink.x + T_SVD_matched_pts (2, 1) * hexPosMAVLink.y + T_SVD_matched_pts (2, 2) * hexPosMAVLink.z + T_SVD_matched_pts (2, 3));
	uint32_t rgbFM = (uint32_t)255 << 8;	//green
	hexPosFM.rgb = *reinterpret_cast<float*>(&rgbFM);
	
	return hexPosFM;
}

pcl::registration::TransformationEstimation<pcl::PointXYZRGB, pcl::PointXYZRGB>::Matrix4 Pose::generate_tf_of_Matched_Keypoints_Point_Cloud
(int img_index, vector<pcl::registration::TransformationEstimation<pcl::PointXYZRGB, pcl::PointXYZRGB>::Matrix4> t_FMVec, 
pcl::registration::TransformationEstimation<pcl::PointXYZRGB, pcl::PointXYZRGB>::Matrix4 t_mat_MAVLink,
vector<int> &row1_UAV_pos_idx, vector<int> &row2_UAV_pos_idx, pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud_hexPos_MAVLink)
{
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_current (new pcl::PointCloud<pcl::PointXYZRGB> ());
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_prior (new pcl::PointCloud<pcl::PointXYZRGB> ());
	cloud_current->is_dense = true;
	cloud_prior->is_dense = true;
	bool first_match = true;
	
	//cout << "Read source disparity image." << endl;
	
	int pose_index_src = data_index_finder(img_numbers[img_index]);
	//pose_data[pose_index_src][tx_ind] << "," << pose_data[pose_index_src][ty_ind]
	
	//find matches and create matched point clouds
	int good_matches_count = generate_Matched_Keypoints_Point_Cloud(img_index, t_FMVec, t_mat_MAVLink, cloud_current, cloud_prior, pose_index_src, row1_UAV_pos_idx, row2_UAV_pos_idx, cloud_hexPos_MAVLink);
	if (good_matches_count <= 500)
	{
		dist_nearby *= 2;
		cout << "\t** LOW MATCHES ** dist_nearby " << dist_nearby << " retrying.." << endl;
		cout << img_numbers[img_index];
		log_file << "\t** LOW MATCHES ** dist_nearby " << dist_nearby << " retrying.." << endl;
		log_file << img_numbers[img_index];
		cloud_current->clear();
		cloud_prior->clear();
		generate_Matched_Keypoints_Point_Cloud(img_index, t_FMVec, t_mat_MAVLink, cloud_current, cloud_prior, pose_index_src, row1_UAV_pos_idx, row2_UAV_pos_idx, cloud_hexPos_MAVLink);
		dist_nearby /= 2;
	}
	
	pcl::registration::TransformationEstimationSVD<pcl::PointXYZRGB, pcl::PointXYZRGB> te2;
	pcl::registration::TransformationEstimation<pcl::PointXYZRGB, pcl::PointXYZRGB>::Matrix4 T_SVD_matched_pts;
	
	//cout << " cloud_current->size():" << cloud_current->size() << " cloud_prior->size():" << cloud_prior->size() << endl;
	te2.estimateRigidTransformation(*cloud_current, *cloud_prior, T_SVD_matched_pts);
	//cout << "computed transformation between MATCHED KEYPOINTS T_SVD2 is\n" << T_SVD_matched_pts << endl;
	//log_file << "computed transformation between MATCHED KEYPOINTS T_SVD2 is\n" << T_SVD_matched_pts << endl;
	
	return T_SVD_matched_pts;
}



int Pose::generate_Matched_Keypoints_Point_Cloud
(int img_index, vector<pcl::registration::TransformationEstimation<pcl::PointXYZRGB, pcl::PointXYZRGB>::Matrix4> t_FMVec, 
pcl::registration::TransformationEstimation<pcl::PointXYZRGB, pcl::PointXYZRGB>::Matrix4 t_mat_MAVLink,
pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud_current, pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud_prior, int pose_index_src,
vector<int> &row1_UAV_pos_idx, vector<int> &row2_UAV_pos_idx, pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud_hexPos_MAVLink)
{
	cout << " matched with_imgs/matches";
	log_file << " matched with_imgs/matches";
	
	Mat disp_img_src;
	if(use_segment_labels)
		disp_img_src = double_disparity_images[img_index];
	else
		disp_img_src = disparity_images[img_index];
	
	int good_matched_imgs_this_src = 0;
	int good_matches_count = 0;
	
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr keypoints3D_src = keypoints3DVec[img_index];
	vector<bool> goodness_src = keypoints3DGoodnessVec[img_index];
	
	if (row2_UAV_pos_idx.size() > 0 && row2_UAV_pos_idx[row2_UAV_pos_idx.size()-1] != img_index)
		throw "Exception: row2_UAV_pos_idx[row2_UAV_pos_idx.size()-1] != img_index";
	if (row1_UAV_pos_idx.size() == 0)
		throw "Exception: row1_UAV_pos_idx.size() == 0";
		
	bool row2_images = true;
	int dst_index;
	int row_index;
	
	if (row2_UAV_pos_idx.size() > 1)
	{
		row_index = row2_UAV_pos_idx.size()-2;
		dst_index = row2_UAV_pos_idx[row_index];
	}
	else
	{
		row2_images = false;
		row_index = row1_UAV_pos_idx.size()-2;
		dst_index = row1_UAV_pos_idx[row_index];
	}
	
	bool continue_check = true;
	while(continue_check)
	{
		//check for only with nearby images
		double dist = sqrt((cloud_hexPos_MAVLink->points[img_index].x - cloud_hexPos_MAVLink->points[dst_index].x) * (cloud_hexPos_MAVLink->points[img_index].x - cloud_hexPos_MAVLink->points[dst_index].x)
			+ (cloud_hexPos_MAVLink->points[img_index].y - cloud_hexPos_MAVLink->points[dst_index].y) * (cloud_hexPos_MAVLink->points[img_index].y - cloud_hexPos_MAVLink->points[dst_index].y));
		
		if(dist < dist_nearby)
		{
			//reference https://stackoverflow.com/questions/44988087/opencv-feature-matching-match-descriptors-to-knn-filtered-keypoints
			//reference https://github.com/opencv/opencv/issues/6130
			//reference http://study.marearts.com/2014/07/opencv-study-orb-gpu-feature-extraction.html
			//reference https://docs.opencv.org/3.1.0/d6/d1d/group__cudafeatures2d.html
			
			//cout << "image " << img_index << " to " << dst_index << endl;
			vector<vector<DMatch>> matches;
			
			matcher->knnMatch(descriptorsVec[img_index], descriptorsVec[dst_index], matches, 2);
			
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
			
			//cout << " " << dist << "/" << good_matches.size();
			
			if(good_matches.size() > 150)	//less number of matches.. don't bother working on this one. good matches are around 500-600
			{
				good_matched_imgs++;
				good_matched_imgs_this_src++;
				good_matches_count += good_matches.size();
				
				pcl::PointCloud<pcl::PointXYZRGB>::Ptr keypoints3D_dst = keypoints3DVec[dst_index];
				vector<bool> goodness_dst = keypoints3DGoodnessVec[dst_index];
				
				//using sequential matched points to estimate the rigid body transformation between matched 3D points
				pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_current_temp (new pcl::PointCloud<pcl::PointXYZRGB> ());
				pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_prior_temp (new pcl::PointCloud<pcl::PointXYZRGB> ());
				cloud_current_temp->is_dense = true;
				cloud_prior_temp->is_dense = true;
				
				for (int match_index = 0; match_index < good_matches.size(); match_index++)
				{
					DMatch match = good_matches[match_index];
					
					int dst_Idx = match.trainIdx;	//dst img
					int src_Idx = match.queryIdx;	//src img
					
					if(goodness_src[src_Idx] == true && goodness_dst[dst_Idx] == true)
					{
						cloud_current_temp->points.push_back(keypoints3D_src->points[src_Idx]);
						cloud_prior_temp->points.push_back(keypoints3D_dst->points[dst_Idx]);
					}
				}
				
				pcl::registration::TransformationEstimation<pcl::PointXYZRGB, pcl::PointXYZRGB>::Matrix4 t_FM = t_FMVec[dst_index];
				
				pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_current_t_temp (new pcl::PointCloud<pcl::PointXYZRGB> ());
				pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_prior_t_temp (new pcl::PointCloud<pcl::PointXYZRGB> ());
				
				pcl::transformPointCloud(*cloud_current_temp, *cloud_current_t_temp, t_mat_MAVLink);
				pcl::transformPointCloud(*cloud_prior_temp, *cloud_prior_t_temp, t_FM);
				
				//cout << " cloud_current_t_temp->size():" << cloud_current_t_temp->size() << " cloud_prior_t_temp->size():" << cloud_prior_t_temp->size() << flush;
				
				cloud_current->insert(cloud_current->end(),cloud_current_t_temp->begin(),cloud_current_t_temp->end());
				cloud_prior->insert(cloud_prior->end(),cloud_prior_t_temp->begin(),cloud_prior_t_temp->end());
			}
		}
		
		//cout << "row_index " << row_index << " continue_check " << continue_check << " row2_images " << row2_images << " img " << img_numbers[dst_index] << endl;
		
		//update dst_index
		if (row2_images)
		{
			if (--row_index >= 0)
			{
				dst_index = row2_UAV_pos_idx[row_index];
			}
			else
			{
				row2_images = false;
				row_index = row1_UAV_pos_idx.size()-1;
				dst_index = row1_UAV_pos_idx[row_index];
			}
		}
		else
		{
			if (--row_index >= 0)
				dst_index = row1_UAV_pos_idx[row_index];
			else
				continue_check = false;
		}
	}
	cout << " " << good_matched_imgs_this_src << "/" << good_matches_count;
	log_file << " " << good_matched_imgs_this_src << "/" << good_matches_count;
	
	return good_matches_count;
}

*/

void Pose::segmentCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloudrgb)
{
	cout << "\nFinding UGV traversible area on map..." << endl;
	
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_seg (new pcl::PointCloud<pcl::PointXYZRGB> ());
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_obstacles (new pcl::PointCloud<pcl::PointXYZRGB> ());
	int part = 0;
	int size_cloud = (int)ceil(1.0 * cloudrgb->size()/10.0);
	cout << "point cloud size " << cloudrgb->size() << endl;
	while (true)
	{
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_small (new pcl::PointCloud<pcl::PointXYZRGB> ());
		for (int i = part * size_cloud; i < (part + 1) * size_cloud && i < cloudrgb->size(); i++)
			cloud_small->points.push_back(cloudrgb->points[i]);
		
		//cout << "part point cloud size " << cloud_small->size() << endl;
		if (cloud_small->size() == 0)
			break;
		
		pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
		pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
		// Create the segmentation object
		pcl::SACSegmentation<pcl::PointXYZRGB> seg;
		// Optional
		seg.setOptimizeCoefficients (true);
		// Mandatory
		seg.setModelType (pcl::SACMODEL_PLANE);
		seg.setMethodType (pcl::SAC_RANSAC);
		seg.setDistanceThreshold (segment_dist_threashold);
		seg.setInputCloud (cloud_small);
		seg.segment (*inliers, *coefficients);
		if (inliers->indices.size () == 0)
		{
			PCL_ERROR ("Could not estimate a planar model for the given dataset.");
			throw "PCL ERROR: Could not estimate a planar model for the given dataset.";
		}

		std::cerr << "Model coefficients: " << coefficients->values[0] << " " 
										  << coefficients->values[1] << " "
										  << coefficients->values[2] << " " 
										  << coefficients->values[3] << std::endl;

		std::cerr << "Model inliers: " << inliers->indices.size () << std::endl;
		
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudrgbseg (new pcl::PointCloud<pcl::PointXYZRGB> ());
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudrgboutlier (new pcl::PointCloud<pcl::PointXYZRGB> ());
		int index = 0;
		for (size_t i = 0; i < inliers->indices.size (); ++i)
		{
			while(index < inliers->indices[i])
			{
				cloudrgboutlier->points.push_back(cloud_small->points[index]);
				index++;
			}
			cloudrgbseg->points.push_back(cloud_small->points[inliers->indices[i]]);
			index++;
		}
		while(index < cloud_small->size())
		{
			cloudrgboutlier->points.push_back(cloud_small->points[index]);
			index++;
		}
		//copyPointCloud(*cloud_small, inliers->indices, *cloudrgbseg);
		cloud_seg->insert(cloud_seg->end(),cloudrgbseg->begin(),cloudrgbseg->end());
		cloud_obstacles->insert(cloud_obstacles->end(),cloudrgboutlier->begin(),cloudrgboutlier->end());
		//visualize_pt_cloud(cloudrgbseg);
		part++;
	}
	
	displayUAVPositions = false;
	visualize_pt_cloud(cloud_seg, "Traversible Area");
	visualize_pt_cloud(cloud_obstacles, "Obstacles on Course");
	
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_projected (new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
	pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
	// Create the segmentation object
	pcl::SACSegmentation<pcl::PointXYZRGB> seg;
	// Optional
	seg.setOptimizeCoefficients (true);
	// Mandatory
	seg.setModelType (pcl::SACMODEL_PLANE);
	seg.setMethodType (pcl::SAC_RANSAC);
	seg.setDistanceThreshold (convexhull_dist_threshold);

	seg.setInputCloud (cloud_seg);
	seg.segment (*inliers, *coefficients);
	//std::cerr << "PointCloud after segmentation has: "
	//		<< inliers->indices.size () << " inliers." << std::endl;

	// Project the model inliers
	pcl::ProjectInliers<pcl::PointXYZRGB> proj;
	proj.setModelType (pcl::SACMODEL_PLANE);
	proj.setIndices (inliers);
	proj.setInputCloud (cloud_seg);
	proj.setModelCoefficients (coefficients);
	proj.filter (*cloud_projected);
	//std::cerr << "PointCloud after projection has: "
	//		<< cloud_projected->points.size () << " data points." << std::endl;

	// Create a Concave Hull representation of the projected inliers
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_hull (new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::ConcaveHull<pcl::PointXYZRGB> chull;
	chull.setInputCloud (cloud_projected);
	chull.setAlpha (convexhull_alpha);
	chull.reconstruct (*cloud_hull);

	std::cerr << "Concave hull has: " << cloud_hull->points.size ()
			<< " data points." << std::endl;
	
	for (int i = 0; i < cloud_hull->size(); i++)
	{
		uint32_t rgbFM = (uint32_t)255 << 8;	//green
		cloud_hull->points[i].rgb = *reinterpret_cast<float*>(&rgbFM);
	}
	
	visualize_pt_cloud(cloud_hull, "cloud_hull");
}
