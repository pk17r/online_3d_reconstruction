#include "pose.h"

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
		"\n  - hexacopter positions recorded, feature match calculated and feature match positions ICP fitted over recorded positions"
		" will be stored in " << hexPosMAVLinkfilename << ", " << hexPosFMfilename << " & " << hexPosFMFittedfilename << ", respectively"
		"\n\nFlags:"
		"\n  --use_segment_labels"
		"\n      Use pre-made segmented labels for every image to improve resolution of disparity images"
		"\n  --jump_pixels [int]"
		"\n      number of pixels to jump on in each direction to make 3D reconstruction. Make it 5 for faster but sparcer 3D reconstruction"
		"\n  --range_width [int]"
		"\n      Number of nearby images to do pairwise matching on for every image"
		"\n  --preview"
		"\n      visualize generated point cloud at the end"
		"\n  --displayCamPositions"
		"\n      Display camera positions along with point cloud during visualization"
		"\n  --visualize [Pt Cloud filename]"
		"\n      Visualize a given point cloud"
		"\n  --align_point_cloud [Pt Cloud 1] [Pt Cloud 2]"
		"\n      Align point clouds using ICP algorithm"
		"\n  --downsample [Pt Cloud file name] (optional)--voxel_size [float]"
		"\n      Downsample a point cloud along with optional voxel size in meters"
		"\n      Voxel size in m to find average value of points for downsampling"
		"\n  --smooth_surface [Pt Cloud file name] (optional)--search_radius [float]"
		"\n      Smooth surface"
		"\n  --mesh_surface [Pt Cloud file name]"
		"\n      Mesh surface using triangulation"
		"\n  --log"
		"\n      log most things in output/log.txt file"
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
		else if (string(argv[i]) == "--displayCamPositions")
		{
			displayCamPositions = true;
			n_imgs = 1;		//just to stop program from reading images.txt file
			cout << "displayCamPositions" << endl;
			i++;
		}
		else if (string(argv[i]) == "--downsample_transform")
		{
			downsample_transform = true;
			n_imgs = 1;		//just to stop program from reading images.txt file
			downsample_transform_file = string(argv[i + 1]);
			cout << "downsample_transform " << downsample_transform_file << endl;
			i++;
		}
		else if (string(argv[i]) == "--voxel_size")
		{
			voxel_size = atof(argv[i + 1]);
			cout << "voxel_size " << voxel_size << endl;
			i++;
		}
		else if (string(argv[i]) == "--search_radius")
		{
			search_radius = atof(argv[i + 1]);
			cout << "search_radius " << search_radius << endl;
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

void Pose::readImages()
{
	full_images = vector<Mat>(img_numbers.size());
	disparity_images = vector<Mat>(img_numbers.size());
	if(use_segment_labels)
		segment_maps = vector<Mat>(img_numbers.size());
	
	//logging stuff
	if(log_stuff)
		log_file.open(save_log_to.c_str(), ios::out);
	
	Mat full_img;
	cout << "\nReading image number";
	for (int i = 0; i < img_numbers.size(); ++i)
	{
		cout << " " << img_numbers[i] << std::flush;
		full_img = imread(imagePrefix + to_string(img_numbers[i]) + ".png");
		if(full_img.empty())
			throw "Exception: cannot read full_img!";
		
		//imshow( "Display window", full_img );                   // Show our image inside it.
		//waitKey(0);                                          // Wait for a keystroke in the window
		if (i == 0)
		{
			rows = full_img.rows;
			cols = full_img.cols;
			cols_start_aft_cutout = (int)(cols/cutout_ratio);
		}
		full_images[i] = full_img;
		
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
			
			if(disp_img.empty())
				throw "Exception: Cannot read disparity image!";
			
			if (release)
			{
				double disp_img_var = getVariance(disp_img, false);
				cout << img_numbers[i] << " disp_img_var " << disp_img_var << "\t";
				log_file << img_numbers[i] << " disp_img_var " << disp_img_var << "\t";
				if (disp_img_var > 5)
					throw "Exception: disp_img_var > 5. Unacceptable disparity image.";
			}
			disparity_images[i] = disp_img;
			//imshow( "Display window", disp_img );                   // Show our image inside it.
			//waitKey(0);                                          // Wait for a keystroke in the window
			
			createPlaneFittedDisparityImages(i);
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
	}
	cout << endl;
	full_img.release();
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
	
	cout << "\nFeatures";
	for (int i = 0; i < img_numbers.size(); ++i)
	{
		(*finder)(full_images[i], features[i]);
		//vector<KeyPoint> keypointsD;
		//detector->detect(img,keypointsD,Mat());
		features[i].img_idx = i;
		//features[i].keypoints = keypointsD;
		cout << " " << features[i].keypoints.size() << std::flush;
	}
	cout << endl;
	finder->collectGarbage();
	//free(detector);
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
	unsigned long t_AAtime=0, t_BBtime=0, t_CCtime=0;
	float t_pt;
	float t_fpt;
	t_AAtime = getTickCount(); 
	cout << "\nOrb cuda Features Finding start.." << endl;
	
	Ptr<cuda::ORB> orb = cuda::ORB::create();
	
	for (int i = 0; i < img_numbers.size(); i++)
	{
		cv::Mat grayImg;
		cv::cvtColor(full_images[i], grayImg, CV_BGR2GRAY);
		cuda::GpuMat grayImg_gpu(grayImg);
		
		vector<KeyPoint> keypoints;
		cuda::GpuMat descriptors;
		orb->detectAndCompute(grayImg_gpu, cuda::GpuMat(), keypoints, descriptors);
		
		keypointsVec.push_back(keypoints);
		descriptorsVec.push_back(descriptors);
		cout << " " << keypoints.size() << std::flush;
	}
	cout << endl;
	
	t_BBtime = getTickCount();
	t_pt = (t_BBtime - t_AAtime)/getTickFrequency();
	t_fpt = img_numbers.size()/t_pt;
	printf("orb cuda features %.4lf sec/ %.4lf fps\n",  t_pt, t_fpt );
	
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
	
	if (release)
	{
		double plane_fitted_disp_img_var = getVariance(new_disp_img, true);
		cout << img_numbers[i] << " plane_fitted_disp_img_var " << plane_fitted_disp_img_var << endl;
		log_file << img_numbers[i] << " plane_fitted_disp_img_var " << plane_fitted_disp_img_var << endl;
		if (plane_fitted_disp_img_var > 3)
			throw "Exception: plane_fitted_disp_img_var > 3. Unacceptable disparity image.";
	}
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
	cout << "Pt Cloud #" << img_index;
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
	cout << " points " << cloudrgb->points.size() << "\t";
	if(log_stuff)
		log_file << " points " << cloudrgb->points.size() << endl;
}

void Pose::createFeaturePtCloud(int img_index, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudrgb)
{
	cout << "Pt Cloud #" << img_index;
	cloudrgb->is_dense = true;
	
	vector<KeyPoint> keypoints = features[img_index].keypoints;
	//vector<KeyPoint> keypoints = keypointsVec[img_index];
	
	cv::Mat_<double> vec_tmp(4,1);
	
	Mat disp_img;
	if(use_segment_labels)
		disp_img = double_disparity_images[img_index];
	else
		disp_img = disparity_images[img_index];
	
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
	cout << " points " << cloudrgb->points.size() << "\t";
	if(log_stuff)
		log_file << " points " << cloudrgb->points.size() << endl;
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

//This gets all of the indices that you box out.   
void area_picking_get_points (const pcl::visualization::AreaPickingEvent &event, void* cloudStruct)
{ 
	struct CloudandIndices *cloudInfoStruct = (struct CloudandIndices*) cloudStruct;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr tempCloud = cloudInfoStruct->cloud_ptr;
	pcl::PointIndices::Ptr point_indices_ = cloudInfoStruct->point_indicies;
	
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
	}
	else 
		cout<<"No valid points selected!"<<std::endl; 
}

void Pose::visualize_pt_cloud(bool showcloud, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudrgb, bool showmesh, pcl::PolygonMesh &mesh, string pt_cloud_name)
{
	cout << "Starting Visualization..." << endl;
	
	cout << "NOTE:" << endl;
	cout << "- use h for help" << endl;
	cout << "- use x to toggle between area selection and pan/rotate/move" << endl;
	cout << "- use SHIFT + LEFT MOUSE to select area, it will give mean values of pixels along with std div" << endl;
	
	pcl::visualization::PCLVisualizer viewer ("3d visualizer " + pt_cloud_name);
	if(showcloud)
	{
		pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb (cloudrgb);
		viewer.addPointCloud<pcl::PointXYZRGB> (cloudrgb, rgb, pt_cloud_name);
	}
	if(showmesh)
	{
		viewer.addPolygonMesh(mesh,"meshes",0);
	}
	
	if(displayCamPositions)
	{
		if (hexPosMAVLinkVec.size() == 0)
		{
			ifstream data_file;
			data_t hexPosMAVLink_data;
			data_file.open(hexPosMAVLinkfilename);
			data_file >> hexPosMAVLink_data;
			// Complain if something went wrong.
			if (!data_file.eof())
				throw "Exception: Could not open hexPosMAVLinkfilename file!";
			data_file.close();

			cout << "hexPosMAVLink" << endl;
			for (int i = 0; i < hexPosMAVLink_data.size(); i++)
			{
				pcl::PointXYZRGB hexPosMAVLink;
				hexPosMAVLink.x = hexPosMAVLink_data[i][0];
				hexPosMAVLink.y = hexPosMAVLink_data[i][1];
				hexPosMAVLink.z = hexPosMAVLink_data[i][2];
				hexPosMAVLinkVec.push_back(hexPosMAVLink);
			}
			
			data_t hexPosFM_data;
			data_file.open(hexPosFMfilename);
			data_file >> hexPosFM_data;
			// Complain if something went wrong.
			if (!data_file.eof())
				throw "Exception: Could not open hexPosFMfilename file!";
			data_file.close();

			cout << "\nhexPosFM" << endl;
			for (int i = 0; i < hexPosFM_data.size(); i++)
			{
				pcl::PointXYZRGB hexPosFM;
				hexPosFM.x = hexPosFM_data[i][0];
				hexPosFM.y = hexPosFM_data[i][1];
				hexPosFM.z = hexPosFM_data[i][2];
				hexPosFMVec.push_back(hexPosFM);
			}

			data_t hexPosFMFitted_data;
			data_file.open(hexPosFMFittedfilename);
			data_file >> hexPosFMFitted_data;
			// Complain if something went wrong.
			if (!data_file.eof())
				throw "Exception: Could not open hexPosFMFittedfilename file!";
			data_file.close();

			cout << "\nhexPosFMFitted" << endl;
			for (int i = 0; i < hexPosFMFitted_data.size(); i++)
			{
				pcl::PointXYZRGB hexPosFMFitted;
				hexPosFMFitted.x = hexPosFMFitted_data[i][0];
				hexPosFMFitted.y = hexPosFMFitted_data[i][1];
				hexPosFMFitted.z = hexPosFMFitted_data[i][2];
				hexPosFMFittedVec.push_back(hexPosFMFitted);
			}
		}
		
		for (int i = 0; i < hexPosMAVLinkVec.size(); i++)
		{
			viewer.addSphere(hexPosMAVLinkVec[i], 0.1, 255, 0, 0, "MAVLink"+to_string(i), 0);
			viewer.addSphere(hexPosFMVec[i], 0.1, 0, 255, 0, "FM"+to_string(i), 0);
			viewer.addSphere(hexPosFMFittedVec[i], 0.1, 0, 0, 255, "FMFitted"+to_string(i), 0);
		}
	}
	
	viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, pt_cloud_name);

	cout << "*** Display the visualiser until 'q' key is pressed ***" << endl;
	
	viewer.addCoordinateSystem (1.0, 0, 0, 0);
	viewer.setBackgroundColor(0.05, 0.05, 0.05, 0); // Setting background to a dark grey
	viewer.setPosition(1280/2, 720/2); // Setting visualiser window position
	
	if(showcloud)
	{
		// Struct Pointers for Passing Cloud to Events/Callbacks ----------- some of this may be redundant 
		pcl::PointIndices::Ptr point_indicies (new pcl::PointIndices());
		struct CloudandIndices pointSelectors;
		pointSelectors.cloud_ptr = cloudrgb;
		pointSelectors.point_indicies = point_indicies;
		CloudandIndices *pointSelectorsPtr = &pointSelectors;
		//reference http://www.pcl-users.org/Select-set-of-points-using-mouse-td3424113.html
		viewer.registerAreaPickingCallback (area_picking_get_points, (void*)pointSelectorsPtr);
	}

	while (!viewer.wasStopped ()) { // Display the visualiser until 'q' key is pressed
		viewer.spinOnce();
	}
	viewer.close();
	
	cout << "Cya!" << endl;
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

pcl::PointCloud<pcl::PointXYZRGB>::Ptr Pose::downsamplePtCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudrgb)
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
