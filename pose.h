#include <iostream>
#include <fstream>
#include <string>
#include <opencv2/opencv_modules.hpp>
#include <opencv2/core/utility.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/stitching/detail/matchers.hpp>
//#include <opencv2/calib3d.hpp>
//#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>
#include <pcl/console/parse.h>
#include <pcl/common/transforms.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/point_types.h>
//#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/registration/transformation_estimation.h>
#include <pcl/registration/transformation_estimation_svd.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/registration/icp.h>
#include <pcl/surface/mls.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/surface/gp3.h>
//#include <pcl/PCLPointCloud2.h>
#include <time.h>

using namespace std;
using namespace cv;
using namespace cv::detail;

typedef vector <double> record_t;
typedef vector <record_t> data_t;

class Pose {

public:
Pose(int argc, char* argv[]);
// Default command line args
vector<int> img_numbers;
double minDisparity = 64;
int boundingBox = 20;
int rows = 0, cols = 0, cols_start_aft_cutout = 0;
int jump_pixels = 1;

bool mesh_surface = false;
bool smooth_surface = false;
int polynomial_order = 2;
double search_radius = 0.02, sqr_gauss_param = 0.02;
bool downsample = false;
bool downsample_transform = false;
string downsample_transform_file = "";
double voxel_size = 0.01; //in meters
bool visualize = false;
bool align_point_cloud = false;
string read_PLY_filename0 = "";
string read_PLY_filename1 = "";
string currentDateTimeStr;
//double reduction_ratio = 1;
double focallength = 16.0 / 1000 / 3.75 * 1000000;
double baseline = 600.0 / 1000;
int cutout_ratio = 8;	//how much ratio of masking is to be done on left side of image as this area is not covered in stereo disparity images.
string calib_file = "cam13calib.yml";
Mat Q;
const string imageNumbersFile = "images/image_numbers.txt";
const string dataFilesPrefix = "data_files/";
const string pose_file = "pose.txt";
const string images_times_file = "images.txt";
const string heading_data_file = "hdg.txt";
const string imagePrefix = "/mnt/win/WORK/kentland19jul/22m_extracted_data/left_rect/";
const string disparityPrefix = "/mnt/win/WORK/kentland19jul/22m_extracted_data/disparities/";
const string segmentlblPrefix = "segmentlabels/";

//indices in pose and heading data files
const int tx_ind=3,ty_ind=4,tz_ind=5,qx_ind=6,qy_ind=7,qz_ind=8,qw_ind=9;//,hdg_ind=3;
//translation and rotation between image and head of hexacopter
const double trans_x_hi = -0.300;
const double trans_y_hi = -0.040;
const double trans_z_hi = -0.350;
const double PI = 3.141592653589793238463;
const double theta_xi = -1.1408 * PI / 180;
const double theta_yi = 1.1945 * PI / 180;

//PROCESS: get times in NSECS from images_times_data and search for corresponding or nearby entry in pose_data and heading_data
data_t pose_data;		//header.seq,secs,NSECS,position.x,position.y,position.z,orientation.x,orientation.y,orientation.z,orientation.w
//data_t heading_data;	//header.seq,secs,NSECS,rostime,heading_in_degs
data_t images_times_data;	//header.seq,secs,NSECS
//vectors to store trigger times for easier searching
vector<double> images_times_seq;
vector<double> pose_times_seq;
//vector<double> heading_times_seq;

	vector<pcl::PointXYZRGB> hexPosMAVLinkVec;
	vector<pcl::PointXYZRGB> hexPosFMVec;
	vector<pcl::PointXYZRGB> hexPosFMFittedVec;
	bool displayCamPositions = false;
	string hexPosMAVLinkfilename = "output/hexPosMAVLink.txt";
	string hexPosFMfilename = "output/hexPosFM.txt";
	string hexPosFMFittedfilename = "output/hexPosFMFitted.txt";

bool log_stuff = false;
bool preview = false;
bool try_cuda = true;
string features_type = "orb";
float match_conf = 0.3f;
const string save_log_to = "output/log.txt";
int range_width = -1;
bool use_segment_labels = false;
bool release = true;

vector<Mat> full_images;
vector<Mat> disparity_images;
vector<Mat> segment_maps;
vector<Mat> double_disparity_images;
ofstream log_file;	//logging stuff
vector<ImageFeatures> features;
vector<MatchesInfo> pairwise_matches;

//declaring functions
void readCalibFile();
void printUsage();
string type2str(int type);
int parseCmdArgs(int argc, char** argv);
void readPoseFile();
void readImages();
void findFeatures();
void pairWiseMatching();
void createPtCloud(int img_index, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudrgb);
void createFeaturePtCloud(int img_index, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudrgb);
void transformPtCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudrgb, pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformed_cloudrgb, pcl::registration::TransformationEstimation<pcl::PointXYZRGB, pcl::PointXYZRGB>::Matrix4 transform);
void createPlaneFittedDisparityImages(int i);
pcl::registration::TransformationEstimation<pcl::PointXYZRGB, pcl::PointXYZRGB>::Matrix4 generateTmat(record_t pose);
int binarySearchUsingTime(vector<double> seq, int l, int r, double time);
int binarySearchImageTime(int l, int r, int imageNumber);
int data_index_finder(int image_number);
void printPoints(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, int num);
const string currentDateTime();
double getMean(Mat disp_img, bool planeFitted);
double getVariance(Mat disp_img, bool planeFitted);
void visualize_pt_cloud(bool showcloud, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudrgb, bool showmesh, pcl::PolygonMesh &mesh, string pt_cloud_name);
pcl::PointCloud<pcl::PointXYZRGB>::Ptr read_PLY_File(string point_cloud_filename);
void save_pt_cloud_to_PLY_File(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudrgb, string &writePath);
pcl::registration::TransformationEstimation<pcl::PointXYZRGB, pcl::PointXYZRGB>::Matrix4 generate_tf_of_Matched_Keypoints_Point_Cloud
(int img_index, vector<pcl::registration::TransformationEstimation<pcl::PointXYZRGB, pcl::PointXYZRGB>::Matrix4> &t_FMVec, 
pcl::registration::TransformationEstimation<pcl::PointXYZRGB, pcl::PointXYZRGB>::Matrix4 t_mat_MAVLink);
pcl::registration::TransformationEstimation<pcl::PointXYZRGB, pcl::PointXYZRGB>::Matrix4 runICPalignment(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_out);
pcl::PointCloud<pcl::PointXYZRGB>::Ptr downsamplePtCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudrgb);

};
