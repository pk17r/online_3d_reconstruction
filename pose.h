#include <iostream>
#include <fstream>
#include <string>
#include "opencv2/opencv_modules.hpp"
#include <opencv2/core/utility.hpp>
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/stitching/detail/autocalib.hpp"
#include "opencv2/stitching/detail/blenders.hpp"
#include "opencv2/stitching/detail/timelapsers.hpp"
#include "opencv2/stitching/detail/camera.hpp"
#include "opencv2/stitching/detail/exposure_compensate.hpp"
#include "opencv2/stitching/detail/matchers.hpp"
#include "opencv2/stitching/detail/motion_estimators.hpp"
#include "opencv2/stitching/detail/seam_finders.hpp"
#include "opencv2/stitching/detail/util.hpp"
#include "opencv2/stitching/detail/warpers.hpp"
#include "opencv2/stitching/warpers.hpp"
#include "opencv2/calib3d.hpp"
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>
#include <pcl/console/parse.h>
#include <pcl/common/transforms.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/point_types.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/registration/transformation_estimation.h>
#include <pcl/registration/transformation_estimation_svd.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/registration/icp.h>
#include <time.h>

//#define ENABLE_LOG 1
//#define LOG(msg) std::cout << msg
//#define LOGLN(msg) std::cout << msg << std::endl

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

bool downsample = false;
double voxel_size = 0.01; //in meters
bool visualize = false;
bool join_point_clouds = false;
bool align_point_cloud = false;
string read_PLY_filename0 = "";
string read_tf_filename0 = "";
string read_PLY_filename1 = "";
string read_tf_filename1 = "";
string currentDateTimeStr;
double reduction_ratio = 1;
double focallength = 16.0 / 1000 / 3.75 * 1000000;
double baseline = 600.0 / 1000;
int cutout_ratio = 8;
string calib_file = "cam13calib.yml";
Mat K1, K2, D1, D2;
Mat R1, R2, P1, P2;
Mat R, E, F, Q;
Vec3d T;
string dataFilesPrefix = "data_files/";
string pose_file = "pose.txt";
string images_times_file = "images.txt";
string heading_data_file = "hdg.txt";
string imagePrefix = "/mnt/win/WORK/kentland19jul/22m_extracted_data/left_rect/";
string disparityPrefix = "/mnt/win/WORK/kentland19jul/22m_extracted_data/disparities/";
string segmentlblPrefix = "segmentlabels/";

const int tx_ind=3,ty_ind=4,tz_ind=5,qx_ind=6,qy_ind=7,qz_ind=8,qw_ind=9,hdg_ind=3;
//translation and rotation between image and head of hexacopter
const double trans_x_hi = -0.300;
const double trans_y_hi = -0.040;
const double trans_z_hi = -0.350;
const double PI = 3.141592653589793238463;
const double theta_xi = -1.1408 * PI / 180;
const double theta_yi = 1.1945 * PI / 180;

//PROCESS: get NSECS from images_times_data and search for corresponding or nearby entry in pose_data and heading_data
data_t pose_data;		//header.seq,secs,NSECS,position.x,position.y,position.z,orientation.x,orientation.y,orientation.z,orientation.w
data_t heading_data;	//header.seq,secs,NSECS,rostime,heading_in_degs
data_t images_times_data;	//header.seq,secs,NSECS
//vectors to store trigger times for easier searching
vector<double> images_times_seq;
vector<double> pose_times_seq;
vector<double> heading_times_seq;

bool log_stuff = false;
bool preview = false;
bool try_cuda = true;
double work_megapix = 0;
double seam_megapix = 0.1;
double compose_megapix = -1;
float conf_thresh = 1.f;
string features_type = "orb";
string ba_cost_func = "ray";
string ba_refine_mask = "xxxxx";
bool do_wave_correct = true;
WaveCorrectKind wave_correct = detail::WAVE_CORRECT_VERT;
bool save_graph = true;
string save_graph_to = "";
string warp_type = "spherical";
int expos_comp_type = ExposureCompensator::GAIN_BLOCKS;
float match_conf = 0.3f;
string seam_find_type = "voronoi";
int blend_type = Blender::MULTI_BAND;
int timelapse_type = Timelapser::AS_IS;
float blend_strength = 5;
string result_name = "output/result.jpg";
bool timelapse = false;
int range_width = -1;
bool use_segment_labels = false;

double work_scale = 1, seam_scale = 1, compose_scale = 1;
bool is_work_scale_set = false, is_seam_scale_set = false, is_compose_scale_set = false;
double seam_work_aspect = 1;
Mat full_img, img;
vector<Mat> images;
vector<Mat> full_images;
vector<Mat> disparity_images;
vector<Mat> segment_maps;
vector<Mat> double_disparity_images;
vector<Size> full_img_sizes;
ofstream f;	//logging stuff
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
void transformPtCloud2(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudrgb, pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformed_cloudrgb, pcl::registration::TransformationEstimation<pcl::PointXYZRGB, pcl::PointXYZRGB>::Matrix4 transform);
void transformPtCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudrgb, pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformed_cloudrgb, Eigen::Affine3f transform_2);
void createPlaneFittedDisparityImages();
pcl::registration::TransformationEstimation<pcl::PointXYZRGB, pcl::PointXYZRGB>::Matrix4 generateTmat(record_t pose);
int binarySearch(vector<double> seq, int l, int r, double x);
int binarySearchImageTime(int l, int r, double x);
int* data_index_finder(int image_number);
void printPoints(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, int num);
void generate_Matched_Keypoints_Point_Clouds(int img0_index, int img1_index,
	pcl::registration::TransformationEstimation<pcl::PointXYZRGB, pcl::PointXYZRGB>::Matrix4 t_mat0,
	pcl::registration::TransformationEstimation<pcl::PointXYZRGB, pcl::PointXYZRGB>::Matrix4 t_mat1,
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud_t0, pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud_t1);
const string currentDateTime();
double getMean(Mat disp_img, bool planeFitted);
double getVariance(Mat disp_img, bool planeFitted);
void visualize_pt_cloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudrgb, string pt_cloud_name);
pcl::PointCloud<pcl::PointXYZRGB>::Ptr read_PLY_File(string point_cloud_filename);
void save_pt_cloud_to_PLY_File(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudrgb, string &writePath);

};
