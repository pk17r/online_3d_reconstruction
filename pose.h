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


//#define ENABLE_LOG 1
//#define LOG(msg) std::cout << msg
//#define LOGLN(msg) std::cout << msg << std::endl

using namespace std;
using namespace cv;
using namespace cv::detail;

// Default command line args
vector<int> img_numbers = { 952, 953 };
double minDisparity = 64;
int boundingBox = 20;
int rows = 0, cols = 0, cols_start_aft_cutout = 0;

double reduction_ratio = 1;
double focallength = 16.0 / 1000 / 3.75 * 1000000;
double baseline = 600.0 / 1000;
int cutout_ratio = 8;
string calib_file = "cam13calib.yml";
Mat K1, K2, D1, D2;
Mat R1, R2, P1, P2;
Mat R, E, F, Q;
Vec3d T;
string pose_file = "pose.txt";

typedef vector <double> record_t;
typedef vector <record_t> data_t;

vector<int> pose_sequence;
data_t pose_data;		//header.seq,position.x,position.y,position.z,orientation.x,orientation.y,orientation.z,orientation.w

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
string imagePrefix = "images/";
string disparityPrefix = "disparities/";
string segmentlblPrefix = "segmentlabels/";
ofstream f;	//logging stuff
vector<ImageFeatures> features;
vector<MatchesInfo> pairwise_matches;

//declaring functions
void readCalibFile();
static void printUsage();
string type2str(int type);
static int parseCmdArgs(int argc, char** argv);
istream& operator >> ( istream& ins, record_t& record );
istream& operator >> ( istream& ins, data_t& data );
inline int binary_search_find_index(std::vector<int> v, int data);
inline void readPoseFile();
inline void readImages();
inline void findFeatures();
inline void pairWiseMatching();
void createPtCloud(int img_index, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformed_cloud);
