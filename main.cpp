#include "pose.h"

using namespace cv;
using namespace std;

void plotMatches(string img1name, string img2name)
{
	cv::Mat img1 = imread(img1name);
    cv::Mat img2 = imread(img2name);
	cout << "read images" << endl;
	
	unsigned long t_AAtime=0, t_BBtime=0;
	float t_pt;
	float t_fpt;
	t_AAtime = getTickCount(); 
	cout << "time start.." << endl;

	cv::Mat gray1;
	cv::Mat gray2;
	cv::cvtColor(img1, gray1, CV_BGR2GRAY);
	cv::cvtColor(img2, gray2, CV_BGR2GRAY);
	cout << "converted images to grayscale" << endl;
	
	Ptr<cuda::ORB> orb1 = cuda::ORB::create(), orb2 = cuda::ORB::create();
	cout << "created orb ptrs" << endl;
	
	cuda::GpuMat gray_gpu1(gray1),gray_gpu2(gray2);
	vector<KeyPoint> keypoints1, keypoints2;
	cuda::GpuMat descriptors1, descriptors2;
	
	orb1->detectAndCompute(gray_gpu1, cuda::GpuMat(), keypoints1, descriptors1);
	orb2->detectAndCompute(gray_gpu2, cuda::GpuMat(), keypoints2, descriptors2);
	
	cout << "FOUND " << keypoints1.size() << " keypoints on first image" << endl;
	cout << "FOUND " << keypoints2.size() << " keypoints on second image" << endl;
	
	Ptr<cuda::DescriptorMatcher> matcher = cv::cuda::DescriptorMatcher::createBFMatcher(cv::NORM_HAMMING);
	vector<vector<DMatch>> matches;
	cout << "created matcher" << endl;
	matcher->knnMatch(descriptors1, descriptors2, matches, 2);
	cout << "matched descriptors! matches: " << matches.size() << endl;
	
	vector<DMatch> all_matches;
	vector<DMatch> good_matches1;
	vector<DMatch> good_matches2;
	for(int k = 0; k < matches.size(); k++)
	{
		//cout << matches[k][0].distance << "/" << matches[k][1].distance << " " << matches[k][0].queryIdx << "/" << matches[k][1].queryIdx << " " << matches[k][0].trainIdx << "/" << matches[k][1].trainIdx << (matches[k][0].distance < 0.5 * matches[k][1].distance ? " accepted" : "") << endl;
		all_matches.push_back(matches[k][0]);
		if(matches[k][0].distance < 0.5 * matches[k][1].distance)
		{
			good_matches1.push_back(matches[k][0]);
			if (matches[k][0].distance < 40)
			{
				//cout << matches[k][0].distance << "/" << matches[k][1].distance << " " << matches[k][0].imgIdx << "/" << matches[k][1].imgIdx << " " << matches[k][0].queryIdx << "/" << matches[k][1].queryIdx << " " << matches[k][0].trainIdx << "/" << matches[k][1].trainIdx << endl;
				good_matches2.push_back(matches[k][0]);
			}
			
		}
	}
	
	t_BBtime = getTickCount();
	t_pt = (t_BBtime - t_AAtime)/getTickFrequency();
	t_fpt = 1/t_pt;
	printf("%.4lf sec/ %.4lf fps\n",  t_pt, t_fpt );
	cout << "matches " << matches.size() << " good_matches1 " << good_matches1.size() << " good_matches2 " << good_matches2.size() << endl;
	
	Mat img_matches0, img_matches1, img_matches2;
	drawMatches(img1, keypoints1, img2, keypoints2, all_matches, img_matches0);
	drawMatches(img1, keypoints1, img2, keypoints2, good_matches1, img_matches1);
	drawMatches(img1, keypoints1, img2, keypoints2, good_matches2, img_matches2);
	//namedWindow("all_matches", 0);
    //namedWindow("good_matches1", 0);
    //namedWindow("good_matches2", 0);
    //imshow("all_matches", img_matches0);
	//imshow("good_matches1", img_matches1);
	//imshow("good_matches2", img_matches2);
	//waitKey(1);
	vector<int> compression_params;
    compression_params.push_back(CV_IMWRITE_PNG_COMPRESSION);
    compression_params.push_back(9);
    
    try {
        imwrite("output/all_matches.png", img_matches0, compression_params);
        imwrite("output/good_matches1.png", img_matches1, compression_params);
        imwrite("output/good_matches2.png", img_matches2, compression_params);
    }
    catch (runtime_error& ex) {
        fprintf(stderr, "Exception converting image to PNG format: %s\n", ex.what());
    }
}

int main(int argc, char* argv[])
{
	cout << 
		  "\n**********   Unmanned Systems Lab    **********"
		"\n\n********** 3D Reconstruction Program **********"
		"\n\nAuthor: Prashant Kumar"
		"\n\nHelp  ./pose --help"
		"\n"
		<< endl;
	
	plotMatches("images/1248.png","images/1251.png");
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

