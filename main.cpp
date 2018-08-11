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
	cout << "v_8th Aug" << endl;
	
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

