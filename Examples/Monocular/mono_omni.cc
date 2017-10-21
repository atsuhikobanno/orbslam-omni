/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* This version was modified for panoramic omnidirectional images
* by Atsuhiko Banno (atsuhiko.banno at aist.go.jp)
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/

#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>
#include<iomanip>

#include<opencv2/core/core.hpp>

#include"System.h"

using namespace std;

void LoadImages(int nTimes, vector<string> &vstrImageFilenames, vector<double> &vTimestamps);

int main(int argc, char **argv)
{
	if (argc != 4) {
		cerr << endl << "Usage: ./mono_omni path_to_vocabulary path_to_settings path_to_sequence number_of_images" << endl;
		return 1;
	}

	// Retrieve paths to images
	vector<string> vstrImageFilenames;
	vector<double> vTimestamps;
	LoadImages(atoi(argv[3]), vstrImageFilenames, vTimestamps);

	int nImages = vstrImageFilenames.size();

	bool mbOmni = true;

	// Create SLAM system. It initializes all system threads and gets ready to process frames.
	ORB_SLAM2::System SLAM(argv[1], argv[2], ORB_SLAM2::System::MONOCULAR, mbOmni, true);

	// Vector for tracking time statistics
	vector<float> vTimesTrack;
	vTimesTrack.resize(nImages);

	cout << "Start processing sequence ..." << endl;
	cout << "Images in the sequence: " << nImages << endl << endl;

	usleep(5.0e+6);

	// Main loop
	cv::Mat im;
	for (int ni = 0; ni < nImages; ni++) {
		// Read image from file
		im = cv::imread(vstrImageFilenames[ni], CV_LOAD_IMAGE_UNCHANGED);
		double tframe = vTimestamps[ni];

		if (im.empty()) {
			cerr << endl << "Failed to load image at: " << vstrImageFilenames[ni] << endl;
			return 1;
		}

		std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();

		// Pass the image to the SLAM system
		SLAM.TrackMonocular(im, tframe);

		std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();

		double ttrack = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1).count();

		vTimesTrack[ni] = ttrack;

		// Wait to load the next frame
		double T = 0;
		if (ni < nImages - 1)
			T = vTimestamps[ni + 1] - tframe;
		else if (ni > 0)
			T = tframe - vTimestamps[ni - 1];

		if (ttrack < T)
			usleep((T - ttrack)*1.0e+6);

		if ((ni + 1) % 500 == 0 || ni == nImages - 1){
			// Save map points
			SLAM.SaveMapsOmni("Map_shapes.txt");

			// Save camera trajectory
			SLAM.SaveKeyFrameTrajectoryOmni("KeyFrame_cameras.txt");

			//	Save Image File name
			SLAM.SaveImageFilesOmni("KeyFrameFileNames.txt", vstrImageFilenames);
		}
	}

	// Save map points
	SLAM.SaveMapsOmni("Map_shapes.txt");

	// Save camera trajectory
	SLAM.SaveKeyFrameTrajectoryOmni("KeyFrame_cameras.txt");

	//	Save Image File name
	SLAM.SaveImageFilesOmni("KeyFrameFileNames.txt", vstrImageFilenames);

	// Stop all threads
	SLAM.Shutdown();

	// Tracking time statistics
	sort(vTimesTrack.begin(), vTimesTrack.end());
	float totaltime = 0;
	for(int ni=0; ni<nImages; ni++)
		totaltime += vTimesTrack[ni];

	cout << "-------" << endl << endl;
	cout << "median tracking time: " << vTimesTrack[nImages/2] << endl;
	cout << "mean tracking time: " << totaltime/nImages << endl;


	return 0;
}

void LoadImages(int nTimes, vector<string> &vstrImageFilenames, vector<double> &vTimestamps)
{
	vTimestamps.resize(nTimes);
	vstrImageFilenames.resize(nTimes);

	for(int i=0; i<nTimes; i++){
		stringstream ss;
		ss << setfill('0') << setw(4) << i;
		vstrImageFilenames[i] = "D:/Data/Image/Data10/img_" + ss.str() + ".bmp";

		vTimestamps[i] = 0.25 * i;
	}
}
