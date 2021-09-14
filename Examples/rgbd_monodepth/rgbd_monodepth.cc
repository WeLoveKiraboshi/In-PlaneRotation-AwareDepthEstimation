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
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/
#pragma once

//#include "DepthPredictCommunicator.h"
//#include "KinectDataProvider.h"
//#include "DatasetConfigure.h"


#include<iostream>
#include<algorithm>
#include<fstream>
#include<iomanip>
#include<chrono>
#include<sstream>
#include<array>
#include<opencv2/core/core.hpp>
#include<opencv2/highgui.hpp>
#include <unistd.h>
#include "System.h"
/*
#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>
#include<opencv2/core/core.hpp>
#include <unistd.h>
*/




#define WITH_SLAM
#define IMAGE_NUMS 2000

#define SERVER_PORT 6666
//#define SERVER_IP "10.200.129.201"
#define SERVER_IP "10.200.129.187"
//#define SERVER_IP "127.0.0.1"

using namespace std;


void LoadImages(const string &strAssociationFilename, vector<string> &vstrImageFilenamesRGB,
                vector<string> &vstrImageFilenamesD, vector<double> &vTimestamps);


int main(int argc, char **argv)
{
    if(argc != 5)
    {
        cerr << endl << "Usage: ./rgbd_tum path_to_vocabulary path_to_settings path_to_sequence path_to_association : argc = " << argc << endl;
        return 1;
    }

    // Retrieve paths to images
    vector<string> vstrImageFilenamesRGB;
    vector<string> vstrImageFilenamesD;
    vector<double> vTimestamps;
    string strAssociationFilename = string(argv[4]);
    LoadImages(strAssociationFilename, vstrImageFilenamesRGB, vstrImageFilenamesD, vTimestamps);

    // Check consistency in the number of images and depthmaps
    int nImages = vstrImageFilenamesRGB.size();
    if(vstrImageFilenamesRGB.empty())
    {
        cerr << endl << "No images found in provided path." << endl;
        return 1;
    }
    else if(vstrImageFilenamesD.size()!=vstrImageFilenamesRGB.size())
    {
        cerr << endl << "Different number of images for rgb and depth." << endl;
        return 1;
    }

    //Check settings file
    const string sSettingfile = argv[2];
    cv::FileStorage fsSettings(sSettingfile.c_str(), cv::FileStorage::READ);
    if(!fsSettings.isOpened())
    {
        cerr << "Failed to open settings file at: " << sSettingfile << endl;
        exit(-1);
    }
    int m_img_width = fsSettings["Camera.width"];
    int m_img_height = fsSettings["Camera.height"];
    int m_net_width = fsSettings["Camera.net_width"];
    int m_net_height = fsSettings["Camera.net_height"];
    fsSettings.release();


    std::chrono::steady_clock::time_point system_overall_t1 = std::chrono::steady_clock::now();

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM2::System SLAM(argv[1],argv[2],ORB_SLAM2::System::RGBD,true);

    std::cout << "System class SLAM constructor loading done" << std::endl;

    // Vector for tracking time statistics
    vector<float> vTimesTrack;
    vTimesTrack.resize(nImages);
    SLAM.calc_processing_time(nImages);

    cout << endl << "-------" << endl;
    cout << "Start processing sequence ..." << endl;
    cout << "Images in the sequence: " << nImages << endl << endl;


    // Main loop
    cv::Mat imRGB_, imD_, imRGB, imD;
    for(int ni=0; ni<nImages; ni++)
    {
        // Read image and depthmap from file
        imRGB_ = cv::imread(string(argv[3])+"/"+vstrImageFilenamesRGB[ni],CV_LOAD_IMAGE_UNCHANGED);
        imD_ = cv::imread(string(argv[3])+"/"+vstrImageFilenamesD[ni],CV_LOAD_IMAGE_UNCHANGED);
        
        double tframe = vTimestamps[ni];
        cv::resize(imRGB_, imRGB, cv::Size(m_img_width, m_img_height), m_img_width / 640,m_img_height / 480);
        cv::resize(imD_, imD, cv::Size(m_img_width, m_img_height), m_img_width / 640,m_img_height / 480);


        float mDepthMapFactor = 1.0f/5000;
        if(/*(fabs(mDepthMapFactor-1.0f)>1e-5) || */imD.type()!=CV_32F)
            imD.convertTo(imD,CV_32F,mDepthMapFactor);


        if(imRGB.empty())
        {
            cerr << endl << "Failed to load image at: "
                 << string(argv[3]) << "/" << vstrImageFilenamesRGB[ni] << endl;
            return 1;
        }

        //cv::namedWindow("ORB-SLAM2: Current Frame");
        //cv::imshow("ORB-SLAM2: Current Frame", imRGB);
        //char key = cv::waitKey(10);
        //if(key == 27) break;


        std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();


        // Pass the image to the SLAM system
        cv::Mat Tcw = SLAM.TrackRGBD(imRGB, imD, tframe, ni);
        //Tcw = SLAM.TrackRGBD(imRGB, imD_float, time(0), isKeyframe, trackingState, frame_id);

        std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();


        double ttrack= std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();
        //std::cout << "msec = " << std::chrono::duration_cast<std::chrono::milliseconds>(t2-t1).count() << " ttrack = " << ttrack << std::endl;

        vTimesTrack[ni]=ttrack;

        // Wait to load the next frame
        double T=0;
        if(ni<nImages-1)
            T = vTimestamps[ni+1]-tframe;
        else if(ni>0)
            T = tframe-vTimestamps[ni-1];

        if(ttrack<T)
            usleep((T-ttrack)*1e6);

        //cv::imshow("CNN-MOnoFusion_Frame", imRGB);
        //char key = cv::waitKey(30);
        //if(key == 27) break;
    }


    SLAM.densemap_request_finish();
    std::cout << "\ndone!" << std::endl;
    // Stop all threads
    SLAM.Shutdown();
    // Save camera trajectory
    //SLAM.SaveTrajectoryKITTI("CameraTrajectory_kitti.txt");
    //SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory_kitti_stereo_orbslam.txt");


    // Tracking time statistics
    sort(vTimesTrack.begin(),vTimesTrack.end());
    float totaltime = 0;
    for(int ni=0; ni<nImages; ni++)
    {
        totaltime+=vTimesTrack[ni];
    }
    cout << "-------" << endl << endl;
    if(vTimesTrack.size() != nImages){
        std::cerr << "the size of vTimesTrack and nImages is different... at main.cpp" << std::endl;
        exit(-1);
    }
    cout << "Tracking vector size: " << vTimesTrack.size() << endl;
    cout << "median tracking time: " << vTimesTrack[nImages/2] << endl;
    cout << "mean tracking time: " << totaltime/nImages << endl;
    cout << "min tracking time: " << vTimesTrack[0] << endl;
    cout << "max tracking time: " << vTimesTrack[vTimesTrack.size()-1] << endl;

    SLAM.show_processing_time();

    std::chrono::steady_clock::time_point system_overall_t2 = std::chrono::steady_clock::now();
    double system_total_time = std::chrono::duration_cast<std::chrono::duration<double> >(system_overall_t2 - system_overall_t1).count();
    cout << endl;
    cout << endl;
    cout << "total system processing time: " << system_total_time << endl;

    return 0;

}

void LoadImages(const string &strAssociationFilename, vector<string> &vstrImageFilenamesRGB,
                vector<string> &vstrImageFilenamesD, vector<double> &vTimestamps)
{
    ifstream fAssociation;
    fAssociation.open(strAssociationFilename.c_str());
    while(!fAssociation.eof())
    {
        string s;
        getline(fAssociation,s);
        if(!s.empty())
        {
            stringstream ss;
            ss << s;
            double t;
            string sRGB, sD;
            ss >> t;
            vTimestamps.push_back(t);
            ss >> sRGB;
            vstrImageFilenamesRGB.push_back(sRGB);
            ss >> t;
            ss >> sD;
            vstrImageFilenamesD.push_back(sD);

        }
    }
}
