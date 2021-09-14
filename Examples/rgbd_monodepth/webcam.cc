#include <iostream>
#include <algorithm> 
#include <fstream>
#include <chrono>
#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>
#include <System.h>

using namespace std;
int main(int argc, char **argv) 
{
   if(argc != 3) 
   {
     cerr << endl << "Usage: ./path_to_PF_ORB path_to_vocabulary path_to_settings" << endl;
     return 1;
   }
   cv::VideoCapture cap(0);
   if (!cap.isOpened())
   {
     cerr << endl  <<"Could not open camera feed."  << endl;
     return -1;
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

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM2::System SLAM(argv[1],argv[2],ORB_SLAM2::System::MONOCULAR,true);

   cout << endl << "-------" << endl;
   cout << "Start processing sequence ..." << endl;

   // Main loop
   int timeStamps=0;
   for(;;timeStamps++)
   {
     //Create a new Mat
     cv::Mat frame;

     //Send the captured frame to the new Mat
     cap>>frame;

     cv::resize(frame, frame, cv::Size(m_img_width, m_img_height), m_img_width / 640,m_img_height / 480);

     // Pass the image to the SLAM system
     SLAM.TrackMonocular(frame, timeStamps);
      

    //cv::Mat Tcw = SLAM.TrackRGB(frame, (double)timeStamps, timeStamps);
    }

   // Stop all threads
   SLAM.densemap_request_finish();
   SLAM.Shutdown();
   std::cout << "\ndone!" << std::endl;



    // Save camera trajectory
    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");
    return 0;
}
