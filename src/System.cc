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



#include "System.h"
#include "Converter.h"
#include <thread>
#include <pangolin/pangolin.h>
#include <iomanip>
#include <unistd.h>

namespace ORB_SLAM2
{

System::System(const string &strVocFile, const string &strSettingsFile, const eSensor sensor,
               const bool bUseViewer):mSensor(sensor)/*, mpViewer(static_cast<Viewer*>(NULL))*/, mbReset(false),mbActivateLocalizationMode(false),
        mbDeactivateLocalizationMode(false)
{
    // Output welcome message
    cout << endl <<
    "ORB-SLAM2 Copyright (C) 2014-2016 Raul Mur-Artal, University of Zaragoza." << endl <<
    "This program comes with ABSOLUTELY NO WARRANTY;" << endl  <<
    "This is free software, and you are welcome to redistribute it" << endl <<
    "under certain conditions. See LICENSE.txt." << endl << endl;

    cout << "Input sensor was set to: ";

    if(mSensor==MONOCULAR)
        cout << "Monocular" << endl;
    else if(mSensor==STEREO)
        cout << "Stereo" << endl;
    else if(mSensor==RGBD)
        cout << "RGB-D" << endl;

    //Check settings file
    cv::FileStorage fsSettings(strSettingsFile.c_str(), cv::FileStorage::READ);
    if(!fsSettings.isOpened())
    {
        cerr << "Failed to open settings file at: " << strSettingsFile << endl;
        exit(-1);
    }
    K = cv::Mat(cv::Matx33f(fsSettings["Camera.fx"], 0, fsSettings["Camera.cx"],
            0, fsSettings["Camera.fy"], fsSettings["Camera.cy"],
            0, 0, 1));
    bf = fsSettings["Camera.bf"];
    m_img_width = fsSettings["Camera.width"];
    m_img_height = fsSettings["Camera.height"];
    m_net_width = fsSettings["Camera.net_width"];
    m_net_height = fsSettings["Camera.net_height"];

    per_frame_wait = fsSettings["Camera.cv_wait_delay"];
    fsSettings.release();



    //Load ORB Vocabulary
    cout << endl << "Loading ORB Vocabulary. This could take a while..." << endl;

    mpVocabulary = new ORBVocabulary();
    bool bVocLoad = mpVocabulary->loadFromTextFile(strVocFile);
    if(!bVocLoad)
    {
        cerr << "Wrong path to vocabulary. " << endl;
        cerr << "Falied to open at: " << strVocFile << endl;
        exit(-1);
    }
    cout << "Vocabulary loaded!" << endl << endl;

    //Create KeyFrame Database
    mpKeyFrameDatabase = new KeyFrameDatabase(*mpVocabulary);

    //Create the Map
    mpMap = new Map();

    //Create Drawers. These are used by the Viewer
    mpFrameDrawer = new FrameDrawer(mpMap);
    mpMapDrawer = new MapDrawer(mpMap, strSettingsFile);



    //Initialize the Tracking thread
    //(it will live in the main thread of execution, the one that called this constructor)
    mpTracker = new Tracking(this, mpVocabulary, mpFrameDrawer, mpMapDrawer,
                             mpMap, mpKeyFrameDatabase, strSettingsFile, mSensor);

    //Initialize the Local Mapping thread and launch
    mpLocalMapper = new LocalMapping(mpMap, mSensor==MONOCULAR);
    mptLocalMapping = new thread(&ORB_SLAM2::LocalMapping::Run,mpLocalMapper);

    //Initialize the Loop Closing thread and launch
    mpLoopCloser = new LoopClosing(mpMap, mpKeyFrameDatabase, mpVocabulary, mSensor!=MONOCULAR);
    mptLoopClosing = new thread(&ORB_SLAM2::LoopClosing::Run, mpLoopCloser);


    mpDepthEstimation = new DepthEstimation(strSettingsFile);
    //mptDepthEstimation = new thread(&ORB_SLAM2::DepthEstimation::Run, mpDepthEstimation);

    mpDenseMap = new DenseMap(strSettingsFile, "");
    frame_queue = new KeyFrameQueue();
    mpDenseMapDrawer = new DenseMapDrawer(mpDenseMap, strSettingsFile);

    mpDepthPredictor = new DepthPredictor(mpMap, frame_queue, mpDenseMap, strSettingsFile);
    mptDepthPredictor = new thread(&DepthPredictor::run, mpDepthPredictor);

    //Initialize the Viewer thread and launch
    if(bUseViewer)
    {
        mpDenseMapViewer = new DenseMapViewer(mpDenseMapDrawer, strSettingsFile/*, mpFrameDrawer*/);
        mptDenseMapViewer = new thread(&DenseMapViewer::run, mpDenseMapViewer);
        mpTracker->SetDenseMapViewer(mpDenseMapViewer);
        /*
        mpViewer = new Viewer(this, mpFrameDrawer,mpMapDrawer,mpTracker,strSettingsFile);
        mptViewer = new thread(&Viewer::Run, mpViewer);
        mpTracker->SetViewer(mpViewer);
        */
    }

    //Set pointers between threads
    mpTracker->SetLocalMapper(mpLocalMapper);
    mpTracker->SetLoopClosing(mpLoopCloser);
    mpTracker->SetDepthEstimation(mpDepthEstimation);

    mpLocalMapper->SetTracker(mpTracker);
    mpLocalMapper->SetLoopCloser(mpLoopCloser);

    mpLoopCloser->SetTracker(mpTracker);
    mpLoopCloser->SetLocalMapper(mpLocalMapper);

    mpDepthEstimation->SetSystem(this);
    mpDepthEstimation->SetTracker(mpTracker);

    while(!mpDepthEstimation->isReady()) {
        usleep(5000);
    }
}

cv::Mat System::TrackStereo(const cv::Mat &imLeft, const cv::Mat &imRight, const double &timestamp)
{
    if(mSensor!=STEREO)
    {
        cerr << "ERROR: you called TrackStereo but input sensor was not set to STEREO." << endl;
        exit(-1);
    }   

    // Check mode change
    {
        unique_lock<mutex> lock(mMutexMode);
        if(mbActivateLocalizationMode)
        {
            mpLocalMapper->RequestStop();

            // Wait until Local Mapping has effectively stopped
            while(!mpLocalMapper->isStopped())
            {
                usleep(1000);
            }

            mpTracker->InformOnlyTracking(true);
            mbActivateLocalizationMode = false;
        }
        if(mbDeactivateLocalizationMode)
        {
            mpTracker->InformOnlyTracking(false);
            mpLocalMapper->Release();
            mbDeactivateLocalizationMode = false;
        }
    }

    // Check reset
    {
    unique_lock<mutex> lock(mMutexReset);
    if(mbReset)
    {
        mpTracker->Reset();
        mbReset = false;
    }
    }

    cv::Mat Tcw = mpTracker->GrabImageStereo(imLeft,imRight,timestamp);

    unique_lock<mutex> lock2(mMutexState);
    mTrackingState = mpTracker->mState;
    mTrackedMapPoints = mpTracker->mCurrentFrame.mvpMapPoints;
    mTrackedKeyPointsUn = mpTracker->mCurrentFrame.mvKeysUn;
    return Tcw;
}
/*
cv::Mat System::TrackRGBD(const cv::Mat &im, const cv::Mat &depthmap, const double &timestamp)
{
    if(mSensor!=RGBD)
    {
        cerr << "ERROR: you called TrackRGBD but input sensor was not set to RGBD." << endl;
        exit(-1);
    }    

    // Check mode change
    {
        unique_lock<mutex> lock(mMutexMode);
        if(mbActivateLocalizationMode)
        {
            mpLocalMapper->RequestStop();

            // Wait until Local Mapping has effectively stopped
            while(!mpLocalMapper->isStopped())
            {
                usleep(1000);
            }

            mpTracker->InformOnlyTracking(true);
            mbActivateLocalizationMode = false;
        }
        if(mbDeactivateLocalizationMode)
        {
            mpTracker->InformOnlyTracking(false);
            mpLocalMapper->Release();
            mbDeactivateLocalizationMode = false;
        }
    }

    // Check reset
    {
    unique_lock<mutex> lock(mMutexReset);
    if(mbReset)
    {
        mpTracker->Reset();
        mbReset = false;
    }
    }

    cv::Mat Tcw = mpTracker->GrabImageRGBD(im,depthmap,timestamp);

    unique_lock<mutex> lock2(mMutexState);
    mTrackingState = mpTracker->mState;
    mTrackedMapPoints = mpTracker->mCurrentFrame.mvpMapPoints;
    mTrackedKeyPointsUn = mpTracker->mCurrentFrame.mvKeysUn;
    return Tcw;
}
*/

cv::Mat System::TrackRGBD(const cv::Mat &im, const cv::Mat &_imD, const double &timestamp, int frame_id)
{
    if(mSensor!=RGBD)
    {
        cerr << "ERROR: you called TrackMonocular but input sensor was not set to Monocular." << endl;
        exit(-1);
    }

    // Check mode change
    {
        unique_lock<mutex> lock(mMutexMode);
        if(mbActivateLocalizationMode)
        {
            mpLocalMapper->RequestStop();

            // Wait until Local Mapping has effectively stopped
            while(!mpLocalMapper->isStopped())
            {
                usleep(1000);
            }

            mpTracker->InformOnlyTracking(true);
            mbActivateLocalizationMode = false;
        }
        if(mbDeactivateLocalizationMode)
        {
            mpTracker->InformOnlyTracking(false);
            mpLocalMapper->Release();
            mbDeactivateLocalizationMode = false;
        }
    }

    // Check reset
    {
        unique_lock<mutex> lock(mMutexReset);
        if(mbReset)
        {
            mpTracker->Reset();
            mbReset = false;
        }
    }

    cv::Mat Tcw;
    if(frame_id == 0) {
        cv::Mat imD_float;
        cv::Mat imD;
        cv::Mat resizedim;
        cv::resize(im, resizedim, cv::Size(m_net_width, m_net_height), m_net_width / m_img_width,
                   m_net_height / m_img_height);
        if (resizedim.rows % 32 != 0 || resizedim.cols % 32 != 0) {
            std::cout << "irregular input shape : " << resizedim.size() << std::endl;
            exit(1);
        }
        std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
        imD = mpDepthEstimation->predict(resizedim);
        std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
        double t = std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();//static_cast<double>(t2 - t1) / CLOCKS_PER_SEC * 1000.0;//
        vTimesDepthEstimation.push_back(t);
        std::cout << "first frame pred done..." << std::endl;
        cv::resize(imD, imD, cv::Size(m_img_width, m_img_height), m_img_width/m_net_width, m_img_height/m_net_height, cv::INTER_NEAREST);
        imD.convertTo(imD_float, CV_32F);

        //// Pass the images to the SLAM system
        bool isKeyframe = false;
        int trackingState = 2;//stable(OK)
        Tcw = mpTracker->GrabImageRGBD(im, imD_float, timestamp, isKeyframe, trackingState, frame_id);
        if(trackingState == 2)
        {
            densemap_frame_enqueue(im, imD_float, K, Tcw, bf, frame_id, isKeyframe);
        }
    }else{
        bool isKeyframe = false;
        int trackingState = 2;//stable(OK)
        usleep(per_frame_wait);//wait 10000
        Tcw = mpTracker->GrabImageRGBD(im, _imD, timestamp, isKeyframe, trackingState, frame_id);
        if(!Tcw.empty()) {
            cv::Mat imD_float = apply_roll_refine(im, Tcw, frame_id);
            if(trackingState == 2)
            {
                densemap_frame_enqueue(im,  imD_float, K, Tcw, bf, frame_id, isKeyframe);
            }
        }else{
            std::cerr << " Warning !...frame pose Tcw is nullptr... id=" << frame_id  << " _isKeyFrame=" << isKeyframe << " trackingState=" << trackingState << std::endl;
            //vTimesDepthEstimation[frame_id] = -1;
        }
    }
    /*
    bool isKeyframe = false;
    int trackingState = 2;//stable(OK)
    usleep(per_frame_wait);//wait 10000
    Tcw = mpTracker->GrabImageRGBD(im, _imD, timestamp, isKeyframe, trackingState, frame_id);
    if(!Tcw.empty()) {
        if(trackingState == 2)
        {
            densemap_frame_enqueue(im, _imD, K, Tcw, bf, frame_id, isKeyframe);
        }
    }else{
        std::cerr << " Warning !...frame pose Tcw is nullptr... id=" << frame_id  << " _isKeyFrame=" << isKeyframe << " trackingState=" << trackingState << std::endl;
        //vTimesDepthEstimation[frame_id] = -1;
    }*/

    unique_lock<mutex> lock2(mMutexState);
    mTrackingState = mpTracker->mState;
    mTrackedMapPoints = mpTracker->mCurrentFrame.mvpMapPoints;
    mTrackedKeyPointsUn = mpTracker->mCurrentFrame.mvKeysUn;
    return Tcw;

}

void System::calc_processing_time(int _nImages){
    isTimeEval = true;
    //vTimesDepthEstimation.resize(nImages);
    nImages = _nImages;
    vTimesDepthEstimation.clear();
    vTimesProposedRefinement.clear();
    mpDepthPredictor->vTimesDenseFusion.clear();
}


void System::show_processing_time(){
    if(!isTimeEval){
        std::cerr << "isTimeEval flag is false..." << std::endl;
        exit(-1);
    }
    // unit is sec(micro sec)
    float totaltime = 0;
    for(int ni= 0; ni < vTimesDepthEstimation.size(); ni++){
        if(vTimesDepthEstimation[ni] <= 0){
            std::cout << "vTimesDepthEstimation idx = " << ni << " have minus values..." << std::endl;
            vTimesDepthEstimation.erase(vTimesDepthEstimation.begin()+ni);
        }else{
            totaltime+=vTimesDepthEstimation[ni];
        }
    }
    sort(vTimesDepthEstimation.begin(),vTimesDepthEstimation.end());
    cout << "-------" << endl << endl;
    cout << "DepthEstimation vector size: " << vTimesDepthEstimation.size() << endl;
    cout << "median DepthEstimation time: " << vTimesDepthEstimation[int(vTimesDepthEstimation.size()/2)] << endl;
    cout << "mean DepthEstimation time: " << totaltime/vTimesDepthEstimation.size() << endl;
    cout << "min DepthEstimation time: " << vTimesDepthEstimation[0] << endl;
    cout << "max DepthEstimation time: " << vTimesDepthEstimation[vTimesDepthEstimation.size()-1] << endl;
    cout << endl;


    totaltime = 0;
    for(int ni= 0; ni < vTimesProposedRefinement.size(); ni++){
        if(vTimesProposedRefinement[ni] <= 0){
            std::cout << "vTimesProposedRefinement idx = " << ni << " have minus values..." << std::endl;
            vTimesProposedRefinement.erase(vTimesProposedRefinement.begin()+ni);
        }else{
            totaltime+=vTimesProposedRefinement[ni];
        }
    }
    sort(vTimesProposedRefinement.begin(),vTimesProposedRefinement.end());
    cout << "-------" << endl << endl;
    cout << "ProposedRefinement vector size: " << vTimesProposedRefinement.size() << endl;
    cout << "median ProposedRefinement time: " << vTimesProposedRefinement[int(vTimesProposedRefinement.size()/2)] << endl;
    cout << "mean ProposedRefinement time: " << totaltime/vTimesProposedRefinement.size() << endl;
    cout << "min ProposedRefinement time: " << vTimesProposedRefinement[0] << endl;
    cout << "max ProposedRefinement time: " << vTimesProposedRefinement[vTimesProposedRefinement.size()-1] << endl;
    cout << endl;

    mpDepthPredictor->show_DenseFusion_processing_time();
}




cv::Mat System::apply_roll_refine(cv::Mat im, cv::Mat Tcw, int frame_id){
    std::chrono::steady_clock::time_point t1_proposed = std::chrono::steady_clock::now();
    double roll;
    try {
        //std::cout << "y = " << Tcw.at<float>(1, 0) << " , x = " << Tcw.at<float>(0, 0) << std::endl;
        roll = atan2(Tcw.at<float>(1, 0), Tcw.at<float>(0, 0));
    }catch (std::exception& e)
    {
        std::cerr << "apply_roll_refinement() at system.cc ... Exception caught : " << e.what() << std::endl;
        exit(-1);
    }
    double angle = roll * 180 / M_PI;
    if(abs(angle) <= 30){
        m_net_width = 256;
        m_net_height = 192;
    }else if(abs(angle) > 30 and abs(angle) < 60){
        m_net_width = 224;
        m_net_height = 224;
    }else if(abs(angle) >= 60){
        m_net_width = 192;
        m_net_height = 256;
    }
    cv::Mat imD;
    double t;
    if(abs(angle) < 30){ // abs(angle) < 30
        cv::Mat resizedim;
        cv::resize(im, resizedim, cv::Size(m_net_width, m_net_height), m_net_width / m_img_width,m_net_height / m_img_height);
        std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
        imD = mpDepthEstimation->predict(resizedim);
        std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
        t = std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();
        vTimesDepthEstimation.push_back(t);
        cv::resize(imD, imD, cv::Size(m_img_width, m_img_height), m_img_width/m_net_width, m_img_height/m_net_height, cv::INTER_NEAREST);
        imD.convertTo(imD, CV_32F);
    }else{
        //calc extended image size
        int w_rot = (int)(m_img_height*abs(sin(roll)) + m_img_width*abs(cos(roll)));
        int h_rot = (int)(m_img_height*abs(cos(roll)) + m_img_width*abs(sin(roll)));
        cv::Mat dst = cv::Mat::zeros(h_rot, w_rot, CV_8UC3);
        cv::Point2d ctr(m_img_width/2, m_img_height/2);
        cv::Mat mat = cv::getRotationMatrix2D(ctr, angle, 1.0);//Rotation Matrix for
        mat.at<double>(0,2) += -m_img_width/2 + w_rot/2;
        mat.at<double>(1,2) += -m_img_height/2 + h_rot/2;
        cv::warpAffine(im, dst, mat, cv::Size(w_rot, h_rot), CV_INTER_LINEAR, cv::BORDER_TRANSPARENT);

        cv::Mat dst_pad = cv::Mat::zeros((int(dst.rows / 32) + 1) * 32, (int(dst.cols / 32) + 1) * 32, CV_8UC3);
        dst.copyTo(dst_pad.rowRange(0, dst.rows).colRange(0, dst.cols));


        cv::Mat dst_pad_resized;
        cv::resize(dst_pad, dst_pad_resized, cv::Size(m_net_width, m_net_height), m_net_width / dst_pad.cols,m_net_height / dst_pad.rows);
        //cv::imwrite("./test/"+to_string(frame_id)+".png", dst_pad_resized);

        //depth prediction
        std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
        cv::Mat imD_pad_resized = mpDepthEstimation->predict(dst_pad_resized);
        std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
        t = std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();//static_cast<double>(t2 - t1) / CLOCKS_PER_SEC * 1000.0;//
        vTimesDepthEstimation.push_back(t); //[frame_id] = 0.1;
        cv::Mat imD_pad;
        cv::resize(imD_pad_resized, imD_pad, cv::Size(dst_pad.cols, dst_pad.rows), dst_pad.cols/m_net_width, dst_pad.rows/m_net_height, cv::INTER_NEAREST);
        cv::Mat imD_dst;
        imD_pad.rowRange(0, dst.rows).colRange(0, dst.cols).copyTo(imD_dst);

        cv::Point2d ctr_D(imD_dst.cols/2, imD_dst.rows/2);
        cv::Mat mat_D = cv::getRotationMatrix2D(ctr_D, -angle, 1.0);//Rotation Matrix for
        mat_D.at<double>(0,2) += -imD_dst.cols/2 + m_img_width/2;
        mat_D.at<double>(1,2) += -imD_dst.rows/2 + m_img_height/2;
        cv::warpAffine(imD_dst, imD, mat_D, cv::Size(m_img_width, m_img_height), CV_INTER_LINEAR, cv::BORDER_TRANSPARENT);
        imD.convertTo(imD, CV_32F);

    }

    std::chrono::steady_clock::time_point t2_proposed = std::chrono::steady_clock::now();
    double t_proposed = std::chrono::duration_cast<std::chrono::duration<double> >(t2_proposed - t1_proposed).count();
    vTimesProposedRefinement.push_back(t_proposed-t);

    float max = 0;
    for(int u = 0; u < imD.cols; u++)
        for(int v = 0; v < imD.rows; v++)
            if(max < imD.at<float>(v, u))
                max = imD.at<float>(v, u);
   cv::Mat imD_8UC3 = 255 * imD / max;
   imD_8UC3.convertTo(imD_8UC3, CV_8UC3);
   cv::Mat imD_colormap;
   cv::applyColorMap(imD_8UC3, imD_colormap, cv::COLORMAP_JET);
   cv::imwrite("./OurDataset_frame/rgb/"+to_string(frame_id)+".png", im);
   cv::imwrite("./OurDataset_frame/depth2/"+to_string(frame_id)+".png", imD_colormap);

   return imD;
}


void System::densemap_frame_enqueue(cv::Mat imRGB, cv::Mat imD_float, cv::Mat K, cv::Mat Tcw, float bf, int frame_id, bool isKeyframe)
{
    OrbKeyFrame* newOrbKF = new OrbKeyFrame(imRGB, imD_float, K, Tcw, bf, frame_id, isKeyframe);
    frame_queue->enqueue(*newOrbKF);
}


void System::densemap_request_finish()
{
    while(frame_queue->size() != 0)
        usleep(5000);
    usleep(100000);
    mpDepthPredictor->request_finish();

    if(mpDenseMapViewer)
    {
        mpDenseMapViewer->request_finish();
        while(!mpDenseMapViewer->is_finished())
            usleep(5000);
    }
    if(mpDenseMapViewer)
        pangolin::BindToContext("Dense Map Viewer");



}








cv::Mat System::TrackMonocular(const cv::Mat &im, const double &timestamp)
{
    if(mSensor!=MONOCULAR)
    {
        cerr << "ERROR: you called TrackMonocular but input sensor was not set to Monocular." << endl;
        exit(-1);
    }

    // Check mode change
    {
        unique_lock<mutex> lock(mMutexMode);
        if(mbActivateLocalizationMode)
        {
            mpLocalMapper->RequestStop();

            // Wait until Local Mapping has effectively stopped
            while(!mpLocalMapper->isStopped())
            {
                usleep(1000);
            }

            mpTracker->InformOnlyTracking(true);
            mbActivateLocalizationMode = false;
        }
        if(mbDeactivateLocalizationMode)
        {
            mpTracker->InformOnlyTracking(false);
            mpLocalMapper->Release();
            mbDeactivateLocalizationMode = false;
        }
    }

    // Check reset
    {
    unique_lock<mutex> lock(mMutexReset);
    if(mbReset)
    {
        mpTracker->Reset();
        mbReset = false;
    }
    }

    cv::Mat resizedim;
    cv::resize(im, resizedim, cv::Size(), 0.2, 0.2);
    if(resizedim.rows != 96 || resizedim.cols != 128){
        std::cout << "irregular input shape : " << resizedim.size() << std::endl;
        exit(1);
    }
    mpDepthEstimation->SetImg(resizedim);
    cv::Mat Tcw = mpTracker->GrabImageMonocular(im,timestamp);

    unique_lock<mutex> lock2(mMutexState);
    mTrackingState = mpTracker->mState;
    mTrackedMapPoints = mpTracker->mCurrentFrame.mvpMapPoints;
    mTrackedKeyPointsUn = mpTracker->mCurrentFrame.mvKeysUn;

    return Tcw;
}

void System::ActivateLocalizationMode()
{
    unique_lock<mutex> lock(mMutexMode);
    mbActivateLocalizationMode = true;
}

void System::DeactivateLocalizationMode()
{
    unique_lock<mutex> lock(mMutexMode);
    mbDeactivateLocalizationMode = true;
}

bool System::MapChanged()
{
    static int n=0;
    int curn = mpMap->GetLastBigChangeIdx();
    if(n<curn)
    {
        n=curn;
        return true;
    }
    else
        return false;
}

void System::Reset()
{
    unique_lock<mutex> lock(mMutexReset);
    mbReset = true;
}

void System::Shutdown()
{
    mpLocalMapper->RequestFinish();
    mpLoopCloser->RequestFinish();
    mpDepthEstimation->RequestFinish();
    /*
    if(mpViewer)
    {
        mpViewer->RequestFinish();
        while(!mpViewer->isFinished())
            usleep(5000);
    }
    */

    // Wait until all thread have effectively stopped
    while(!mpLocalMapper->isFinished() || !mpLoopCloser->isFinished() || mpLoopCloser->isRunningGBA())
    {
        usleep(5000);
    }

    /*
    if(mpViewer)
        pangolin::BindToContext("ORB-SLAM2: Map Viewer");
    */
}

void System::SaveTrajectoryTUM(const string &filename)
{
    cout << endl << "Saving camera trajectory to " << filename << " ..." << endl;
    if(mSensor==MONOCULAR)
    {
        cerr << "ERROR: SaveTrajectoryTUM cannot be used for monocular." << endl;
        return;
    }

    vector<KeyFrame*> vpKFs = mpMap->GetAllKeyFrames();
    sort(vpKFs.begin(),vpKFs.end(),KeyFrame::lId);

    // Transform all keyframes so that the first keyframe is at the origin.
    // After a loop closure the first keyframe might not be at the origin.
    cv::Mat Two = vpKFs[0]->GetPoseInverse();

    ofstream f;
    f.open(filename.c_str());
    f << fixed;

    // Frame pose is stored relative to its reference keyframe (which is optimized by BA and pose graph).
    // We need to get first the keyframe pose and then concatenate the relative transformation.
    // Frames not localized (tracking failure) are not saved.

    // For each frame we have a reference keyframe (lRit), the timestamp (lT) and a flag
    // which is true when tracking failed (lbL).
    list<ORB_SLAM2::KeyFrame*>::iterator lRit = mpTracker->mlpReferences.begin();
    list<double>::iterator lT = mpTracker->mlFrameTimes.begin();
    list<bool>::iterator lbL = mpTracker->mlbLost.begin();
    for(list<cv::Mat>::iterator lit=mpTracker->mlRelativeFramePoses.begin(),
        lend=mpTracker->mlRelativeFramePoses.end();lit!=lend;lit++, lRit++, lT++, lbL++)
    {
        if(*lbL)
            continue;

        KeyFrame* pKF = *lRit;

        cv::Mat Trw = cv::Mat::eye(4,4,CV_32F);

        // If the reference keyframe was culled, traverse the spanning tree to get a suitable keyframe.
        while(pKF->isBad())
        {
            Trw = Trw*pKF->mTcp;
            pKF = pKF->GetParent();
        }

        Trw = Trw*pKF->GetPose()*Two;

        cv::Mat Tcw = (*lit)*Trw;
        cv::Mat Rwc = Tcw.rowRange(0,3).colRange(0,3).t();
        cv::Mat twc = -Rwc*Tcw.rowRange(0,3).col(3);

        vector<float> q = Converter::toQuaternion(Rwc);

        f << setprecision(6) << *lT << " " <<  setprecision(9) << twc.at<float>(0) << " " << twc.at<float>(1) << " " << twc.at<float>(2) << " " << q[0] << " " << q[1] << " " << q[2] << " " << q[3] << endl;
    }
    f.close();
    cout << endl << "trajectory saved!" << endl;
}


void System::SaveKeyFrameTrajectoryTUM(const string &filename)
{
    cout << endl << "Saving keyframe trajectory to " << filename << " ..." << endl;

    vector<KeyFrame*> vpKFs = mpMap->GetAllKeyFrames();
    sort(vpKFs.begin(),vpKFs.end(),KeyFrame::lId);

    // Transform all keyframes so that the first keyframe is at the origin.
    // After a loop closure the first keyframe might not be at the origin.
    //cv::Mat Two = vpKFs[0]->GetPoseInverse();

    ofstream f;
    f.open(filename.c_str());
    f << fixed;

    for(size_t i=0; i<vpKFs.size(); i++)
    {
        KeyFrame* pKF = vpKFs[i];

       // pKF->SetPose(pKF->GetPose()*Two);

        if(pKF->isBad())
            continue;

        cv::Mat R = pKF->GetRotation().t();
        vector<float> q = Converter::toQuaternion(R);
        cv::Mat t = pKF->GetCameraCenter();
        f << setprecision(6) << pKF->mTimeStamp << setprecision(7) << " " << t.at<float>(0) << " " << t.at<float>(1) << " " << t.at<float>(2)
          << " " << q[0] << " " << q[1] << " " << q[2] << " " << q[3] << endl;

    }

    f.close();
    cout << endl << "trajectory saved!" << endl;
}

void System::SaveTrajectoryKITTI(const string &filename)
{
    cout << endl << "Saving camera trajectory to " << filename << " ..." << endl;
    if(mSensor==MONOCULAR)
    {
        cerr << "ERROR: SaveTrajectoryKITTI cannot be used for monocular." << endl;
        return;
    }

    vector<KeyFrame*> vpKFs = mpMap->GetAllKeyFrames();
    sort(vpKFs.begin(),vpKFs.end(),KeyFrame::lId);

    // Transform all keyframes so that the first keyframe is at the origin.
    // After a loop closure the first keyframe might not be at the origin.
    cv::Mat Two = vpKFs[0]->GetPoseInverse();

    ofstream f;
    f.open(filename.c_str());
    f << fixed;

    // Frame pose is stored relative to its reference keyframe (which is optimized by BA and pose graph).
    // We need to get first the keyframe pose and then concatenate the relative transformation.
    // Frames not localized (tracking failure) are not saved.

    // For each frame we have a reference keyframe (lRit), the timestamp (lT) and a flag
    // which is true when tracking failed (lbL).
    list<ORB_SLAM2::KeyFrame*>::iterator lRit = mpTracker->mlpReferences.begin();
    list<double>::iterator lT = mpTracker->mlFrameTimes.begin();
    for(list<cv::Mat>::iterator lit=mpTracker->mlRelativeFramePoses.begin(), lend=mpTracker->mlRelativeFramePoses.end();lit!=lend;lit++, lRit++, lT++)
    {
        ORB_SLAM2::KeyFrame* pKF = *lRit;

        cv::Mat Trw = cv::Mat::eye(4,4,CV_32F);

        while(pKF->isBad())
        {
          //  cout << "bad parent" << endl;
            Trw = Trw*pKF->mTcp;
            pKF = pKF->GetParent();
        }

        Trw = Trw*pKF->GetPose()*Two;

        cv::Mat Tcw = (*lit)*Trw;
        cv::Mat Rwc = Tcw.rowRange(0,3).colRange(0,3).t();
        cv::Mat twc = -Rwc*Tcw.rowRange(0,3).col(3);

        f << setprecision(9) << Rwc.at<float>(0,0) << " " << Rwc.at<float>(0,1)  << " " << Rwc.at<float>(0,2) << " "  << twc.at<float>(0) << " " <<
             Rwc.at<float>(1,0) << " " << Rwc.at<float>(1,1)  << " " << Rwc.at<float>(1,2) << " "  << twc.at<float>(1) << " " <<
             Rwc.at<float>(2,0) << " " << Rwc.at<float>(2,1)  << " " << Rwc.at<float>(2,2) << " "  << twc.at<float>(2) << endl;
    }
    f.close();
    cout << endl << "trajectory saved!" << endl;
}

int System::GetTrackingState()
{
    unique_lock<mutex> lock(mMutexState);
    return mTrackingState;
}

vector<MapPoint*> System::GetTrackedMapPoints()
{
    unique_lock<mutex> lock(mMutexState);
    return mTrackedMapPoints;
}

vector<cv::KeyPoint> System::GetTrackedKeyPointsUn()
{
    unique_lock<mutex> lock(mMutexState);
    return mTrackedKeyPointsUn;
}

} //namespace ORB_SLAM
