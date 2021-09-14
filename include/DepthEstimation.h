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
#include "Frame.h"
#include "LocalMapping.h"
#include "Map.h"
#include "ORBVocabulary.h"
#include "Tracking.h"
#include "KeyFrameDatabase.h"
#include <thread>
#include <mutex>
#include "Thirdparty/g2o/g2o/types/types_seven_dof_expmap.h"
#include <opencv2/opencv.hpp>
#include <Python.h>
#define NPY_NO_DEPRECATED_API NPY_1_7_API_VERSION
#include <numpy/ndarraytypes.h>
#include <numpy/arrayobject.h>
#include <thread>
#include "System.h"
#include <unistd.h>

namespace ORB_SLAM2
{

class System;
class Tracking;
class LocalMapping;
class KeyFrameDatabase;

class DepthEstimation
{
public:

    DepthEstimation(const string &strSettingsFile);
    void SetSystem(System* pSystem);
    void SetTracker(Tracking* pTracker);
    void Run();//main function

    void SetImg(cv::Mat img);
    bool isGet();
    bool isReady();

    void RequestFinish();
    bool isFinished();
    void RequestReset();

    cv::Mat predict(cv::Mat img);
    void InsertFrame(Frame *pF);
    bool CheckNewFrames();







private:

    void _usleep();
    PyObject* createArguments(cv::Mat rgbImage);
	void* loadModule();
	void initialise();
	inline PyObject* getPyObject(const char* name);
	cv::Mat extractImage();
	cv::Mat execute(cv::Mat img);

	PyObject *pModule;
	PyObject *pExecute;

    bool _isSet = false;
    bool _isGet = false;
    bool _isReady = false;

    // ORB setting info (.yaml)
    cv::Mat K;
    float bf;
    int m_img_width;
    int m_img_height;
    int m_net_width;
    int m_net_height;
    std::list<Frame*> mlNewFrames;
    std::mutex mMutexNewFrames;
    Frame* mpCurrentFrame;

protected:
    System* mpSystem;
    Tracking* mpTracker;
    void ResetIfRequested();
    bool mbResetRequested;
    std::mutex mMutexReset;

    bool CheckFinish();
    void SetFinish();
    bool mbFinishRequested;
    bool mbFinished;
    std::mutex mMutexFinish;





};//end of class DepthEstimation


} //namespace ORB_SLAM

