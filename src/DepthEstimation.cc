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
#include "DepthEstimation.h"
#include <stdio.h>
#include <mutex>
#include <thread>
#include <locale.h>
#include <string>
#include <time.h>
#include<unistd.h>

namespace ORB_SLAM2
{

DepthEstimation::DepthEstimation(const string &strSettingsFile):
        mbResetRequested(false), mbFinishRequested(false), mbFinished(true)
{
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
    fsSettings.release();

    // set python network settings
    _isSet = false;
    _isGet = false;
    initialise();
    std::cout << "now running Dense Depth for keyframe" << std::endl;
}


void DepthEstimation::SetSystem(System* pSystem)
{
    mpSystem = pSystem;
}

void DepthEstimation::SetTracker(Tracking* pTracker)
{
    mpTracker = pTracker;
}



void DepthEstimation::Run()
{
    _isSet = false;
    _isGet = false;
    if(!_isReady)
        exit(-1);
    while(1)
    {
        if(CheckNewFrames())
        {
            std::cout << "get new Frame : " << mlNewFrames.size() << std::endl;
            {
                unique_lock<mutex> lock(mMutexNewFrames);
                mpCurrentFrame = mlNewFrames.front();
                mlNewFrames.pop_front();
            }

            cv::Mat resizedim;
            cv::Mat imD_float;
            cv::resize(mpCurrentFrame->imRGB, resizedim, cv::Size(m_net_width, m_net_height), m_net_width/m_img_width, m_net_height/m_img_height);
            if(resizedim.rows % 32 != 0 || resizedim.cols % 32 != 0){
                std::cout << "irregular input shape : " << resizedim.size() << std::endl;
                exit(1);
            }

            _isSet = true;
            _isGet = false;
            std::cout << "input : " << resizedim.size() << std::endl;

            cv::Mat imD = execute(resizedim);
            std::cout << "pred done" << std::endl;
            _isSet = false;
            _isGet = true;
            if (mpCurrentFrame->imRGB.size() != imD.size()) {
                std::cerr << "didn't match size inputImg and resultDepth : im = " << mpCurrentFrame->imRGB.size() << " depth = " << imD.size()
                          << std::endl;
                exit(-1);
            }
            cv::resize(imD, imD, cv::Size(m_img_width, m_img_height), 0,0, cv::INTER_NEAREST);
            imD.convertTo(imD_float, CV_32F);
            //// Pass the images to the SLAM system
            if(mpTracker->mState == 2)
            {
                std::cout << "enqueue densemap" << std::endl;
                mpSystem->densemap_frame_enqueue(mpCurrentFrame->imRGB, imD, K, mpCurrentFrame->mTcw, bf, mpCurrentFrame->mnId,  mpCurrentFrame->isKeyframe);
            }
            //mpCurrentFrame->imRGB.release();

        }
        ResetIfRequested();
        if(CheckFinish())
            break;
        usleep(3000);
    }
    SetFinish();
}


void DepthEstimation::InsertFrame(Frame *pF)
{
    unique_lock<mutex> lock(mMutexNewFrames);
    mlNewFrames.push_back(pF);
}

bool DepthEstimation::CheckNewFrames()
{
    unique_lock<mutex> lock(mMutexNewFrames);
    return(!mlNewFrames.empty());
}


void DepthEstimation::_usleep()
{
    //std::cout << "now sleeping" << std::endl;
    usleep(5000);
}

void DepthEstimation::SetImg(cv::Mat img)
{
    /*
    if(!_isSet){
        inputImg = img;
        _isSet = true;
    }
     */
}

cv::Mat DepthEstimation::predict(cv::Mat img)
{
    if(mbFinishRequested){
        SetFinish();
        std::cout << "DepthEstimation class is shutdown..." << std::endl;
    }else {
        _isSet = true;
        _isGet = false;
        cv::Mat imD = execute(img);
        _isSet = false;
        _isGet = true;
        if (img.size() != imD.size()) {
            std::cerr << "didn't match size inputImg and resultDepth : im = " << img.size() << " depth = " << imD.size()
                      << std::endl;
            exit(-1);
        }
        return imD;
    }
}


void DepthEstimation::initialise()
{
    Py_SetProgramName((wchar_t*)L"test");
	Py_Initialize();
	wchar_t const * argv2[] = { L"test.py" };
	PySys_SetArgv(1, const_cast<wchar_t**>(argv2));
    // Load module
    loadModule();
	// Get function
	pExecute = PyObject_GetAttrString(pModule, "execute");
	if (pExecute == NULL || !PyCallable_Check(pExecute)) {
		if (PyErr_Occurred()) {
			std::cout << "Python error indicator is set:" << std::endl;
			PyErr_Print();
		}
		throw std::runtime_error("Could not load function 'execute' from DenseDepth module.");
	}
	std::cout << "* Initialised Done" << std::endl;
	_isReady = true;
}

void* DepthEstimation::loadModule()
{
    std::cout << " * Loading module..." << std::endl;
    pModule = PyImport_ImportModule("test");
    if (pModule == NULL) {
        if (PyErr_Occurred()) {
            std::cout << "Python error indicator is set:" << std::endl;
            PyErr_Print();
        }
        throw std::runtime_error("Could not open DenseDepth module.");
    }
    import_array();
    return 0;
}

cv::Mat DepthEstimation::extractImage()
{
    PyObject* pImage = getPyObject("result");
    PyArrayObject *pImageArray = (PyArrayObject*)(pImage);
    float* pData = (float*)PyArray_GETPTR1(pImageArray, 0);
    npy_intp h = PyArray_DIM(pImageArray, 0);
    npy_intp w = PyArray_DIM(pImageArray, 1);
    cv::Mat result;
    cv::Mat(h, w, CV_32FC1, pData).copyTo(result);//CV_8UC1
    Py_DECREF(pImage);
    return result;
}


PyObject *DepthEstimation::getPyObject(const char* name){
    PyObject* obj = PyObject_GetAttrString(pModule, name);
    if(!obj) throw std::runtime_error(std::string("Failed to get python object (there are no objects): ") + name);
    else if(obj == Py_None) throw std::runtime_error(std::string("Failed to get python object (Py_None): ") + name);
    return obj;
}

cv::Mat DepthEstimation::execute(cv::Mat inputImg)
{
    Py_XDECREF(PyObject_CallFunctionObjArgs(pExecute, createArguments(inputImg), NULL));
    cv::Mat resultImg = extractImage();
    _isSet = false;
    _isGet = true;
    return resultImg;
}

bool DepthEstimation::isGet()
{
    return _isGet;
}

bool DepthEstimation::isReady()
{
    return _isReady;
}

PyObject *DepthEstimation::createArguments(cv::Mat rgbImage) {
    assert(rgbImage.channels() == 3);
    npy_intp dims[3] = { rgbImage.rows, rgbImage.cols, 3 };
    return PyArray_SimpleNewFromData(3, dims, NPY_UINT8, rgbImage.data); // TODO Release?
}





void DepthEstimation::RequestReset()
{
    {
        unique_lock<mutex> lock(mMutexReset);
        mbResetRequested = true;
    }

    while(1)
    {
        {
            unique_lock<mutex> lock2(mMutexReset);
            if(!mbResetRequested)
                break;
        }
        usleep(5000);
    }
}

void DepthEstimation::ResetIfRequested()
{
    unique_lock<mutex> lock(mMutexReset);
    if(mbResetRequested)
    {
        _isSet = false;
        _isGet = false;
        mbResetRequested=false;
    }
}



void DepthEstimation::RequestFinish()
{
    unique_lock<mutex> lock(mMutexFinish);
    mbFinishRequested = true;
}

bool DepthEstimation::CheckFinish()
{
    unique_lock<mutex> lock(mMutexFinish);
    return mbFinishRequested;
}

void DepthEstimation::SetFinish()
{
    unique_lock<mutex> lock(mMutexFinish);
    mbFinished = true;
}

bool DepthEstimation::isFinished()
{
    unique_lock<mutex> lock(mMutexFinish);
    return mbFinished;
}

} //namespace ORB_SLAM
