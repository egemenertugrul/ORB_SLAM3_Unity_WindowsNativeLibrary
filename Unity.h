#ifndef UNITY_H
#define UNITY_H

#include "global.h"
#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>
#include <iomanip>
//#include <unistd.h>

#include <opencv2/core/core.hpp>

#include "System.h"
#include "Tracking.h"
#include "Converter.h"
//#include "ImuTypes.h"

struct Color32
{
    uchar red;
    uchar green;
    uchar blue;
    uchar alpha;
};

typedef intptr_t ItemListHandle;
//auto mapPoints = new std::vector<void*>();

static ORB_SLAM3::System* _SLAM = nullptr;

EXPORT int CreateSLAMSystem(const char* vocabularyPath, const char* settingsPath, int sensorType);

EXPORT void ExecuteSLAM_File_Monocular(const char* imagePath, double timestamp, float** matData, int* rows, int* cols);

EXPORT void ExecuteSLAM_File_IMU_Monocular(const char* imagePath, double timestamp, ORB_SLAM3::IMU::Point *imuMeas, int imuMeasSize, float** matData, int* rows, int* cols);

EXPORT void ExecuteSLAM_IMU_Monocular(Color32** imagePtr, double timestamp, ORB_SLAM3::IMU::Point *imuMeas, int imuMeasSize, int imageWidth, int imageHeight, float** matData, int* rows, int* cols);

EXPORT void ExecuteSLAM_Monocular(Color32** imagePtr, double timestamp, int imageWidth, int imageHeight, float** matData, int* rows, int* cols);

EXPORT void PrepareForMapPoints(int* itemCount);

EXPORT void GetMapPoints(ItemListHandle* hItems, double** itemsFound);

EXPORT int GetTrackingState();

EXPORT void ShutdownSLAMSystem();

#endif