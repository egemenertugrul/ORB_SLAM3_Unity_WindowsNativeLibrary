#include "global.h"
#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>
#include <iomanip>
//#include <unistd.h>

#include <opencv2/core/core.hpp>

#include "System.h"
#include "Converter.h"

struct Color32
{
    uchar red;
    uchar green;
    uchar blue;
    uchar alpha;
};

static ORB_SLAM3::System* _SLAM;

EXPORT int CreateSLAMSystem(const char* vocabularyPath, const char* settingsPath);

EXPORT void ExecuteSLAM_Filepath(const char* imagePath, double timestamp, float** matData, int* rows, int* cols);

EXPORT void ExecuteSLAM_Image(Color32** imagePtr, double timestamp, int imageWidth, int imageHeight, float** matData, int* rows, int* cols);

EXPORT void ShutdownSLAMSystem();
