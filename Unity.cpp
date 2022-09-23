
#include "Unity.h"

EXPORT int CreateSLAMSystem(const char* vocabularyPath, const char* settingsPath)
{
    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    _SLAM = new ORB_SLAM3::System
    (
        true,
        std::string (vocabularyPath),
        std::string (settingsPath),
        //"C:/Users/Egemen/Documents/GitHub/ORB_SLAM3_Windows/Vocabulary/ORBvoc.bin",
        //"C:/Users/Egemen/Documents/GitHub/ORB_SLAM3_Windows/Examples/Monocular-Inertial/TUM_512.yaml",
        ORB_SLAM3::System::MONOCULAR
    );

    return 1;
}

EXPORT void ExecuteSLAM_Filepath(const char *imagePath, double timestamp, float** matData, int* rows, int* cols) {
    std::string imagePathStr(imagePath);
    // TODO: fix leak

    if (_SLAM != NULL) {
        cv::Mat cameraPose = _SLAM->Execute(imagePathStr, timestamp);
        if (cameraPose.rows > 0 && cameraPose.cols > 0) {
            cameraPose = cameraPose.inv();
            cameraPose = cameraPose.reshape(1, 1);
        }

        *matData = cameraPose.ptr<float>(0);
        *rows = cameraPose.rows;
        *cols = cameraPose.cols;
    }
    else {
        cerr << "SLAM system is not initialized." << endl;
    }
}

// TODO: Convert parameters to struct
EXPORT void ExecuteSLAM_Image(Color32 **imagePtr, double timestamp, int imageWidth, int imageHeight, float** matData, int* rows, int* cols) {
    cv::Mat image(imageHeight, imageWidth, CV_8UC4, *imagePtr);
    
    if (_SLAM != NULL) {
        cv::Mat cameraPose = _SLAM->Execute(image, timestamp);
        if (cameraPose.rows > 0 && cameraPose.cols > 0) {
            cameraPose = cameraPose.inv();          // to get Twc from Tcw
            cameraPose = cameraPose.reshape(1, 1);  // flatten
        }

        *matData = cameraPose.ptr<float>(0);
        *rows = cameraPose.rows;
        *cols = cameraPose.cols;
    }
    else {
        cerr << "SLAM system is not initialized." << endl;
    }
}

EXPORT void ShutdownSLAMSystem() {
    if (_SLAM != NULL) {
        _SLAM->Shutdown();
    }
    else {
        cerr << "SLAM system is not initialized." << endl;
    }
}