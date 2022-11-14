
#include "Unity.h"

EXPORT int CreateSLAMSystem(const char* vocabularyPath, const char* settingsPath, int sensorType)
{
    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    _SLAM = new ORB_SLAM3::System
    (
        true,
        std::string (vocabularyPath),
        std::string (settingsPath),
        //"C:/Users/Egemen/Documents/GitHub/ORB_SLAM3_Windows/Vocabulary/ORBvoc.bin",
        //"C:/Users/Egemen/Documents/GitHub/ORB_SLAM3_Windows/Examples/Monocular-Inertial/TUM_512.yaml",
        (ORB_SLAM3::System::eSensor) sensorType
    );

    return 1;
}

EXPORT void ExecuteSLAM_File_Monocular(const char *imagePath, double timestamp, float** matData, int* rows, int* cols) {
    std::string imagePathStr(imagePath);

    if (_SLAM != nullptr) {
        cv::Mat cameraPose = _SLAM->Execute(imagePathStr, timestamp);
        if (cameraPose.rows > 0 && cameraPose.cols > 0) {
            cameraPose = cameraPose.inv();          // to get Twc from Tcw
            cameraPose = cameraPose.reshape(1, 1);  // flatten
        }
        *rows = cameraPose.rows;
        *cols = cameraPose.cols;

        memcpy(&matData[0], &cameraPose.data[0], cameraPose.rows * cameraPose.cols * 4.);
        //*matData = cameraPose.ptr<float>(0);
    }
    else {
        cerr << "SLAM system is not initialized." << endl;
    }
}
EXPORT void ExecuteSLAM_File_IMU_Monocular(const char* imagePath, double timestamp, ORB_SLAM3::IMU::Point *imuMeas, int imuMeasSize, float** matData, int* rows, int* cols) {

    if (_SLAM != nullptr) {
        std::string imagePathStr(imagePath);

        vector<ORB_SLAM3::IMU::Point> vImuMeas;
        ORB_SLAM3::IMU::Point* imuMeasPtr = imuMeas;

        for (int i = 0; i < imuMeasSize; i++, imuMeasPtr++) {
            vImuMeas.push_back(*imuMeasPtr);
        }

        cv::Mat cameraPose = _SLAM->Execute(imagePathStr, timestamp, vImuMeas);
        if (cameraPose.rows > 0 && cameraPose.cols > 0) {
            cameraPose = cameraPose.inv();          // to get Twc from Tcw
            cameraPose = cameraPose.reshape(1, 1);  // flatten
        }
        *rows = cameraPose.rows;
        *cols = cameraPose.cols;

        memcpy(&matData[0], &cameraPose.data[0], cameraPose.rows * cameraPose.cols * 4.);
        //*matData = cameraPose.ptr<float>(0);
        vImuMeas.clear();
        vector<ORB_SLAM3::IMU::Point>().swap(vImuMeas);
    }
    else {
        cerr << "SLAM system is not initialized." << endl;
    }
}

// TODO: Convert parameters to struct
EXPORT void ExecuteSLAM_IMU_Monocular(Color32 **imagePtr, double timestamp, ORB_SLAM3::IMU::Point *imuMeas, int imuMeasSize, int imageWidth, int imageHeight, float** matData, int* rows, int* cols) {
    
    if (_SLAM != nullptr) {
        cv::Mat image(imageHeight, imageWidth, CV_8UC4, *imagePtr);

        vector<ORB_SLAM3::IMU::Point> vImuMeas;
        ORB_SLAM3::IMU::Point* imuMeasPtr = imuMeas;

        for (int i = 0; i < imuMeasSize; i++, imuMeasPtr++) {
            vImuMeas.push_back(*imuMeasPtr);
        }
        cv::Mat cameraPose = _SLAM->Execute(image, timestamp, vImuMeas);
        if (cameraPose.rows > 0 && cameraPose.cols > 0) {
            cameraPose = cameraPose.inv();          // to get Twc from Tcw
            cameraPose = cameraPose.reshape(1, 1);  // flatten
        }
        *rows = cameraPose.rows;
        *cols = cameraPose.cols;

        memcpy(&matData[0], &cameraPose.data[0], cameraPose.rows * cameraPose.cols * 4.);
        //*matData = cameraPose.ptr<float>(0);
        vImuMeas.clear();
        vector<ORB_SLAM3::IMU::Point>().swap(vImuMeas);
    }
    else {
        cerr << "SLAM system is not initialized." << endl;
    }
}

EXPORT void ExecuteSLAM_Monocular(Color32** imagePtr, double timestamp, int imageWidth, int imageHeight, float** matData, int* rows, int* cols) {
    cv::Mat image(imageHeight, imageWidth, CV_8UC4, *imagePtr);

    if (_SLAM != nullptr) {
        cv::Mat cameraPose = _SLAM->Execute(image, timestamp);

        if (cameraPose.rows > 0 && cameraPose.cols > 0) {
            cameraPose = cameraPose.inv();          // to get Twc from Tcw
            cameraPose = cameraPose.reshape(1, 1);  // flatten
        }
        *rows = cameraPose.rows;
        *cols = cameraPose.cols;

        memcpy(&matData[0], &cameraPose.data[0], cameraPose.rows * cameraPose.cols * 4.);
        //*matData = cameraPose.ptr<float>(0);
    }
    else {
        cerr << "SLAM system is not initialized." << endl;
    }
}

EXPORT void PrepareForMapPoints(int* itemCount) {
    if (_SLAM == nullptr) {
        cerr << "SLAM system is not initialized." << endl;
        return;
    }

    //mapPoints->clear();
    vector<ORB_SLAM3::MapPoint*> vMPs = _SLAM->GetTrackedMapPoints();
    //vector<cv::Mat> worldPos;

    int i = 0;
    for (auto& element : vMPs) {
        if (element != nullptr) {
            //cv::Mat mapWorldPoint = element->GetWorldPos().clone();
            //cout << "M = " << endl << " " << mapWorldPoint << endl << endl;
            //mapPoints->push_back(&mapWorldPoint.data[0]);
            ++i;
        }
    }

    *itemCount = i;
}

EXPORT void GetMapPoints(ItemListHandle* hItems, double** itemsFound) {
    if (_SLAM == nullptr) {
        cerr << "SLAM system is not initialized." << endl;
        return;
    }
    
    //mapPoints->clear();
    vector<ORB_SLAM3::MapPoint*> vMPs = _SLAM->GetTrackedMapPoints();
    //vector<cv::Mat> worldPos;
    
    int offset = 0; 
    for (auto& element : vMPs) {
        if (element != nullptr) {
            cv::Mat mapWorldPoint = element->GetWorldPos();
            //cout << "M = " << endl << " " << mapWorldPoint << endl << endl;
            memcpy(&itemsFound[offset], &mapWorldPoint.data[0], mapWorldPoint.rows * mapWorldPoint.cols * sizeof(double));
            offset += mapWorldPoint.rows * mapWorldPoint.cols;
        }
    }
    //*hItems = reinterpret_cast<ItemListHandle>(mapPoints);
    //*itemsFound = (float*) mapPoints->data();
    //*itemCount = mapPoints->size();

}

EXPORT int GetTrackingState() {
    if (_SLAM == nullptr) {
        cerr << "SLAM system is not initialized." << endl;
        return ORB_SLAM3::Tracking::eTrackingState::SYSTEM_NOT_READY;
    }

    return _SLAM->GetTrackingState();
}

EXPORT void ShutdownSLAMSystem() {
    if (_SLAM != nullptr) {
        _SLAM->Shutdown();
    }
    else {
        cerr << "SLAM system is not initialized." << endl;
    }
}