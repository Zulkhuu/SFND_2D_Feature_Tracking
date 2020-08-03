/* INCLUDES FOR THIS PROJECT */
#include <iostream>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <numeric>
#include <vector>
#include <cmath>
#include <limits>
#include <opencv2/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/xfeatures2d.hpp>
#include <opencv2/xfeatures2d/nonfree.hpp>

#include "dataStructures.h"
#include "matching2D.hpp"

using namespace std;

struct perfStat
{
    string detectorType;
    string descriptorType;
    string distanceType;
    int n_keypoints;
    int n_matches;
    double t_desc;
    double t_det;
    perfStat(string detector, string descriptor, string distance) : 
        detectorType(detector), 
        descriptorType(descriptor),
        distanceType(distance),
        n_keypoints(0),
        n_matches(0),
        t_desc(0.0),
        t_det(0.0)
        {}
    void print(){
        cout << detectorType << ", " << descriptorType << ", " << distanceType << ", "
               << n_keypoints << ", " << n_matches << ", " << 1000*t_det << ", " << 1000*t_desc << endl;
    }
    string getString(){
        std::stringstream ss;
        ss << detectorType << ", " << descriptorType << ", " << distanceType << ", "
               << n_keypoints << ", " << n_matches << ", " << 100*t_det << ", " << 100*t_desc << ", " << 100*(t_det + t_desc) << endl;
        return ss.str();
    }
};

/* MAIN PROGRAM */
int main(int argc, const char *argv[])
{

    /* INIT VARIABLES AND DATA STRUCTURES */

    // data location
    string dataPath = "../";

    // camera
    string imgBasePath = dataPath + "images/";
    string imgPrefix = "KITTI/2011_09_26/image_00/data/000000"; // left camera, color
    string imgFileType = ".png";
    int imgStartIndex = 0; // first file index to load (assumes Lidar and camera names have identical naming convention)
    int imgEndIndex = 9;   // last file index to load
    int imgFillWidth = 4;  // no. of digits which make up the file index (e.g. img-0001.png)

    // misc
    int dataBufferSize = 2;       // no. of images which are held in memory (ring buffer) at the same time
    vector<DataFrame> dataBuffer; // list of data frames which are held in memory at the same time
    bool bVis = false;            // visualize results

    /* MAIN LOOP OVER ALL IMAGES */
    vector<perfStat> performances{};
    
    /*
    vector<string> detectorsTypes{"SHITOMASI", "HARRIS", "FAST", "BRISK", "ORB", "AKAZE", "SIFT"};
    vector<string> descriptorTypes{"BRISK", "BRIEF", "ORB", "FREAK", "AKAZE", "SIFT"};
    vector<string> distanceTypes{"DES_BINARY", "DES_HOG"};
    */
    
    
    vector<string> detectorsTypes{"SHITOMASI"};
    vector<string> descriptorTypes{"BRISK"};
    vector<string> distanceTypes{"DES_BINARY"};
    

    //string detectorType = "BRISK"; // HARRIS, FAST, BRISK, ORB, AKAZE, SIFT
    //string descriptorType = "ORB"; // BRIEF, ORB, FREAK, AKAZE, SIFT
    //string distanceType = "DES_HOG";  // DES_BINARY, DES_HOG

    for(const string& detectorType : detectorsTypes) {
        for(const string& descriptorType : descriptorTypes) {
            for(const string& distanceType : distanceTypes) {

                perfStat perf = perfStat(detectorType, descriptorType, distanceType);
                for (size_t imgIndex = 0; imgIndex <= imgEndIndex - imgStartIndex; imgIndex++)
                {
                    /* LOAD IMAGE INTO BUFFER */

                    // assemble filenames for current index
                    ostringstream imgNumber;
                    imgNumber << setfill('0') << setw(imgFillWidth) << imgStartIndex + imgIndex;
                    string imgFullFilename = imgBasePath + imgPrefix + imgNumber.str() + imgFileType;

                    // load image from file and convert to grayscale
                    cv::Mat img, imgGray;
                    img = cv::imread(imgFullFilename);
                    cv::cvtColor(img, imgGray, cv::COLOR_BGR2GRAY);

                    //// STUDENT ASSIGNMENT
                    //// TASK MP.1 -> replace the following code with ring buffer of size dataBufferSize

                    // push image into data frame buffer
                    DataFrame frame;
                    frame.cameraImg = imgGray;
                    dataBuffer.push_back(frame);
                    if(dataBuffer.size() > dataBufferSize)
                        dataBuffer.erase(dataBuffer.begin());

                    //// EOF STUDENT ASSIGNMENT
                    cout << "#1 : LOAD IMAGE INTO BUFFER done" << endl;

                    /* DETECT IMAGE KEYPOINTS */

                    // extract 2D keypoints from current image
                    vector<cv::KeyPoint> keypoints; // create empty feature list for current image

                    //// STUDENT ASSIGNMENT
                    //// TASK MP.2 -> add the following keypoint detectors in file matching2D.cpp and enable string-based selection based on detectorType
                    //// -> HARRIS, FAST, BRISK, ORB, AKAZE, SIFT

                    try {
                        double t_det = (double)cv::getTickCount();
                        if (detectorType.compare("SHITOMASI") == 0) {
                            detKeypointsShiTomasi(keypoints, imgGray, false);
                        }
                        else if (detectorType.compare("HARRIS") == 0) {
                            detKeypointsHarris(keypoints, imgGray, false);
                        }
                        else {
                            detKeypointsModern(keypoints, imgGray, detectorType, false);
                        }

                        t_det = ((double)cv::getTickCount() - t_det) / cv::getTickFrequency();
                        perf.t_det += t_det;
                    } catch (cv::Exception & ex) {
                        cout << detectorType << " feature detector resulted in exception " << endl;
                        continue;
                    }
                    //// EOF STUDENT ASSIGNMENT

                    //// STUDENT ASSIGNMENT
                    //// TASK MP.3 -> only keep keypoints on the preceding vehicle

                    // only keep keypoints on the preceding vehicle
                    bool bFocusOnVehicle = true;
                    cv::Rect vehicleRect(535, 180, 180, 150);
                    if (bFocusOnVehicle)
                    {
                        vector<cv::KeyPoint> tmp;
                        for (const cv::KeyPoint& keypoint : keypoints) {
                            if (vehicleRect.contains(keypoint.pt)) {
                                tmp.push_back(keypoint);
                            }
                        }
                        keypoints = tmp;
                        perf.n_keypoints += keypoints.size();
                        cout << "Number of keypoints on the preceding vehicle: " << keypoints.size() << endl;
                    }

                    //// EOF STUDENT ASSIGNMENT

                    // optional : limit number of keypoints (helpful for debugging and learning)
                    bool bLimitKpts = false;
                    if (bLimitKpts)
                    {
                        int maxKeypoints = 50;

                        if (detectorType.compare("SHITOMASI") == 0)
                        { // there is no response info, so keep the first 50 as they are sorted in descending quality order
                            keypoints.erase(keypoints.begin() + maxKeypoints, keypoints.end());
                        }
                        cv::KeyPointsFilter::retainBest(keypoints, maxKeypoints);
                        cout << " NOTE: Keypoints have been limited!" << endl;
                    }

                    // push keypoints and descriptor for current frame to end of data buffer
                    (dataBuffer.end() - 1)->keypoints = keypoints;
                    cout << "#2 : DETECT KEYPOINTS done" << endl;

                    /* EXTRACT KEYPOINT DESCRIPTORS */

                    //// STUDENT ASSIGNMENT
                    //// TASK MP.4 -> add the following descriptors in file matching2D.cpp and enable string-based selection based on descriptorType
                    //// -> BRIEF, ORB, FREAK, AKAZE, SIFT

                    cv::Mat descriptors;
                    try {
                        double t_desc = (double)cv::getTickCount();
                        descKeypoints((dataBuffer.end() - 1)->keypoints, (dataBuffer.end() - 1)->cameraImg, descriptors, descriptorType);

                        t_desc = ((double)cv::getTickCount() - t_desc) / cv::getTickFrequency();
                        perf.t_desc += t_desc;
                    }
                    catch (cv::Exception & ex) {
                        cout << detectorType << " feature detector combined with " << descriptorType << " resulted in exception during finding descriptor" << endl;
                        continue;
                    }
                    
                    //// EOF STUDENT ASSIGNMENT

                    // push descriptors for current frame to end of data buffer
                    (dataBuffer.end() - 1)->descriptors = descriptors;

                    cout << "#3 : EXTRACT DESCRIPTORS done" << endl;

                    if (dataBuffer.size() > 1) // wait until at least two images have been processed
                    {

                        /* MATCH KEYPOINT DESCRIPTORS */

                        vector<cv::DMatch> matches;
                        string matcherType = "MAT_BF";        // MAT_BF, MAT_FLANN
                        string selectorType = "SEL_KNN";       // SEL_NN, SEL_KNN

                        //// STUDENT ASSIGNMENT
                        //// TASK MP.5 -> add FLANN matching in file matching2D.cpp
                        //// TASK MP.6 -> add KNN match selection and perform descriptor distance ratio filtering with t=0.8 in file matching2D.cpp

                        try {
                            matchDescriptors((dataBuffer.end() - 2)->keypoints, (dataBuffer.end() - 1)->keypoints,
                                            (dataBuffer.end() - 2)->descriptors, (dataBuffer.end() - 1)->descriptors,
                                            matches, distanceType, matcherType, selectorType);
                            perf.n_matches += matches.size();
                        } 
                        catch (cv::Exception & ex) {
                            cout << detectorType << " feature detector combined with " << descriptorType << " resulted in exception during matching" << endl;
                            continue;
                        }

                        //// EOF STUDENT ASSIGNMENT

                        // store matches in current data frame
                        (dataBuffer.end() - 1)->kptMatches = matches;

                        cout << "#4 : MATCH KEYPOINT DESCRIPTORS done" << endl;

                        // visualize matches between current and previous image
                        bVis = true;
                        if (bVis)
                        {
                            cv::Mat matchImg = ((dataBuffer.end() - 1)->cameraImg).clone();
                            cv::drawMatches((dataBuffer.end() - 2)->cameraImg, (dataBuffer.end() - 2)->keypoints,
                                            (dataBuffer.end() - 1)->cameraImg, (dataBuffer.end() - 1)->keypoints,
                                            matches, matchImg,
                                            cv::Scalar::all(-1), cv::Scalar::all(-1),
                                            vector<char>(), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);

                            string windowName = "Matching keypoints between two camera images";
                            cv::namedWindow(windowName, 7);
                            cv::imshow(windowName, matchImg);
                            cout << "Press key to continue to next image" << endl;
                            cv::waitKey(0); // wait for key to be pressed
                        }
                        bVis = false;
                    }

                } // eof loop over all images

                performances.push_back(perf);
            } //eof loop over all distances
        } //eof loop over all descriptors
    } //eof loop over all detectors

    bool save_result = false;
    if(save_result) {
        // save performance to performance.csv
        std::ofstream result_file;
        result_file.open ("../performance.csv");
        // Column names
        result_file << "Detector, Descriptor, Distance, Number of keypoints, Number of matches, Det time, Desc time, Total time\n";
        // Result
        for(auto performance : performances) {
            result_file << performance.getString();
        }
    }
    return 0;
}
