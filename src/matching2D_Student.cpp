#include <numeric>
#include "matching2D.hpp"

using namespace std;

// Find best matches for keypoints in two camera images based on several matching methods
void matchDescriptors(std::vector<cv::KeyPoint> &kPtsSource, std::vector<cv::KeyPoint> &kPtsRef, cv::Mat &descSource, cv::Mat &descRef,
                      std::vector<cv::DMatch> &matches, std::string descriptorType, std::string matcherType, std::string selectorType)
{
    // configure matcher
    bool crossCheck = false;
    cv::Ptr<cv::DescriptorMatcher> matcher;

    // create the matcher
    if (matcherType.compare("MAT_BF") == 0)
    {
        int normType = (descriptorType.compare("DES_BINARY") == 0) ? cv::NORM_HAMMING : cv::NORM_L2;
        matcher = cv::BFMatcher::create(normType, crossCheck);
    }
    else if (matcherType.compare("MAT_FLANN") == 0)
    {

        if(descriptorType.compare("DES_BINARY") == 0) {
            // See it here: https://answers.opencv.org/question/59996/flann-error-in-opencv-3/
            matcher = cv::makePtr<cv::FlannBasedMatcher>(cv::makePtr<cv::flann::LshIndexParams>(12, 20, 2));
        }
        else {
            matcher = cv::DescriptorMatcher::create(cv::DescriptorMatcher::FLANNBASED);
        }

    }

    // perform matching task
    if (selectorType.compare("SEL_NN") == 0)
    { // nearest neighbor (best match)

        matcher->match(descSource, descRef, matches); // Finds the best match for each descriptor in desc1
    }
    else if (selectorType.compare("SEL_KNN") == 0)
    { // k nearest neighbors (k=2)

        std::vector< std::vector<cv::DMatch> > knn_matches;
        matcher->knnMatch(descSource,descRef,knn_matches,2);

        const float threshold = 0.8;
        for (int i = 0; i < knn_matches.size(); i++)
        {
            if(knn_matches[i].size() != 2) {
                continue;
            }

            if (knn_matches[i][0].distance < threshold * knn_matches[i][1].distance)
            {
                matches.push_back(knn_matches[i][0]);
            }
        }
    }
}

// Use one of several types of state-of-art descriptors to uniquely identify keypoints
void descKeypoints(vector<cv::KeyPoint> &keypoints, cv::Mat &img, cv::Mat &descriptors, string descriptorType)
{
    // select appropriate descriptor
    cv::Ptr<cv::DescriptorExtractor> extractor;
    if (descriptorType.compare("BRISK") == 0)
    {

        int threshold = 30;        // FAST/AGAST detection threshold score.
        int octaves = 3;           // detection octaves (use 0 to do single scale)
        float patternScale = 1.0f; // apply this scale to the pattern used for sampling the neighbourhood of a keypoint.

        extractor = cv::BRISK::create(threshold, octaves, patternScale);
    }
    else if (descriptorType.compare("BRIEF") == 0)
    {
        extractor = cv::xfeatures2d::BriefDescriptorExtractor::create();
    }
    else if (descriptorType.compare("ORB") == 0)
    {
        extractor = cv::ORB::create();
    }
    else if (descriptorType.compare("FREAK") == 0)
    {
        extractor = cv::xfeatures2d::FREAK::create();
    }
    else if (descriptorType.compare("AKAZE") == 0)
    {
        extractor = cv::AKAZE::create();
    }
    else if (descriptorType.compare("SIFT") == 0)
    {
        extractor = cv::xfeatures2d::SIFT::create();
    }

    extractor->compute(img, keypoints, descriptors);

}

// Detect keypoints in image using the traditional Shi-Thomasi detector
void detKeypointsShiTomasi(vector<cv::KeyPoint> &keypoints, cv::Mat &img, bool bVis)
{
    // compute detector parameters based on image size
    int blockSize = 4;       //  size of an average block for computing a derivative covariation matrix over each pixel neighborhood
    double maxOverlap = 0.0; // max. permissible overlap between two features in %
    double minDistance = (1.0 - maxOverlap) * blockSize;
    int maxCorners = img.rows * img.cols / max(1.0, minDistance); // max. num. of keypoints

    double qualityLevel = 0.01; // minimal accepted quality of image corners
    double k = 0.04;

    // Apply corner detection
    vector<cv::Point2f> corners;
    cv::goodFeaturesToTrack(img, corners, maxCorners, qualityLevel, minDistance, cv::Mat(), blockSize, false, k);

    // add corners to result vector
    for (auto it = corners.begin(); it != corners.end(); ++it)
    {

        cv::KeyPoint newKeyPoint;
        newKeyPoint.pt = cv::Point2f((*it).x, (*it).y);
        newKeyPoint.size = blockSize;
        keypoints.push_back(newKeyPoint);
    }

    // visualize results
    if (bVis)
    {
        visualize(keypoints, img);
    }
}

void detKeypointsHarris(std::vector<cv::KeyPoint> &keypoints, cv::Mat &img, bool bVis) {

    // Detector parameters
    int blockSize = 2;     // for every pixel, a blockSize × blockSize neighborhood is considered
    int apertureSize = 3;  // aperture parameter for Sobel operator (must be odd)
    int minResponse = 100; // minimum value for a corner in the 8bit scaled response matrix
    double k = 0.04;       // Harris parameter (see equation for details)

    // Detect Harris corners and normalize output
    cv::Mat dst, dst_norm, dst_norm_scaled;
    dst = cv::Mat::zeros(img.size(), CV_32FC1);
    cv::cornerHarris(img, dst, blockSize, apertureSize, k, cv::BORDER_DEFAULT);
    cv::normalize(dst, dst_norm, 0, 255, cv::NORM_MINMAX, CV_32FC1, cv::Mat());
    cv::convertScaleAbs(dst_norm, dst_norm_scaled);

    float maxOverlap = 0.1;
    float overlap;
    bool isOverlapping = false;

    // Note: all pixels in dst_norm are candidates
    for(int r=0; r<dst_norm.rows; ++r) {

        for(int c=0; c<dst_norm.cols; ++c) {

            // Check if the keypoint candidate is above the threshold
            int response = (int) dst_norm.at<float>(r,c);
            if( response > minResponse) {

                isOverlapping = false;

                // Create the new keypoint
                cv::KeyPoint newKeyPoint = cv::KeyPoint((float) c, (float) r, 2.0*apertureSize, response);

                // Check if the keypoint is overlapping
                for(auto it=keypoints.begin(); it!=keypoints.end(); ++it) {

                    overlap = cv::KeyPoint::overlap(newKeyPoint,*it);
                    if(overlap > maxOverlap) {
                        isOverlapping= true;

                        // Replace the keypoint in the list if the current one has a higher response
                        if(newKeyPoint.response > it->response) {
                            *it = newKeyPoint;
                        }

                        break;
                    }
                }

                // If the keypoint is not overlapping, add it to the vector of keypoints
                if(!isOverlapping) {
                    keypoints.push_back(newKeyPoint);
                }

            }
        }
    }

    // visualize results
    if (bVis)
    {
        visualize(keypoints, img);
    }


}

void detKeypointsModern(std::vector<cv::KeyPoint> &keypoints, cv::Mat &img, std::string detectorType, bool bVis) {

    // All detectors are created with their default parameters
    if (detectorType.compare("FAST") == 0) {
        cv::Ptr<cv::FeatureDetector> detector = cv::FastFeatureDetector::create();
        detector->detect(img, keypoints);
    }
    else if(detectorType.compare("BRISK") == 0) {
        cv::Ptr<cv::FeatureDetector> detector = cv::BRISK::create();
        detector->detect(img, keypoints);
    }
    else if(detectorType.compare("ORB") == 0) {
        cv::Ptr<cv::FeatureDetector> detector = cv::ORB::create();
        detector->detect(img, keypoints);
    }
    else if(detectorType.compare("AKAZE") == 0) {
        cv::Ptr<cv::FeatureDetector> detector = cv::AKAZE::create();
        detector->detect(img, keypoints);
    }
    else if(detectorType.compare("SIFT") == 0) {
        cv::Ptr<cv::FeatureDetector> detector = cv::xfeatures2d::SiftFeatureDetector::create();
        detector->detect(img, keypoints);
    }

    // visualize results
    if (bVis)
    {
        visualize(keypoints, img);
    }
}

void visualize(std::vector<cv::KeyPoint> &keypoints, cv::Mat &img) {
    cv::Mat visImage = img.clone();
    cv::drawKeypoints(img, keypoints, visImage, cv::Scalar::all(-1), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
    string windowName = "Detector Results";
    cv::namedWindow(windowName, 6);
    imshow(windowName, visImage);
    cv::waitKey(0);
}

void detKeypoints(std::vector<cv::KeyPoint> &keypoints, cv::Mat &imgGray, std::string detectorType, bool bVis) {
    if (detectorType.compare("SHITOMASI") == 0)
    {
        detKeypointsShiTomasi(keypoints, imgGray, bVis);
    }
    else if (detectorType.compare("HARRIS") == 0)
    {
        detKeypointsHarris(keypoints,imgGray,bVis);
    }
    else if (detectorType.compare("FAST") == 0)
    {
        detKeypointsModern(keypoints, imgGray, "FAST", bVis);
    }
    else if (detectorType.compare("BRISK") == 0)
    {
        detKeypointsModern(keypoints, imgGray, "BRISK", bVis);
    }
    else if (detectorType.compare("ORB") == 0)
    {
        detKeypointsModern(keypoints, imgGray, "ORB", bVis);
    }
    else if (detectorType.compare("AKAZE") == 0)
    {
        detKeypointsModern(keypoints, imgGray, "AKAZE", bVis);
    }
    else if (detectorType.compare("SIFT") == 0)
    {
        detKeypointsModern(keypoints, imgGray, "SIFT", bVis);
    }
}
