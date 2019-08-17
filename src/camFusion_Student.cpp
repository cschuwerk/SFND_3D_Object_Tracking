
#include <iostream>
#include <algorithm>
#include <numeric>
#include <map>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "camFusion.hpp"
#include "dataStructures.h"

using namespace std;


// Create groups of Lidar points whose projection into the camera falls into the same bounding box
void clusterLidarWithROI(std::vector<BoundingBox> &boundingBoxes, std::vector<LidarPoint> &lidarPoints, float shrinkFactor, cv::Mat &P_rect_xx, cv::Mat &R_rect_xx, cv::Mat &RT)
{
    // loop over all Lidar points and associate them to a 2D bounding box
    cv::Mat X(4, 1, cv::DataType<double>::type);
    cv::Mat Y(3, 1, cv::DataType<double>::type);

    for (auto it1 = lidarPoints.begin(); it1 != lidarPoints.end(); ++it1)
    {
        // assemble vector for matrix-vector-multiplication
        X.at<double>(0, 0) = it1->x;
        X.at<double>(1, 0) = it1->y;
        X.at<double>(2, 0) = it1->z;
        X.at<double>(3, 0) = 1;

        // project Lidar point into camera
        Y = P_rect_xx * R_rect_xx * RT * X;
        cv::Point pt;
        pt.x = Y.at<double>(0, 0) / Y.at<double>(0, 2); // pixel coordinates
        pt.y = Y.at<double>(1, 0) / Y.at<double>(0, 2);

        vector<vector<BoundingBox>::iterator> enclosingBoxes; // pointers to all bounding boxes which enclose the current Lidar point
        for (vector<BoundingBox>::iterator it2 = boundingBoxes.begin(); it2 != boundingBoxes.end(); ++it2)
        {
            // shrink current bounding box slightly to avoid having too many outlier points around the edges
            cv::Rect smallerBox;
            smallerBox.x = (*it2).roi.x + shrinkFactor * (*it2).roi.width / 2.0;
            smallerBox.y = (*it2).roi.y + shrinkFactor * (*it2).roi.height / 2.0;
            smallerBox.width = (*it2).roi.width * (1 - shrinkFactor);
            smallerBox.height = (*it2).roi.height * (1 - shrinkFactor);

            // check wether point is within current bounding box
            if (smallerBox.contains(pt))
            {
                enclosingBoxes.push_back(it2);
            }

        } // eof loop over all bounding boxes

        // check wether point has been enclosed by one or by multiple boxes
        if (enclosingBoxes.size() == 1)
        { 
            // add Lidar point to bounding box
            enclosingBoxes[0]->lidarPoints.push_back(*it1);
        }

    } // eof loop over all Lidar points
}


void show3DObjects(std::vector<BoundingBox> &boundingBoxes, cv::Size worldSize, cv::Size imageSize, bool bWait)
{
    // create topview image
    cv::Mat topviewImg(imageSize, CV_8UC3, cv::Scalar(255, 255, 255));

    for(auto it1=boundingBoxes.begin(); it1!=boundingBoxes.end(); ++it1)
    {
        // create randomized color for current 3D object
        cv::RNG rng(it1->boxID);
        cv::Scalar currColor = cv::Scalar(rng.uniform(0,150), rng.uniform(0, 150), rng.uniform(0, 150));

        // plot Lidar points into top view image
        int top=1e8, left=1e8, bottom=0.0, right=0.0; 
        float xwmin=1e8, ywmin=1e8, ywmax=-1e8;
        std::vector<double> xValues;
        for (auto it2 = it1->lidarPoints.begin(); it2 != it1->lidarPoints.end(); ++it2)
        {
            // world coordinates
            float xw = (*it2).x; // world position in m with x facing forward from sensor
            float yw = (*it2).y; // world position in m with y facing left from sensor
            xwmin = xwmin<xw ? xwmin : xw;
            ywmin = ywmin<yw ? ywmin : yw;
            ywmax = ywmax>yw ? ywmax : yw;

            // top-view coordinates
            int y = (-xw * imageSize.height / worldSize.height) + imageSize.height;
            int x = (-yw * imageSize.width / worldSize.width) + imageSize.width / 2;

            // find enclosing rectangle
            top = top<y ? top : y;
            left = left<x ? left : x;
            bottom = bottom>y ? bottom : y;
            right = right>x ? right : x;

            // draw individual point
            cv::circle(topviewImg, cv::Point(x, y), 4, currColor, -1);

            xValues.push_back(xw);
        }

        // Calculate median of xw values
        double xw_median = calculateMedian(xValues);
        if(xw_median != NAN) {
            int y = (-xw_median * imageSize.height / worldSize.height) + imageSize.height;
            cv::rectangle(topviewImg, cv::Point(left+(right-left)/2-5, y+5), cv::Point(left+(right-left)/2+5,y-5 ), cv::Scalar(0,0,255),-1);
        }

        // draw enclosing rectangle
        cv::rectangle(topviewImg, cv::Point(left, top), cv::Point(right, bottom),cv::Scalar(0,0,0), 2);

        // augment object with some key data
        char str1[200], str2[200], str3[200];
        sprintf(str1, "id=%d, #pts=%d", it1->boxID, (int)it1->lidarPoints.size());
        putText(topviewImg, str1, cv::Point2f(left-250, bottom+50), cv::FONT_ITALIC, 2, currColor);
        sprintf(str2, "xmin=%2.2f m, yw=%2.2f m", xwmin, ywmax-ywmin);
        putText(topviewImg, str2, cv::Point2f(left-250, bottom+125), cv::FONT_ITALIC, 2, currColor);
        sprintf(str3, "xmed=%2.2fm", xw_median);
        putText(topviewImg, str3, cv::Point2f(left-250, bottom+200), cv::FONT_ITALIC, 2, currColor);
    }

    // plot distance markers
    float lineSpacing = 2.0; // gap between distance markers
    int nMarkers = floor(worldSize.height / lineSpacing);
    for (size_t i = 0; i < nMarkers; ++i)
    {
        int y = (-(i * lineSpacing) * imageSize.height / worldSize.height) + imageSize.height;
        cv::line(topviewImg, cv::Point(0, y), cv::Point(imageSize.width, y), cv::Scalar(255, 0, 0));
    }

    // display image
    string windowName = "3D Objects";
    cv::namedWindow(windowName, 1);
    cv::imshow(windowName, topviewImg);

    if(bWait)
    {
        cv::waitKey(0); // wait for key to be pressed
    }
}


// associate a given bounding box with the keypoints it contains
void clusterKptMatchesWithROI(BoundingBox &boundingBox, std::vector<cv::KeyPoint> &kptsPrev, std::vector<cv::KeyPoint> &kptsCurr, std::vector<cv::DMatch> &kptMatches)
{
    const bool filterOutliers = true;
    std::vector<double> kptDistances;
    double mean=0.0,minDistance,maxDistance;

    if(filterOutliers) {
        // Iterate over all matches and calculate the Euclidian distance
        for(auto kptMatch=kptMatches.begin(); kptMatch!=kptMatches.end(); ++kptMatch) {
            if(boundingBox.roi.contains(kptsCurr[kptMatch->trainIdx].pt)) {
                double distance = cv::norm(kptsPrev[kptMatch->queryIdx].pt - kptsCurr[kptMatch->trainIdx].pt);
                kptDistances.push_back(distance);
                mean+=distance;
            }
        }

        // Find the min./max distance based on the mean and standard deviation
        mean = mean/kptDistances.size();
        double std = calculateStd(kptDistances, mean);
        minDistance = mean - 2*std;
        maxDistance = mean + 2*std;

        // Find the the min./max. distances, discarding most extreme 10% of the values
        //sort(kptDistances.begin(), kptDistances.end());
        //minDistance = kptDistances[int(floor(kptDistances.size()*0.1))];
        //maxDistance = kptDistances[int(floor(kptDistances.size()*0.9))];
    }

    // Loop over all matches, filter them and add them to the bounding box if they are within the ROI
    int n=0;
    for(int i=0; i<kptMatches.size(); ++i) {

        if(boundingBox.roi.contains(kptsCurr[kptMatches[i].trainIdx].pt)) {

            if(filterOutliers) {
                double distance = cv::norm(kptsPrev[kptMatches[i].queryIdx].pt - kptsCurr[kptMatches[i].trainIdx].pt);
                if(distance< minDistance || distance > maxDistance) {
                    n++;
                    continue;
                }
            }
            boundingBox.kptMatches.push_back(kptMatches[i]);
            boundingBox.keypoints.push_back(kptsCurr.at(kptMatches[i].trainIdx));
        }
    }
    // Only for debugging
    std::cout << "Filtered " << n << " outliers (kept " << boundingBox.kptMatches.size() << " out of " << kptMatches.size() << ")" << std::endl;
}


// Compute time-to-collision (TTC) based on keypoint correspondences in successive images
void computeTTCCamera(std::vector<cv::KeyPoint> &kptsPrev, std::vector<cv::KeyPoint> &kptsCurr, 
                      std::vector<cv::DMatch> kptMatches, double frameRate, double &TTC, cv::Mat *visImg)
{
    // compute distance ratios between all matched keypoints
    vector<double> distRatios; // stores the distance ratios for all keypoints between curr. and prev. frame
    for (auto it1 = kptMatches.begin(); it1 != kptMatches.end() - 1; ++it1)
    { // outer kpt. loop

        // get current keypoint and its matched partner in the prev. frame
        cv::KeyPoint kpOuterCurr = kptsCurr.at(it1->trainIdx);
        cv::KeyPoint kpOuterPrev = kptsPrev.at(it1->queryIdx);

        for (auto it2 = kptMatches.begin() + 1; it2 != kptMatches.end(); ++it2)
        { // inner kpt.-loop

            double minDist = 100.0; // min. required distance

            // get next keypoint and its matched partner in the prev. frame
            cv::KeyPoint kpInnerCurr = kptsCurr.at(it2->trainIdx);
            cv::KeyPoint kpInnerPrev = kptsPrev.at(it2->queryIdx);

            // compute distances and distance ratios
            double distCurr = cv::norm(kpOuterCurr.pt - kpInnerCurr.pt);
            double distPrev = cv::norm(kpOuterPrev.pt - kpInnerPrev.pt);

            if (distPrev > std::numeric_limits<double>::epsilon() && distCurr >= minDist)
            { // avoid division by zero

                double distRatio = distCurr / distPrev;
                distRatios.push_back(distRatio);
            }
        } // eof inner loop over all matched kpts
    }     // eof outer loop over all matched kpts

    // only continue if list of distance ratios is not empty
    if (distRatios.size() == 0)
    {
        TTC = NAN;
        return;
    }

    // compute camera-based TTC from distance ratios and the median
    double dT = 1 / frameRate;
    double medianDistRatio = calculateMedian(distRatios);
    TTC = -dT / (1 - medianDistRatio);
}


void computeTTCLidar(std::vector<LidarPoint> &lidarPointsPrev,
                     std::vector<LidarPoint> &lidarPointsCurr, double frameRate, double &TTC, resultTTC &res)
{

    double t = (double)cv::getTickCount();

    double dT = 1/frameRate;        // time between two measurements in seconds

    // Using the median
    double medianPrev=0.0, medianCurr=0.0, minCurr=10000.0, minPrev=10000.0;
    std::vector<double> distPrev, distCurr;

    for (auto it = lidarPointsPrev.begin(); it != lidarPointsPrev.end(); ++it)
    {
        distPrev.push_back(it->x);
        minPrev = (it->x < minPrev) ? it->x : minPrev;

    }

    for (auto it = lidarPointsCurr.begin(); it != lidarPointsCurr.end(); ++it)
    {
        distCurr.push_back(it->x);
        minCurr = (it->x < minCurr) ? it->x : minCurr;
    }

    // Calculate the median of the lidar values in x-direction
    medianPrev = calculateMedian(distPrev);
    medianCurr = calculateMedian(distCurr);

    // Store values for evaluation purposes
    res.LidarMedian.push_back(medianCurr);
    res.LidarMin.push_back(minCurr);

    // compute TTC from both measurements
    TTC = medianCurr * dT / (medianPrev - medianCurr);
    //TTC = minCurr * dT / (minPrev - minCurr);
    res.TTCLidar.push_back(TTC);

    t = ((double)cv::getTickCount() - t) / cv::getTickFrequency();
    cout << "computeTTCLidar in " << 1000 * t / 1.0 << " ms" << endl;
}

double calculateMedian(std::vector<double> &values) {

    sort(values.begin(), values.end());
    int size = values.size();

    if(size == 0) return NAN;

    return size%2==0 ? (values[size / 2 - 1] + values[size / 2]) / 2 : values[size / 2];
}

double calculateStd(std::vector<double> &values, double &mean) {

    if(values.size() == 0)
        return 0.0;

    double sumsd = 0.0;
    std::for_each (values.begin(), values.end(), [&](const double d) {
        sumsd += (d - mean) * (d - mean);
    });
    return sqrt(sumsd / values.size());
}

void matchBoundingBoxes(std::vector<cv::DMatch> &matches, std::map<int, int> &bbBestMatches, DataFrame &prevFrame, DataFrame &currFrame)
{

    const int minNumberOfMatches = 5; // Bounding box matches are only considered, if there are at least minNumberOfMatches common keypoints
    double t = (double)cv::getTickCount();

    // Iterate through all matches
    std::multimap<int,int> prevFrameMatchToBBox;    // contains a mapping between keypoint id and bbox id
    std::map<int,std::vector<int>> currFrameBBoxMatches; // contains a map between the bbox id and a vector of all keypoints within this bbox in the current frame

    for(auto match=matches.begin(); match!=matches.end(); ++match) {

        // Go through all bounding boxes of the current frame
        for(auto bbox=currFrame.boundingBoxes.begin(); bbox!=currFrame.boundingBoxes.end(); ++bbox) {

            currFrameBBoxMatches.insert({bbox->boxID,std::vector<int>()});

            //Check if the keypoint lies inside the bbox and add it to the vector
            if(bbox->roi.contains(currFrame.keypoints.at(match->trainIdx).pt)) {
                currFrameBBoxMatches.at(bbox->boxID).push_back(match->queryIdx);
            }
        }

        // Go through all bounding boxes of the previous frame
        for(auto bbox=prevFrame.boundingBoxes.begin(); bbox!=prevFrame.boundingBoxes.end(); ++bbox) {

            //Check if the keypoint lies inside the bbox
            if(bbox->roi.contains(prevFrame.keypoints.at(match->queryIdx).pt)) {
                prevFrameMatchToBBox.insert({match->queryIdx, bbox->boxID});
            }
        }
    } // end iterate over all matches

    // Go through all bounding boxes in the current image and find the bbox in the previous image with the most common keypoints
    for(auto bbox=currFrameBBoxMatches.begin(); bbox!=currFrameBBoxMatches.end(); ++bbox ) {
        std::map<int,int> matchCounter;

        // Go through all matches in this bbox
        for(int i=0; i <bbox->second.size(); ++i) {
            // Check if this match id is associated with which bbox
            // bbox->first is the ID of the bbox
            // bbox->second[i] is the ID of the match
            auto ret = prevFrameMatchToBBox.equal_range(bbox->second[i]);
            for(auto it=ret.first; it!=ret.second; ++it) {
                //it->second is the bbox id in the previous frame
                if(matchCounter.find(it->second) != matchCounter.end()) {
                    matchCounter.at(it->second)++;
                }
                else {
                    matchCounter.insert({it->second,0});
                }
            }
        }

        // Find the bbox id with the max number of identical keypoint matches
        auto bestMatch = std::max_element(matchCounter.begin(), matchCounter.end(), [] (const std::pair<int,int> &p1, const std::pair<int,int> &p2) { return p1.second < p2.second; } );

        // Consider the match only if there are at least minNumberOfMatches
        if(bestMatch->second >= minNumberOfMatches) {
            bbBestMatches.insert({bestMatch->first, bbox->first});
        }

    } // end iterate over all bboxes

    t = ((double)cv::getTickCount() - t) / cv::getTickFrequency();
    cout << "matchBoundingBoxes in " << 1000 * t / 1.0 << " ms" << endl;

}
