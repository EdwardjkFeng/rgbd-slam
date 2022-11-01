/**
 * @author Jingkun Feng
 * @email jingkun.feng19@gmail.com
 * @create date 2022-10-27 03:01:33
 * @modify date 2022-10-27 03:01:33
 * @desc [description]
 */

#include <iostream>
#include <fstream>
#include <sstream>
using namespace std;

#include "slamBase.h"
#include "parameterReader.h"
#include "indexReader.h"

FRAME readFrame(int index, ParameterReader& pd, IndexReader& rgb_id, IndexReader& depth_id);

double normOfTransform(cv::Mat rvec, cv::Mat tvec);

int main(int argc, char** argv)
{
    ParameterReader pd;
    int startIndex = atoi(pd.getData("start_index").c_str());
    int endIndex = atoi(pd.getData("end_index").c_str());

    // initialize
    cout << "Initializing ..." << endl;
    int currIndex = startIndex;
    IndexReader rgb_id("../data/rgbd_dataset_freiburg2_pioneer_360/rgb.txt");
    IndexReader depth_id("../data/rgbd_dataset_freiburg2_pioneer_360/depth.txt");
    FRAME lastFrame = readFrame(currIndex, pd, rgb_id, depth_id);

    string detector = pd.getData("detector");
    string descriptor = pd.getData("descriptor");
    CAMERA_INTRINSIC_PARAMETERS camera = getDefaultCamera();
    computeKeyPointsAndDesp(lastFrame, detector, descriptor);
    PointCloud::Ptr cloud = image2PointCloud(lastFrame.rgb, lastFrame.depth, camera);

    pcl::visualization::CloudViewer viewer("viewer");

    bool visualize = pd.getData("visualize_pointcloud") == string("true");

    int min_inliers = atoi(pd.getData("min_inliers").c_str());
    double max_norm = atof(pd.getData("max_norm").c_str());

    for (currIndex = startIndex + 1; currIndex < endIndex; currIndex++)
    {
        cout << "Reading files" << currIndex << endl;
        FRAME currFrame = readFrame(currIndex, pd, rgb_id, depth_id);
        computeKeyPointsAndDesp(currFrame, detector, descriptor);

        RESULT_OF_PNP result = estimateMotion(lastFrame, currFrame, camera);
        if (result.inliers < min_inliers) // too few inliers, drop frame
            continue;

        double norm = normOfTransform(result.rvec, result.tvec);
        cout << "norm = " << norm << endl;
        if (norm >= max_norm) 
        // range of motion is too large, transform between adjacent frames is supposed to be small
            continue;
        Eigen::Isometry3d T = cvMat2Eigen(result.rvec, result.tvec);
        cout << "T = " << T.matrix() << endl;

        cloud = jointPointCloud(cloud, currFrame, T, camera);

        if (visualize == true)
            viewer.showCloud(cloud);

        lastFrame = currFrame;
    }
    

    pcl::io::savePCDFile("../data/result.pcd", *cloud);
    
    return 0;
}


FRAME readFrame(int index, ParameterReader& pd, IndexReader& rgb_id, IndexReader& depth_id)
{
    FRAME f;
    string rgbDir = pd.getData("rgb_dir");
    string depthDir = pd.getData("depth_dir");

    string rgbExt = pd.getData("rgb_extension");
    string depthExt = pd.getData("depth_extension");

    stringstream ss;
    ss << rgbDir << rgb_id.indices[index] << rgbExt;
    string filename;
    ss >> filename;
    f.rgb = cv::imread(filename);

    ss.clear();
    filename.clear();
    ss << depthDir << depth_id.indices[index] << depthExt;
    ss >> filename;

    f.depth = cv::imread(filename, -1);
    f.frameID = index;
    return f;
}


double normOfTransform(cv::Mat rvec, cv::Mat tvec)
{
    return fabs(min(cv::norm(rvec), 2 * M_PI - cv::norm(rvec))) + fabs(cv::norm(tvec));
}