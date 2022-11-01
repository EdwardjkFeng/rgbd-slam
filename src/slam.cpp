/**
 * @author Jingkun Feng
 * @email jingkun.feng19@gmail.com
 * @create date 2022-10-31 12:05:02
 * @modify date 2022-10-31 13:46:20
 * @desc [description]
 */


#include <iostream>
#include <fstream>
#include <sstream>
using namespace std;


#include "slamBase.h"
#include "parameterReader.h"
#include "indexReader.h"

#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>

#include <g2o/types/slam3d/types_slam3d.h>
#include <g2o/core/sparse_optimizer.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/factory.h>
#include <g2o/core/optimization_algorithm_factory.h>
#include <g2o/core/optimization_algorithm_gauss_newton.h>
#include <g2o/solvers/eigen/linear_solver_eigen.h>
#include <g2o/core/robust_kernel.h>
#include <g2o/core/robust_kernel_impl.h>
#include <g2o/core/optimization_algorithm_levenberg.h>


typedef g2o::BlockSolver_6_3 SlamBlockSolver;
typedef g2o::LinearSolverEigen< g2o::BlockSolver_6_3::PoseMatrixType > SlamLinearSolver;

FRAME readFrame(int index, ParameterReader& pd, IndexReader& idxReader);
double normOfTransform(cv::Mat rvec, cv::Mat tvec);

// Check two frames, define result
enum CHECK_RESULT 
{
    NOT_MATCHED=0, 
    TOO_FAR_AWAY, 
    TOO_CLOSE, 
    KEYFRAME
};

// Define function
CHECK_RESULT checkKeyframes(FRAME& f1, FRAME& f2, g2o::SparseOptimizer& opti, bool is_loops=false);
// Check nearby loops
void checkNearbyLoops(vector<FRAME>& frames, FRAME& currFrame, g2o::SparseOptimizer& opti);
// Check random loops
void checkRandomLoops(vector<FRAME>& frames, FRAME& currFrame, g2o::SparseOptimizer& opti);


int main( int argc, char** argv )
{
    ParameterReader pd;
    int startIndex = atoi(pd.getData("start_index").c_str());
    int endIndex = atoi(pd.getData("end_index").c_str());

    // Store all keyframes in a vector
    vector<FRAME> keyframes;
    // Initialize
    cout << "Initializing ... " << endl;
    int currIndex = startIndex; // Current frame index is start index
    IndexReader idx("../data/rgbd_dataset_freiburg2_pioneer_360/associated.txt");
    FRAME currFrame = readFrame(currIndex, pd, idx);

    string detector = pd.getData("detector");
    string descriptor = pd.getData("descriptor");
    CAMERA_INTRINSIC_PARAMETERS camera = getDefaultCamera();
    computeKeyPointsAndDesp(currFrame, detector, descriptor);
    PointCloud::Ptr cloud = image2PointCloud(currFrame.rgb, currFrame.depth, camera);

    unique_ptr<SlamLinearSolver> linearSolver(new SlamLinearSolver());
    linearSolver->setBlockOrdering( false );
    unique_ptr<SlamBlockSolver> blockSolver(new SlamBlockSolver(move(linearSolver)));
    g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(move(blockSolver));

    g2o::SparseOptimizer globalOptimizer;
    globalOptimizer.setAlgorithm(solver);
    globalOptimizer.setVerbose(false);

    // add new vertex to globalOptimizer
    g2o::VertexSE3* v = new g2o::VertexSE3();
    v->setId(currIndex);
    v->setEstimate(Eigen::Isometry3d::Identity()); // Estimation as identity matrix
    v->setFixed(true); // Fix the first vertex, do not optimize it
    globalOptimizer.addVertex(v);

    keyframes.push_back(currFrame);

    double keyframe_threshold = atof(pd.getData("keyframe_threshold").c_str());

    bool check_loop_closure = pd.getData("check_loop_closure") == string("true");
    for (currIndex = startIndex + 1; currIndex < endIndex; currIndex++)
    {
        cout << "Reading files " << currIndex << endl;
        FRAME currFrame = readFrame(currIndex, pd, idx);
        computeKeyPointsAndDesp(currFrame, detector, descriptor);
        CHECK_RESULT result = checkKeyframes(keyframes.back(), currFrame, globalOptimizer);
        switch(result)
        {
        case NOT_MATCHED:
            cout << RED"Not enough inliers." << endl;
            break;
        case TOO_FAR_AWAY:
            cout << RED"Too far away, may be an error." << endl;
            break;
        case TOO_CLOSE:
            cout << RESET"Too close, not a keyframe." << endl;
            break;
        case KEYFRAME:
            cout << GREEN"This is a new keyframe." << endl;
            if (check_loop_closure)
            {
                checkNearbyLoops(keyframes, currFrame, globalOptimizer);
                checkRandomLoops(keyframes, currFrame, globalOptimizer);
            }
            keyframes.push_back(currFrame);
            break;
        default:
            break;
        }
    }

    cout << RESET"optimizing pose graph, vertices: " << globalOptimizer.vertices().size() << endl;
    globalOptimizer.save("../result/result_before.g2o");
    globalOptimizer.initializeOptimization();
    globalOptimizer.optimize(100);
    globalOptimizer.save("../result/result_after.g2o");
    cout << "Optimization done." << endl;


    cout << "Saving the point cloud map ..." << endl;
    PointCloud::Ptr output( new PointCloud() );
    PointCloud::Ptr tmp( new PointCloud() );

    pcl::VoxelGrid<PointT> voxel;
    pcl::PassThrough<PointT> pass;
    pass.setFilterFieldName("z");
    pass.setFilterLimits(0.0, 10.0);

    double gridsize = atof(pd.getData("voxel_grid").c_str());
    voxel.setLeafSize(gridsize, gridsize, gridsize);

    for (size_t i = 0; i < keyframes.size(); i++)
    {
        g2o::VertexSE3* vertex = dynamic_cast<g2o::VertexSE3*>(globalOptimizer.vertex(keyframes[i].frameID));
        Eigen::Isometry3d pose = vertex->estimate();
        PointCloud::Ptr newCloud = image2PointCloud(keyframes[i].rgb, keyframes[i].depth, camera);
        voxel.setInputCloud(newCloud);
        voxel.filter(*tmp);
        pass.setInputCloud(tmp);
        pass.filter(*newCloud);
        pcl::transformPointCloud(*newCloud, *tmp, pose.matrix());
        *output += *tmp;
        tmp->clear();
        newCloud->clear();
    }

    voxel.setInputCloud(output);
    voxel.filter(*tmp);
    pcl::io::savePCDFile("../result/result.pcd", *tmp);

    cout << "Final map is saved." << endl;
    globalOptimizer.clear();

    return 0; 
}


FRAME readFrame(int index, ParameterReader& pd, IndexReader& idxReader)
{
    FRAME f;
    // string rgbDir = pd.getData("rgb_dir");
    // string depthDir = pd.getData("depth_dir");

    // string rgbExt = pd.getData("rgb_extension");
    // string depthExt = pd.getData("depth_extension");

    string data_dir = pd.getData("data_dir");

    stringstream ss;
    ss << data_dir << idxReader.rgb[index];
    string filename;
    ss >> filename;
    f.rgb = cv::imread(filename);
    // cv::imshow("rgb", f.rgb);
    // cout << filename << endl;

    ss.clear();
    filename.clear();
    ss << data_dir << idxReader.depth[index];
    ss >> filename;

    f.depth = cv::imread(filename, -1);
    // cout << filename << endl;
    // cv::imshow("depth", f.depth);
    f.frameID = index;
    return f;
}


double normOfTransform(cv::Mat rvec, cv::Mat tvec)
{
    return fabs(min(cv::norm(rvec), 2 * M_PI - cv::norm(rvec))) + fabs(cv::norm(tvec));
}

CHECK_RESULT checkKeyframes(FRAME& f1, FRAME& f2, g2o::SparseOptimizer& opti, bool is_loops)
{
    static ParameterReader pd;
    static int min_inliers = atoi(pd.getData("min_inliers").c_str());
    static double max_norm = atof(pd.getData("max_norm").c_str());
    static double keyframe_threshold = atof(pd.getData("keyframe_threshold").c_str());
    static double max_norm_lp = atof( pd.getData("max_norm_lp").c_str() );
    static CAMERA_INTRINSIC_PARAMETERS camera = getDefaultCamera();

    RESULT_OF_PNP result = estimateMotion(f1, f2, camera);
    if (result.inliers < min_inliers)
    {
        return NOT_MATCHED;
    }
    double norm = normOfTransform(result.rvec, result.tvec);
    if (is_loops == false)
    {
        if (norm >= max_norm)
        {
            cout << "norm = " << norm << endl;
            return TOO_FAR_AWAY;
        }
    }
    else
    {
        if(norm >= max_norm_lp)
            return TOO_FAR_AWAY;
    }
    
    if (norm <= keyframe_threshold)
        return TOO_CLOSE;

    if (is_loops == false)
    {
        g2o::VertexSE3 *v = new g2o::VertexSE3();
        v->setId(f2.frameID);
        v->setEstimate(Eigen::Isometry3d::Identity());
        opti.addVertex(v);
    }

    g2o::EdgeSE3* edge = new g2o::EdgeSE3();
    edge->setVertex(0, opti.vertex(f1.frameID));
    edge->setVertex(1, opti.vertex(f2.frameID));
    edge->setRobustKernel(new g2o::RobustKernelHuber());

    Eigen::Matrix<double, 6, 6> information = Eigen::Matrix<double, 6, 6>::Identity();
    information(0, 0) = information(1, 1) = information(2, 2) = 100;
    information(3, 3) = information(4, 4) = information(5, 5) = 100;

    edge->setInformation(information);
    Eigen::Isometry3d T = cvMat2Eigen(result.rvec, result.tvec);
    edge->setMeasurement(T.inverse());
    opti.addEdge(edge);
    return KEYFRAME;
}


void checkNearbyLoops(vector<FRAME>& frames, FRAME& currFrame, g2o::SparseOptimizer& opti)
{
    static ParameterReader pd;
    static int nearby_loops = atoi(pd.getData("nearby_loops").c_str());

    if (frames.size()  <= nearby_loops)
    {
        for (size_t i = 0; i < frames.size(); i++)
        {
            checkKeyframes(frames[i], currFrame, opti, true);
        }
        
    }
    else
    {
        for (size_t i = frames.size() - nearby_loops; i < frames.size(); i++)
        {
            checkKeyframes(frames[i], currFrame, opti, true);
        }
        
    }
}


void checkRandomLoops(vector<FRAME>& frames, FRAME& currFrame, g2o::SparseOptimizer& opti)
{
    static ParameterReader pd;
    static int random_loops = atoi(pd.getData("random_loops").c_str());
    srand((unsigned int) time(NULL));

    if (frames.size() <= random_loops)
    {
        for (size_t i = 0; i < frames.size(); i++)
        {
            checkKeyframes(frames[i], currFrame, opti, true);
        }
    }
    else
    {
        for (int i = 0; i < random_loops; i++)
        {
            int index = rand()%frames.size();
            checkKeyframes(frames[index], currFrame, opti, true);
        }
        
    }
}