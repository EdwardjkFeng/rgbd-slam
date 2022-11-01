/**
 * @author Jingkun Feng
 * @email jingkun.feng19@gmail.com
 * @create date 2022-10-28 15:36:12
 * @modify date 2022-10-28 15:36:12
 * @desc [description]
 */

#include <iostream>
#include <fstream>
#include <sstream>
using namespace std;

#include "slamBase.h"
#include "parameterReader.h"
#include "indexReader.h"

// g2o header files
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


// 给定index, 读取一帧数据
FRAME readFrame(int index, ParameterReader& pd, IndexReader& rgb_id, IndexReader& depth_id);

double normOfTransform(cv::Mat rvec, cv::Mat tvec);

int main(int argc, char** argv)
{
    ParameterReader pd;
    int startIndex = atoi(pd.getData("start_index").c_str());
    int endIndex = atoi(pd.getData("end_index").c_str());

    // Initialize
    cout << "Initializing ... " << endl;
    int currIndex = startIndex;
    IndexReader rgb_id("../data/rgbd_dataset_freiburg2_pioneer_360/rgb.txt");
    IndexReader depth_id("../data/rgbd_dataset_freiburg2_pioneer_360/depth.txt");
    FRAME lastFrame = readFrame(currIndex, pd, rgb_id, depth_id);

    // compare last frame and current frame
    string detector = pd.getData("detector");
    string descriptor = pd.getData("descriptor");
    CAMERA_INTRINSIC_PARAMETERS camera = getDefaultCamera();
    computeKeyPointsAndDesp(lastFrame, detector, descriptor);
    PointCloud::Ptr cloud = image2PointCloud(lastFrame.rgb, lastFrame.depth, camera);

    pcl::visualization::CloudViewer viewer("viewer");

    // if should point cloud
    bool visualize = pd.getData("visualize_pointcloud") == string("true");

    int min_inliers = atoi(pd.getData("min_inliers").c_str());
    double max_norm = atof(pd.getData("max_norm").c_str());

    /***********************
     * G2O initialization
    ************************/
    // Choose optimization method
    typedef g2o::BlockSolver_6_3 SlamBlockSolver;
    typedef g2o::LinearSolverEigen< g2o::BlockSolver_6_3::PoseMatrixType > SlamLinearSolver;

    // Initializing solver
    std::unique_ptr<SlamLinearSolver> linearSolver (new SlamLinearSolver());
    linearSolver->setBlockOrdering(false);
    std::unique_ptr<SlamBlockSolver> blockSolver( new SlamBlockSolver( std::move(linearSolver)) );
    g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg( std::move(blockSolver) );

    g2o::SparseOptimizer globalOptimizer;
    globalOptimizer.setAlgorithm( solver );
    // do not output verbose 
    globalOptimizer.setVerbose( false );

    g2o::VertexSE3* v = new g2o::VertexSE3();
    v->setId(currIndex);
    v->setEstimate(Eigen::Isometry3d::Identity());
    v->setFixed(true); // fix the first vertex, which is not optimized
    globalOptimizer.addVertex(v);

    int lastIndex = currIndex;

    for (currIndex = startIndex + 1; currIndex < endIndex; currIndex ++)
    {
        cout << "Reading files " << currIndex << endl;
        FRAME currFrame = readFrame(currIndex, pd, rgb_id, depth_id);
        computeKeyPointsAndDesp(currFrame, detector, descriptor);
        RESULT_OF_PNP result = estimateMotion(lastFrame, currFrame, camera);
        if (result.inliers < min_inliers) // if not enough inliers, drop
            continue;
        
        double norm = normOfTransform(result.rvec, result.tvec);
        cout << "norm = " << norm << endl;
        if (norm >= max_norm)
            continue;
        Eigen::Isometry3d T = cvMat2Eigen(result.rvec, result.tvec);
        cout << "T = " << T.matrix() << endl;

        if ( visualize == true )
        {
            cloud = jointPointCloud( cloud, currFrame, T, camera );
            viewer.showCloud( cloud );
        }

        // cloud = jointPointCloud(cloud, currFrame, T, camera);

        // Add edge to g2o between current vertex and last frame
        // Vertex
        // Set vertex id
        g2o::VertexSE3 *v = new g2o::VertexSE3();
        v->setId(currIndex);
        v->setEstimate(Eigen::Isometry3d::Identity());
        globalOptimizer.addVertex(v);
        // Edge
        g2o::EdgeSE3* edge = new g2o::EdgeSE3();
        // Two vertices on the edge
        edge->vertices()[0] = globalOptimizer.vertex(lastIndex);
        edge->vertices()[1] = globalOptimizer.vertex(currIndex);
        // Info matrix
        Eigen::Matrix<double, 6, 6>information = Eigen::Matrix<double, 6, 6>::Identity();
        // 信息矩阵是协方差矩阵的逆，表示我们对边的精度的预先估计
        // 因为pose为6D的，信息矩阵是6*6的阵，假设位置和角度的估计精度均为0.1且互相独立
        // 那么协方差则为对角为0.01的矩阵，信息阵则为100的矩阵
        information(0, 0) = information(1, 1) = information(2, 2) = 100;
        information(3, 3) = information(4, 4) = information(5, 5) = 100;
        // 也可以将角度设大一些，表示对角度的估计更加准确
        edge->setInformation(information);
        // 边的估计即是pnp求解之结果
        edge->setMeasurement(T);
        // 将此边加入图中
        globalOptimizer.addEdge(edge);

        lastFrame = currFrame;
        lastIndex = currIndex;
    }

    // Optimize all edges 
    cout << "optimizing pose graph, vertices: " << globalOptimizer.vertices().size() << endl;
    globalOptimizer.save("../data/result_before.g2o");
    globalOptimizer.initializeOptimization();
    globalOptimizer.optimize(100); // Interations
    globalOptimizer.save("../data/result_after.g2o");
    cout << "Optimization done." << endl;

    globalOptimizer.clear();

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