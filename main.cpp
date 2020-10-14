#include <iostream>
#include "fileOps.h"
#include "params.h"

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/calib3d.hpp>

#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/voxel_grid.h>

#include <g2o/types/slam3d/types_slam3d.h>
#include <g2o/core/sparse_optimizer.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_factory.h>
#include <g2o/solvers/csparse/linear_solver_csparse.h>
#include <g2o/core/optimization_algorithm_levenberg.h>

typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<PointT> PointCloud;

using namespace std;

struct feature_data{
    cv::Mat rvec, tvec;
    int inliers;
};


cv::Point3f point2dTo3d( cv::Point3f& point, CAMERA& camera )
{
    cv::Point3f p; // 3D
    p.z = double( point.z ) / camera.factor;
    p.x = ( point.x - camera.cx) * p.z / camera.fx;
    p.y = ( point.y - camera.cy) * p.z / camera.fy;
    return p;
}

// cvMat2Eigen
Eigen::Isometry3d cvMat2Eigen( cv::Mat& rvec, cv::Mat& tvec )
{
    cv::Mat R;
    cv::Rodrigues( rvec, R );
    Eigen::Matrix3d r;
    for ( int i=0; i<3; i++ )
        for ( int j=0; j<3; j++ )
            r(i,j) = R.at<double>(i,j);


    Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
    // Convert translation and rotation matrix into transformation matrix t
    Eigen::AngleAxisd angle(r);
    T = angle;
    T(0,3) = tvec.at<double>(0,0);
    T(1,3) = tvec.at<double>(1,0);
    T(2,3) = tvec.at<double>(2,0);
    return T;
}

struct feature_data getFeatures(cv::Mat rgb1,cv::Mat depth1,cv::Mat rgb2,cv::Mat depth2)
{
    feature_data result;
    vector<cv::KeyPoint>kpts1,kpts2;
    cv::Mat desc1, desc2,res;
    vector<cv::DMatch> matches;

    cv::Ptr<cv::ORB> detector = cv::ORB::create();

    // find the keypoints and descriptors with ORB
    cv::Ptr<cv::ORB> orb = cv::ORB::create();
    orb->detectAndCompute(rgb1, cv::Mat(), kpts1, desc1);
    orb->detectAndCompute(rgb2, cv::Mat(), kpts2, desc2);


    cv::BFMatcher desc_matcher(cv::NORM_L2, true);
    desc_matcher.match(desc1, desc2, matches, cv::Mat());

    cv::drawMatches(rgb1, kpts1, rgb2, kpts2, matches, res, cv::Scalar::all(-1),
                    cv::Scalar::all(-1), cv::Mat(), cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);


    vector< cv::DMatch > goodMatches;
    double minDis = 9999;
    double good_match_threshold = 10 ;
    for ( size_t i=0; i<matches.size(); i++ )
    {
        if ( matches[i].distance < minDis )
            minDis = matches[i].distance;
    }

    cout<<"min dis = "<<minDis<<endl;
    if ( minDis < 10 )
        minDis = 10;

    for ( size_t i=0; i<matches.size(); i++ )
    {
        if (matches[i].distance < good_match_threshold*minDis )
            goodMatches.push_back( matches[i] );
    }

    cout<<"good matches: "<<goodMatches.size()<<endl;

    if (goodMatches.size() <= 15)
    {
        result.inliers = -1;
        return result;
    }

    vector<cv::Point3f> pts_obj;
    vector< cv::Point2f > pts_img;

    for (size_t i=0; i<goodMatches.size(); i++)
    {

        cv::Point2f p = kpts1[goodMatches[i].queryIdx].pt;
        ushort d =depth1 .ptr<ushort>( int(p.y) )[ int(p.x) ];
        if (d == 0)
            continue;
        pts_img.push_back( cv::Point2f( kpts2[goodMatches[i].trainIdx].pt ) );

        // convert image coordianrtes to world cordinates
        cv::Point3f pt ( p.x, p.y, d );
        cv::Point3f pd = point2dTo3d( pt, camera );
        pts_obj.push_back( pd );
    }

    if (pts_obj.size() ==0 || pts_img.size()==0)
    {
        result.inliers = -1;
        cout <<"Result ==-1"<<endl;
        return result;
    }

    double camera_matrix_data[3][3] = {
            {camera.fx, 0, camera.cx},
            {0, camera.fy, camera.cy},
            {0, 0, 1}
    };



    cv::Mat rvec, tvec, inliers;
    //  solvepnp
    std::cout<<"here"<<endl;


    cv::Mat cameraMatrix( 3, 3, CV_64F, camera_matrix_data );
    cv::solvePnPRansac( pts_obj, pts_img, cameraMatrix, cv::Mat(), rvec, tvec, false, 100, 1.0, 0.99, inliers );
    std::cout <<"here2"<<endl;

    result.rvec = rvec;
    result.tvec = tvec;
    result.inliers = inliers.rows;

    return result;

}
PointCloud::Ptr image2PointCloud( cv::Mat& rgb, cv::Mat& depth, CAMERA& camera )
{
    PointCloud::Ptr cloud ( new PointCloud );

    for (int m = 0; m < depth.rows; m+=2)
        for (int n=0; n < depth.cols; n+=2)
        {
            //get depth value
            ushort d = depth.ptr<ushort>(m)[n];

            if (d == 0)
                continue;
            PointT p;

            // calculate 2d imaage to 3d world
            p.z = double(d) / camera.factor;
            p.x = (n - camera.cx) * p.z / camera.fx;
            p.y = (m - camera.cy) * p.z / camera.fy;

            //
            p.b = rgb.ptr<uchar>(m)[n*3];
            p.g = rgb.ptr<uchar>(m)[n*3+1];
            p.r = rgb.ptr<uchar>(m)[n*3+2];

            cloud->points.push_back( p );
        }

    cloud->height = 1;
    cloud->width = cloud->points.size();
    cloud->is_dense = false;

    return cloud;
}
//
//void generatePointcloud( cv::Mat rgb,cv::Mat depth, pcl::visualization::CloudViewer& viewer)
//{
//
//
//    // Point cloud variables
//    // Using smart pointers, create an empty point cloud. This pointer will be automatically released when it runs out.。
//    // Traverse the depth map
//    for (int m = 0; m < depth.rows; m++)
//        for (int n=0; n < depth.cols; n++)
//        {
//            // Get the value at (m,n) in the depth map
//            ushort d = depth.ptr<ushort>(m)[n];
//            // d There may be no value, if so, skip this point
//            if (d == 0)
//                continue;
//            // d Existing value, add a point to the point cloud
//            PointT p;
//
//            // Calculate the space coordinates of this point
//            p.z = double(d) / camera.factor;
//            p.x = (n - camera.cx) * p.z / camera.fx;
//            p.y = (m - camera.cy) * p.z / camera.fy;
//
//
//            p.b = rgb.ptr<uchar>(m)[n*3];
//            p.g = rgb.ptr<uchar>(m)[n*3+1];
//            p.r = rgb.ptr<uchar>(m)[n*3+2];
//
//
//            cloud->points.push_back( p );
//        }
//    cloud->height = 1;
//    cloud->width = cloud->points.size();
//    cout<<"point cloud size = "<<cloud->points.size()<<endl;
//    cloud->is_dense = false;
//
//    viewer.showCloud( cloud );
//    while( !viewer.wasStopped() )
//    {
//        break;
//
//    }

//}

PointCloud::Ptr joinPointCloud( PointCloud::Ptr original, cv::Mat rgb,cv::Mat depth, Eigen::Isometry3d T, CAMERA camera )
{
    PointCloud::Ptr newCloud = image2PointCloud( rgb, depth, camera );

    // initiate new point cloud
    PointCloud::Ptr output (new PointCloud());
    pcl::transformPointCloud( *original, *output, T.matrix() );
    *newCloud += *output;

    // Voxel grid
    static pcl::VoxelGrid<PointT> voxel;
    double gridsize = 0.005; // limitinf pcl grid points
    voxel.setLeafSize( gridsize, gridsize, gridsize );
    voxel.setInputCloud( newCloud );
    PointCloud::Ptr tmp( new PointCloud() );
    voxel.filter( *tmp );
    return tmp;
}
double normofTransform( cv::Mat rvec, cv::Mat tvec )
{
    return fabs(min(cv::norm(rvec), 2*M_PI-cv::norm(rvec)))+ fabs(cv::norm(tvec));
}

int main() {
    fileOps _fileOps;
    feature_data PNpresult;

    pcl::visualization::CloudViewer viewer( "viewer" );

    int min_inliers = 5;
    double max_norm = 2.0;


    // initilab slam blocker
    typedef g2o::BlockSolver_6_3 SlamBlockSolver;
    typedef g2o::LinearSolverCSparse< SlamBlockSolver::PoseMatrixType > SlamLinearSolver;

    // initilize linear solver
    SlamLinearSolver* linearSolver = new SlamLinearSolver();
    linearSolver->setBlockOrdering( false );
    SlamBlockSolver* blockSolver = new SlamBlockSolver( linearSolver );
    g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg( blockSolver );

    g2o::SparseOptimizer globalOptimizer;
    globalOptimizer.setAlgorithm( solver );
    globalOptimizer.setVerbose( false );//disable debugging
    g2o::VertexSE3* v = new g2o::VertexSE3();
    v->setId( 0 );
    v->setEstimate( Eigen::Isometry3d::Identity() ); //identy matrix
    v->setFixed( true ); // initial vertix without optimization
    globalOptimizer.addVertex( v );
    PointCloud::Ptr cloud ( new PointCloud );


    string rootfoldet="/home/nagesh/Desktop/rgbd_dataset_freiburg1_room/";
    for (int i =0; i<_fileOps.rgb_names.size()-1; i++)
    {
        // read N-th frame rgb +depth
        cv::Mat image1 =cv::imread(rootfoldet+_fileOps.rgb_names[i]);
        cv::imshow("image",image1);
        cv::Mat depth1 =cv::imread(rootfoldet+_fileOps.depth_names[i]);
        cv::imshow("depth",depth1);
        cv::waitKey(1);

        // read N+1 th frame rgb +depth
        cv::Mat image2 =cv::imread(rootfoldet+_fileOps.rgb_names[i+1]);
//        cv::imshow("image2",image2);
        cv::Mat depth2 =cv::imread(rootfoldet+_fileOps.depth_names[i+1]);
//        cv::imshow("depth2",depth2);
//        cv::waitKey(1);
//        generatePointcloud(image1,depth1,viewer);
        PNpresult=getFeatures(image1,depth1,image2,depth2);
        cout << PNpresult.inliers<<endl;
        if ( PNpresult.inliers < min_inliers ) //inliers不够，放弃该帧
            continue;
        cout << "continued"<<endl;

        double norm = normofTransform(PNpresult.rvec, PNpresult.tvec);
        cout<<"norm = "<<norm<<endl;
        if ( norm >= max_norm )
            continue;

        Eigen::Isometry3d T = cvMat2Eigen( PNpresult.rvec, PNpresult.tvec );
        cout<<"T="<<T.matrix()<<endl;

         cloud = joinPointCloud( cloud, image1,depth1, T, camera );
        viewer.showCloud( cloud );
        if(!(i%10)){
            while (!viewer.wasStopped()) {
                break;

            }
        }

        // Add the edge of this vertex to the previous frame to g2o
        // Vertex part
        // Vertex only needs to set id
        g2o::VertexSE3 *v = new g2o::VertexSE3();
        v->setId( i );
        v->setEstimate( Eigen::Isometry3d::Identity() );
        globalOptimizer.addVertex(v);
        // Edge part
        g2o::EdgeSE3* edge = new g2o::EdgeSE3();
        // two vertices connecting this edge
        edge->vertices() [0] = globalOptimizer.vertex( i );
        edge->vertices() [1] = globalOptimizer.vertex( i+1 );
        //  information matrix
        Eigen::Matrix<double, 6, 6> information = Eigen::Matrix< double, 6,6 >::Identity();
        // The information matrix is the inverse of the covariance matrix and represents our pre-estimation of the accuracy of the edges
        // Because pose is 6D, the information matrix is a 6*6 matrix, assuming that the estimation accuracy of position and angle are both 0.1
        // and independent of each other
        // Then the covariance is a matrix of 0.01 diagonal, and the information matrix is ​​a matrix of 100
        information(0,0) = information(1,1) = information(2,2) = 100;
        information(3,3) = information(4,4) = information(5,5) = 100;
        // You can also set the angle larger, which means that the angle estimation is more accurate
        edge->setInformation( information );
        // The edge estimation is the result of pnp solution
        edge->setMeasurement( T );
        // add ege to graph
        globalOptimizer.addEdge(edge);


    }

    cout<<"optimizing pose graph, vertices: "<<globalOptimizer.vertices().size()<<endl;
    globalOptimizer.initializeOptimization();
    globalOptimizer.optimize( 100 );
    globalOptimizer.save( "./result.g2o" );
    cout<<"Optimization done."<<endl;

    globalOptimizer.clear();
    return 0;


}
