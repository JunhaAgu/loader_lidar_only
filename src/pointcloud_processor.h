#pragma once

#include <iostream>
#include <thread>
#include <fstream>
#include <vector>
#include <sstream>
#include <string>
#include <chrono>

#include "experimental/filesystem"

#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/search/kdtree.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/common/geometry.h>
#include <pcl/common/transforms.h>
#include <pcl/features/normal_3d.h>
#include <pcl/point_cloud.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <Eigen/Core>

namespace fs = std::experimental::filesystem;

typedef pcl::PointCloud<pcl::PointXYZ> pointcloud;
typedef std::vector<pointcloud, Eigen::aligned_allocator<pointcloud>> pointcloud_vector;
typedef std::vector<pointcloud::Ptr, Eigen::aligned_allocator<pointcloud::Ptr>> pointcloudptr_vector;

typedef Eigen::Matrix4d Mat44;
typedef Eigen::Matrix3d Mat33;
typedef Eigen::Vector3d Vec3;

#define RESET   "\033[0m"
#define RED     "\033[31m"     // Red text
#define GREEN   "\033[32m"     // Green text
#define YELLOW  "\033[33m"     // Yellow text
#define BLUE    "\033[34m"     // Blue text
#define MAGENTA "\033[35m"     // Magenta text
#define CYAN    "\033[36m"     // Cyan text
#define WHITE   "\033[37m"     // White text

class PointCloudProcessor
{
    public : 
        PointCloudProcessor();
        void InsertPointClouds(fs::path pc_path);
        void ReadVehicleToSensorTransformation(Mat44 T);
        void ReadVehicleToSensorTransformation(fs::path T_path); 
        void CheckPointClouds();
        void RemoveRedundantArea(std::vector<double> xrange,
        std::vector<double> yrange,std::vector<int> vidx);
        void RemoveGroundPlane(std::vector<int> vidx, double thres);
        void DenoisePointCloud(std::vector<int> vidx, int K, double K_std);
        void CheckMatrix();
        void ViewProcessedPointCloud(int i);
        void UpdatePointClouds();

    protected :
        // before processing.
        pointcloudptr_vector pcl_ptr_vec_;
        // after processing.
        pointcloudptr_vector pcl_ptr_vec_processed_;
        // vehicle to sensor transformation
        Mat44 Tvs;

};

class PointCloudViewer {
public:
    PointCloudViewer() {
        viewer_.reset(new pcl::visualization::PCLVisualizer("PointCloud Viewer"));
        viewer_->setBackgroundColor(0.0, 0.0, 0.0);
        viewer_->addCoordinateSystem(1.0);
    }

    void addPointCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, const std::string& name, double r, double g, double b, double point_size) {
        viewer_->addPointCloud(cloud, name);
        viewer_->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, r, g, b, name);
        viewer_->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, point_size, name);
    }

    void spin() {
        while (!viewer_->wasStopped()) {
            viewer_->spinOnce(100);
            std::this_thread::sleep_for(std::chrono::milliseconds(100)); // Sleep to reduce CPU usage
        }
    }

private:
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer_;
};

class ComputationTimer
{
    public:
        ComputationTimer(){
            this->start_time = std::chrono::high_resolution_clock::now();}
        void Start(){
            this->start_time =  std::chrono::high_resolution_clock::now();}
        void End(){
            this->end_time = std::chrono::high_resolution_clock::now();}
        double PrintToSecond(){
            std::chrono::duration<double> duration = end_time-start_time;
            double duration_sec = static_cast<double>(duration.count());
            return duration_sec;
        }
        double PrintToMilliSecond(){
            std::chrono::milliseconds duration = 
            std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
            double duration_msec = static_cast<double>(duration.count());
            return duration_msec;
        }
    protected : 
        std::chrono::high_resolution_clock::time_point start_time;
        std::chrono::high_resolution_clock::time_point end_time;

};