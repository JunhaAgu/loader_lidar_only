#include "pointcloud_processor.h"

int main(int argc, char* argv[])
{   
    int meter_data = 12;
    // Set Data path
    fs::path executablePath = fs::canonical(fs::path(argv[0]));
    fs::path dataPath = executablePath.parent_path().parent_path();
    std::cout << "executablePath: " << executablePath << std::endl;
    std::cout << "dataPath: " << dataPath << std::endl; 
    dataPath /= "/exp_20231016_12m";
    fs::path TPath = dataPath;
    TPath /= "/T_vs.txt";

    // Set Timer
    ComputationTimer timer;

    // Make PointCloudProcessor
    PointCloudProcessor* pcp (new PointCloudProcessor);

    // Insert all of the pointclouds in data folder.
    pcp->InsertPointClouds(dataPath);
    // pcp->CheckPointClouds();

    // Insert T_vs with textfile in data folder.
    pcp->ReadVehicleToSensorTransformation(TPath);
    // pcp->CheckMatrix();

    // Update PointClouds with T_vs (trigger this just once)
    pcp->UpdatePointClouds();

    // choose indices of pointclouds which you want to handle.
    std::vector<int> process_idx; 
    process_idx.push_back(0);

    // Test Remove Redundant Area module
    std::vector<double> xrange;
    std::vector<double> yrange;

    if (meter_data == 12) {
        xrange = {0, 30};
        yrange = {-15, 15};
    } else if (meter_data == 18) {
        xrange = {10, 25};
        yrange = {-10, 4.25};
    } else if (meter_data == 25) {
        xrange = {10, 25};
        yrange = {-10, 4.25};
    }

    timer.Start();
    pcp->RemoveRedundantArea(xrange,yrange,process_idx);
    timer.End();
    std::cerr << "[RemoveRedundantArea] : " <<timer.PrintToSecond() << " second" << std::endl;
    // check the result with pointcloud viewer.
    // pcp->ViewProcessedPointCloud(process_idx[0]);

    // Test Remove Ground Plane module
    timer.Start();
    pcp->RemoveGroundPlane(process_idx,0.1);
    timer.End();
    std::cerr << "[RemoveGroundPlane] : " <<timer.PrintToSecond() << " second" << std::endl;
    // pcp->ViewProcessedPointCloud(process_idx[0]);

    // Test Generate Grid-map
    timer.Start();
    float size_grid = 0.3;
    pcp->GenerateGridMap(process_idx, size_grid);
    timer.End();
    std::cerr << "[RemoveGroundPlane] : " <<timer.PrintToSecond() << " second" << std::endl;
    pcp->ViewProcessedPointCloud(process_idx[0]);

    // Test Denoise PointCloud
    timer.Start();
    pcp->DenoisePointCloud(process_idx,50,1.0);
    timer.End();
    std::cerr << "[DenoisePointCloud] : " <<timer.PrintToSecond() << " second" << std::endl;
    pcp->ViewProcessedPointCloud(process_idx[0]);
    return 1;
}