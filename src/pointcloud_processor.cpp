#include "pointcloud_processor.h"

PointCloudProcessor::PointCloudProcessor()
{
    // initialize Tvs with identity matrix
    this->Tvs = Mat44::Identity();
}

void PointCloudProcessor::InsertPointClouds(fs::path pc_path)
{
    // Read all of the poinclouds in pc_path
    for (const auto& entry : fs::directory_iterator(pc_path)) {
            // Check if the file has a .pcd extension
            if (entry.path().extension() == ".pcd") {
                // Read and process the PCD file (replace this with your code)
                std::string filePath = entry.path().string();
                std::cout << "Processing PCD file: " << filePath << std::endl;
                pointcloud::Ptr cloud (new pointcloud);
                if(pcl::io::loadPCDFile<pcl::PointXYZ>(filePath,*cloud) == -1)
                    PCL_ERROR("couldn't read pcd file %s.\n",filePath.c_str());
                // save pointcloud in pointcloud vector
                this->pcl_ptr_vec_.push_back(cloud);
            }
        }
    this->pcl_ptr_vec_processed_ = this->pcl_ptr_vec_;
    return;
}

void PointCloudProcessor::CheckPointClouds()
{
    // check the pointcloud vector
    std::cerr << "We have " << this->pcl_ptr_vec_.size() << " pointclouds \n";
    int pc_num = 0;
    for(auto pc : this->pcl_ptr_vec_)
    {
        std::cerr << pc_num << "th pointcloud : " << YELLOW << pc->points.size() << " points" << RESET << std::endl;
        pc_num++;
    }
}


void PointCloudProcessor::ReadVehicleToSensorTransformation(Mat44 T)
{
    // set calibration matrix
    this->Tvs = T;
}

void PointCloudProcessor::ReadVehicleToSensorTransformation(fs::path T_path)
{
    // Set T_vs with text file which has ',' seperator.
    // input text directory
    std::string filename = T_path.string();
    // initialize matrix
    Mat44 tempT;
    // open txt file and read it
    std::ifstream file(filename);
    if(!file.is_open())
        std::cerr << RED << "Error opening file : " << filename << 
        RESET<< std::endl;

    std::string line;
    const int rows = 4;
    const int cols = 4;
    int row = 0;
    while(std::getline(file,line) && row < rows)
    {
        std::string temp_line = line;
        std::vector<double> temp_value;
        std::stringstream ss(temp_line);
        int col = 0;
        while(getline(ss,line,','))
        {
            tempT(row,col) = std::stod(line);
            col ++;
        }
        row++;
    }
    file.close();

    this->Tvs = tempT;
}

void PointCloudProcessor::CheckMatrix()
{
    std::cerr << this->Tvs << std::endl;
}

void PointCloudProcessor::RemoveRedundantArea(std::vector<double> xrange,
                                              std::vector<double> yrange,
                                              std::vector<int> vidx) {
    // Remove redundant area manually.
    // xrange : 1x2 vector for x-axis range
    // yrange : 1x2 vector for y-axis range
    // vidx : you can choose indices of poitnclouds
    if(xrange.size()!=2 || yrange.size()!=2)
    {
        std::cerr << RED <<"Invalid x, y range vector\n" << RESET;
        return;
    }
    else
    {
        // sort range vector in ascending order.
        std::sort(xrange.begin(),xrange.end());
        std::sort(yrange.begin(),yrange.end());
    }
    // use only valid indices
    std::vector<int> valid_idx = {};
    if(vidx.size() == 0)
    {
        for(int i=0;i<pcl_ptr_vec_.size();i++)
            valid_idx.push_back(i);
    }
    else
    {
        for(int i=0;i<vidx.size();i++)
        {
            if(vidx[i] < pcl_ptr_vec_.size() && vidx[i] >= 0)
                valid_idx.push_back(vidx[i]);
        }
    }

    for(int i = 0; i < valid_idx.size(); i++)
    {
        int idx = valid_idx[i];
        pointcloud::Ptr pc_ptr = pcl_ptr_vec_[idx];
        pointcloud::Ptr pc_ptr_ (new pointcloud);
        // fileter by passthrough filter
        pcl::PassThrough<pcl::PointXYZ> pass;
        pass.setInputCloud(pc_ptr);
        pass.setFilterFieldName("x");
        pass.setFilterLimits((float)xrange[0],(float)xrange[1]);
        pass.filter(*pc_ptr_);

        pass.setInputCloud(pc_ptr_);
        pass.setFilterFieldName("y");
        pass.setFilterLimits((float)yrange[0],(float)yrange[1]);
        pass.filter(*pc_ptr_);

        this->pcl_ptr_vec_processed_[idx] = pc_ptr_;
    }

    return;
}

void PointCloudProcessor::ViewProcessedPointCloud(int i)
{
    // view i th pointclouds.
    // red   --> before process.
    // green --> after process.
    if(i < 0 || i > this->pcl_ptr_vec_.size())
    {
        std::cerr << RED << "It's invalid index" << RESET << std::endl;
        return;
    }
    PointCloudViewer viewer;
    viewer.addPointCloud(this->pcl_ptr_vec_[i],"cloud_before",1.0, 0.0, 0.0, 1.0);
    viewer.addPointCloud(this->pcl_ptr_vec_processed_[i],"cloud_after",0.0,1.0,0.0,2.0);
    viewer.spin();

    return;
}

void PointCloudProcessor::UpdatePointClouds()
{
    // If you put Tvs, update pointcloud for coordinate transform.
    // Do not trigger this again!
    int idx = 0;
    for(auto pc_ptr : pcl_ptr_vec_)
    {
        Eigen::Matrix4f T_vs_f = this->Tvs.cast<float>();
        pcl::transformPointCloud(*pc_ptr,*pc_ptr,T_vs_f);
        this->pcl_ptr_vec_[idx] = pc_ptr;
        idx++;
    }
}

void PointCloudProcessor::RemoveGroundPlane(std::vector<int> vidx,
double thres)
{
    // use only valid indices
    std::vector<int> valid_idx = {};
    if(vidx.size() == 0)
    {
        for(int i=0;i<pcl_ptr_vec_.size();i++)
            valid_idx.push_back(i);
    }
    else
    {
        for(int i=0;i<vidx.size();i++)
        {
            if(vidx[i] < pcl_ptr_vec_.size() && vidx[i] >= 0)
                valid_idx.push_back(vidx[i]);
        }
    }

    // set reference normal vector
    Eigen::Vector3f ground_normal(0.0f,0.0f,1.0f);
    // make segmentation module
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    seg.setOptimizeCoefficients(true);
    
    seg.setModelType(pcl::SACMODEL_PERPENDICULAR_PLANE);
    // a model for determining a plane perpendicular to a user-specified axis, 
    // within a maximum specified angular deviation. The plane coefficients are similar to SACMODEL_PLANE .

    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(10000);
    seg.setAxis(ground_normal);
    seg.setEpsAngle(2.0f * (M_PI/180.0f));
    // make extraction module
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setNegative(true); // we have to eliminate plane
    // threshold for plane extraction.
    std::vector<float> distthres = {(float)thres}; // TODO : make multiple process if we need.
    for(int i=0; i< valid_idx.size(); i++)
    {
        int i_idx = valid_idx.at(i);
        pointcloud::Ptr pcl_ground_removed (new pointcloud);
        pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
        pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
        for(int j=0;j<distthres.size();j++)
        {
            seg.setDistanceThreshold(distthres[j]);
            seg.setInputCloud(pcl_ptr_vec_processed_[i_idx]);
            seg.segment(*inliers, *coefficients);

            extract.setInputCloud(pcl_ptr_vec_processed_[i_idx]);
            extract.setIndices(inliers);
            extract.filter(*pcl_ground_removed);
        }
        pcl_ptr_vec_processed_[i_idx] = pcl_ground_removed;
    }
    return;
}

void PointCloudProcessor::DenoisePointCloud(std::vector<int> vidx,
int K, double K_std)
{   // K     : Set the number of nearest neighbors 
    //         to use for mean distance estimation
    // K_std : Set the standard deviation multiplier 
    //         for the distance threshold calculation
    // use only valid indices
    std::vector<int> valid_idx = {};
    if(vidx.size() == 0)
    {
        for(int i=0;i<pcl_ptr_vec_.size();i++)
            valid_idx.push_back(i);
    }
    else
    {
        for(int i=0;i<vidx.size();i++)
        {
            if(vidx[i] < pcl_ptr_vec_.size() && vidx[i] >= 0)
                valid_idx.push_back(vidx[i]);
        }
    }
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    sor.setMeanK(K);
    sor.setStddevMulThresh(K_std);
    for(int i=0; i< valid_idx.size(); i++)
    {
        pointcloud::Ptr cloud_i (new pointcloud);
        sor.setInputCloud(pcl_ptr_vec_processed_[i]);
        sor.filter(*cloud_i);
        pcl_ptr_vec_processed_[i] = cloud_i;
    }

    return;
}