#include "pointcloud_processor.h"

PointCloudProcessor::PointCloudProcessor()
{
    // initialize Tvs with identity matrix
    this->Tvs = Mat44::Identity();

    // initialization
    pts_x_.reserve(100000);
    pts_y_.reserve(100000);
    pts_z_.reserve(100000);
    
    x_map_.reserve(100000);
    y_map_.reserve(100000);

    pts_idx_.resize(100000);
    for (int i=0; i<100000; ++i)
    {
        pts_idx_[i].reserve(1000);
    }

    pcl_gridmap_ = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();

    object_row_.reserve(100000);
    object_col_.reserve(100000);
}

PointCloudProcessor::~PointCloudProcessor() {
    // destructor

    // pts_x_.resize(0);
    // pts_y_.resize(0);
    // pts_z_.resize(0);

    // x_map_.resize(0);
    // y_map_.resize(0);

    // pts_idx_.resize(0);

    // END YOUR CODE
};

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

void PointCloudProcessor::GenerateGridMap(std::vector<int> vidx, float size_grid)
{
    // ComputationTimer timer;
    // timer.Start();
    pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_processed = this->pcl_ptr_vec_processed_[0];

    int n_pcl_pts = pcl_processed->size();
    std::cout << "# of pts: " << n_pcl_pts << std::endl;

    pcl::PointXYZ pts_tmp;

    for (int i=0; i<n_pcl_pts; ++i)
    {
        pts_tmp = pcl_processed->points[i];
        pts_x_.push_back(pts_tmp.x);
        pts_y_.push_back(pts_tmp.y);
        pts_z_.push_back(pts_tmp.z);
    }
    float x_max = *max_element(pts_x_.begin(), pts_x_.end());
    float x_min = *min_element(pts_x_.begin(), pts_x_.end());

    float y_max = *max_element(pts_y_.begin(), pts_y_.end());
    float y_min = *min_element(pts_y_.begin(), pts_y_.end());

    std::cout << "x_max: " << x_max << std::endl;
    std::cout << "x_min: " << x_min << std::endl;
     std::cout << "y_max: " << y_max << std::endl;
    std::cout << "y_min: " << y_min << std::endl;

    int n_map_x = (int)ceilf32((x_max-x_min)/size_grid);
    int n_map_y = (int)ceilf32((y_max-y_min)/size_grid);
    std::cout << "n_map_x: " << n_map_x << std::endl;
    std::cout << "n_map_y: " << n_map_y << std::endl;

    float size_grid_accum = 0;
    for (int i=0; i<n_map_x; ++i)
    {
        x_map_.push_back(x_min + size_grid_accum);
        size_grid_accum += size_grid;
    }

    size_grid_accum = 0;
    for (int i=0; i<n_map_y; ++i)
    {
        y_map_.push_back(y_min + size_grid_accum);
        size_grid_accum += size_grid;
    }

    int n_row = y_map_.size();
    int n_col = x_map_.size();

    std::cout << "n_row: " << n_row << std::endl;
    std::cout << "n_col: " << n_col << std::endl;

    cv::Mat grid_z = -0.5*cv::Mat::ones(n_row, n_col,CV_32FC1);
    float* ptr_grid_z = grid_z.ptr<float>(0);

    std::vector<float> x_diff(x_map_.size());
    std::vector<float> y_diff(y_map_.size());
    float* ptr_x_diff = x_diff.data();
    float* ptr_y_diff = y_diff.data();

    float pts_x_tmp = 0.0;
    float pts_y_tmp = 0.0;

    float* ptr_pts_x_ = pts_x_.data();
    float* ptr_pts_y_ = pts_y_.data();
    float* ptr_x_map_ = x_map_.data();
    float* ptr_y_map_ = y_map_.data();

    int idx_x_min = 0;
    int idx_y_min = 0;

    float x_diff_tmp = 0.0;
    float y_diff_tmp = 0.0;
    for (int i=0; i<pts_x_.size(); ++i)
    {
        pts_x_tmp = ptr_pts_x_[i];
        pts_y_tmp = ptr_pts_y_[i];

        for (int j = 0; j < x_map_.size(); ++j) {
            x_diff_tmp = (pts_x_tmp - ptr_x_map_[j]);
            if (x_diff_tmp<0)
            {
                idx_x_min = j;
                break;
            }
        }
        // for (int j = 0; j < x_map_.size(); ++j) {
        //     ptr_x_diff[j] = fabs(pts_x_tmp - ptr_x_map_[j]);
        // }
        // idx_x_min = min_element(x_diff.begin(), x_diff.end()) - x_diff.begin();
        for (int k = 0; k < y_map_.size(); ++k) {
            y_diff_tmp = (pts_y_tmp - ptr_y_map_[k]);
            if (y_diff_tmp<0)
            {
                idx_y_min = k;
                break;
            }
        }
        // for (int k = 0; k < y_map_.size(); ++k) {
        //     ptr_y_diff[k] = fabs(pts_y_tmp - ptr_y_map_[k]);
        // }
        // idx_y_min = min_element(y_diff.begin(), y_diff.end()) - y_diff.begin();

        int i_ncols_j = idx_y_min * n_col + idx_x_min;
        // if (*(ptr_grid_z + i_ncols_j)!=0) 
        // {
            *(ptr_grid_z + i_ncols_j) = pts_z_[i];
            pts_idx_[i_ncols_j].push_back(i);
            // pcl_gridmap_->push_back(pcl::PointXYZ(pts_x_[i], pts_y_[i], pts_z_[i]));
        // }
        // else if (*(ptr_grid_z + i_ncols_j) < pts_z_[i])
        // {
        //     *(ptr_grid_z + i_ncols_j) = pts_z_[i];
        //     pts_idx_[i_ncols_j].push_back(i);
        //     // pcl_gridmap_->push_back(pcl::PointXYZ(pts_x_[i], pts_y_[i], pts_z_[i]));
        // }
    }

    //// remove outlier via bwlabel ////

    cv::Mat bin_grid_z = cv::Mat::zeros(n_row, n_col, CV_8UC1);
    unsigned char* ptr_bin_grid_z = bin_grid_z.ptr<uchar>(0);

    for(int i=0; i<n_row; ++i)
    {
        int i_ncols = i * n_col;
        for(int j=0; j<n_col; ++j)
        {
            if(*(ptr_grid_z + i_ncols + j)>-0.5)
            {
                *(ptr_bin_grid_z + i_ncols + j) = 255;
            }
        }
    }

    // Label objects in 2D image

    cv::Mat object_label = cv::Mat::zeros(n_row, n_col, CV_32SC1);
    int* ptr_object_label = object_label.ptr<int>(0);

    cv::Mat grid_z_valid = cv::Mat::zeros(n_row, n_col, CV_32FC1);
    float* ptr_grid_z_valid = grid_z_valid.ptr<float>(0);

    cv::Mat stats, centroids;
    int n_label = cv::connectedComponentsWithStats(bin_grid_z, object_label,
                                                   stats, centroids, 8);

    if (n_label == 0) {
        return;
    }
    std::cout << n_label <<std::endl;
    int max_n_seg = 0;

    for (int object_idx = 0; object_idx < n_label; ++object_idx) 
    {
        if (object_idx == 0)  // background (major portion)
        {
            continue;
        }
        object_row_.resize(0);
        object_col_.resize(0);

        int obj_left = stats.at<int>(object_idx, cv::CC_STAT_LEFT);
        int obj_top = stats.at<int>(object_idx, cv::CC_STAT_TOP);
        int obj_width = stats.at<int>(object_idx, cv::CC_STAT_WIDTH);
        int obj_height = stats.at<int>(object_idx, cv::CC_STAT_HEIGHT);

        for (int i = obj_top; i < obj_top + obj_height; ++i) {
            int i_ncols = i * n_col;
            for (int j = obj_left; j < obj_left + obj_width; ++j) {
                if (*(ptr_object_label + i_ncols + j) == object_idx) {
                    object_row_.push_back(i);
                    object_col_.push_back(j);
                }
            }
        }

        if (object_row_.size() < 50) 
        {
            continue;
        } 
        if (object_row_.size() < max_n_seg)
        {
            continue;
        }
        max_n_seg = object_row_.size();

        for (int i=0; i<object_row_.size(); ++i)
        {
            int i_valid = object_row_[i];
            for (int j=0; j<object_col_.size(); ++j)
            {
                int j_valid = object_col_[i];
                *(ptr_grid_z_valid + i_valid * n_col + j_valid) = 255;
            }
        }
    }  // end for object_idx

    int cnt = 0;
    for (int i=0; i<n_row; ++i)
    {
        int i_ncols = i * n_col;
        for (int j=0; j<n_col; ++j)
        {
            if (*(ptr_grid_z_valid + i_ncols + j) != 0) 
            {
                int i_ncols_j = i_ncols + j;
                for (int k=0; k<pts_idx_[i_ncols_j].size(); ++k)
                {
                    int pts_idx = pts_idx_[i_ncols_j][k];
                    cnt += 1;       
                    pcl_gridmap_->push_back(pcl::PointXYZ(pts_x_[pts_idx], pts_y_[pts_idx], pts_z_[pts_idx]));
                }
            }
        }
    }
    std::cout << "cnt: " << cnt << std::endl;
    std::cout << "max #: " << max_n_seg << std::endl;

    cv::imshow("grid_z_valid", grid_z_valid);
    // cv::imshow("grid_z", grid_z);
    // cv::waitKey(0);
    // exit(0);

    this->pcl_ptr_vec_processed_[0] = pcl_gridmap_;

    //     timer.End();
    // std::cerr << "[GenerateGridMap] : " <<timer.PrintToSecond() << " second" << std::endl;

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