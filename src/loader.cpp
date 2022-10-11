#include <geometry_msgs/TransformStamped.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>

#include "lidar_align/loader.h"
#include "lidar_align/transform.h"

namespace lidar_align
{

Loader::Loader(const Config &config) : config_(config) {}

Loader::Config Loader::getConfig(ros::NodeHandle *nh){
    Loader::Config config;
    nh->param("use_n_scans", config.use_n_scans, config.use_n_scans);
    return config;
}

void Loader::parsePointcloudPcd(const std::string name, LoaderPointcloud *pointcloud) {
    // pcl::PointCloud<pcl::PointXYZI> cloud_in;
    LoaderPointcloud cloud_in;
    if (pcl::io::loadPCDFile(name, cloud_in) < 0)
    {
        PCL_ERROR("\a->点云文件不存在！\n");
        return;
    }

    for (const PointAllFields &raw_point : cloud_in){
        PointAllFields point;
        point.x = raw_point.x;
        point.y = raw_point.y;
        point.z = raw_point.z;
        point.intensity = raw_point.intensity;
        point.time_offset_us = raw_point.time_offset_us;   //

        if (raw_point.reflectivity == 0)
            continue;

        pointcloud->push_back(point);
    }
    // PCLHeader stamp: The value represents microseconds since 1970-01-01 00:00:00 (the UNIX epoch)
    // string substr (size_t pos = 0, size_t len = npos) const;
    // int len = name.size();
    // std::size_t pos = name.find(".pcd");
    // std::string t_ = name.substr(pos-19, 19); // 19: ns time length, 3: ns-us time length
    std::string t_ = name.substr(name.length()-20, 16); // 16: ns time length, 3: ns-us time length
    pointcloud->header.stamp = std::stoll(t_);  //  / 1000ull;   //pcl_stamp = stamp.toNSec() / 1000ull;
    pointcloud->header.seq = seq_;
    pointcloud->header.frame_id = "os1";
    seq_++;  
    std::cout << "The " << name << " of time: " << pointcloud->header.stamp << " load " << cloud_in.points.size() << " points" << std::endl;// '\r' << std::flush;
}

void Loader::parsePointcloudMsg(const sensor_msgs::PointCloud2 msg, LoaderPointcloud *pointcloud){
    bool has_timing = false;
    bool has_intensity = false;
    for (const sensor_msgs::PointField &field : msg.fields){
        if (field.name == "time_offset_us"){
            has_timing = true;
        }
        else if (field.name == "intensity"){
            has_intensity = true;
        }
    }

    if (has_timing){
        pcl::fromROSMsg(msg, *pointcloud);
        std::string fn = "/home/one/test_ws/result/t_" + std::to_string(pointcloud->header.stamp) + ".pcd";
        pcl::io::savePCDFileASCII(fn, *pointcloud);        
        return;
    }
    else if (has_intensity){
        Pointcloud raw_pointcloud;
        pcl::fromROSMsg(msg, raw_pointcloud);

        for (const Point &raw_point : raw_pointcloud){
            PointAllFields point;
            point.x = raw_point.x;
            point.y = raw_point.y;
            point.z = raw_point.z;
            point.intensity = raw_point.intensity;

            if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z) || !std::isfinite(point.intensity))
                continue;

            pointcloud->push_back(point);
        }
        pointcloud->header = raw_pointcloud.header;
        std::string fn = "/home/one/test_ws/result/i_" + std::to_string(pointcloud->header.stamp) + ".pcd";
        pcl::io::savePCDFileASCII(fn, *pointcloud);         
    }
    else{
        pcl::PointCloud<pcl::PointXYZ> raw_pointcloud;
        pcl::fromROSMsg(msg, raw_pointcloud);

        for (const pcl::PointXYZ &raw_point : raw_pointcloud){
            PointAllFields point;
            point.x = raw_point.x;
            point.y = raw_point.y;
            point.z = raw_point.z;

            if (!std::isfinite(point.x) || !std::isfinite(point.y) ||!std::isfinite(point.z))
                continue;
            
            pointcloud->push_back(point);
        }
        pointcloud->header = raw_pointcloud.header;
        std::string fn = "/home/one/test_ws/result/n_" + std::to_string(pointcloud->header.stamp) + ".pcd";
        pcl::io::savePCDFileASCII(fn, *pointcloud);         
    }
}

bool Loader::loadPointcloudFromPCD(const std::string &pcd_path, const Scan::Config &scan_config, Lidar *lidar){
    if (!boost::filesystem::exists(pcd_path) && !boost::filesystem::is_directory(pcd_path)) {
        std::cerr << "# ERROR: Cannot find input directory " << pcd_path << "." << std::endl;
        return false;
    }
    // look for pcd in input directory
    std::string fileExtension = ".pcd";
    std::string prefixL = "";
    std::vector<std::string> pcdFilenames;
    boost::filesystem::directory_iterator itr;
    for (boost::filesystem::directory_iterator itr(pcd_path); itr != boost::filesystem::directory_iterator(); ++itr) {
        if (!boost::filesystem::is_regular_file(itr->status())) continue;

        std::string filename = itr->path().filename().string();

        // check if file extension matches
        if (filename.compare(filename.length() - fileExtension.length(), fileExtension.length(), fileExtension) != 0) continue;

        // check if prefix matches
        if (prefixL.empty() || (!prefixL.empty() && (filename.compare(0, prefixL.length(), prefixL) == 0))) {
            pcdFilenames.push_back(itr->path().string());
        }
    }
    if (pcdFilenames.empty()) {
        std::cerr << "# ERROR: No chessboard images found." << std::endl;
        return 1;
    }

    std::sort(pcdFilenames.begin(), pcdFilenames.end());
    // , [](std::string a, std::string b) {
    //   return std::stoi(a.substr(a.length() - 20, 16)) < std::stoi(b.substr(b.length() - 20, 16));
    // });  

    for(auto val : pcdFilenames) {
        LoaderPointcloud pointcloud;
        parsePointcloudPcd(val, &pointcloud);
        lidar->addPointcloud(pointcloud, scan_config);
        if (lidar->getNumberOfScans() >= config_.use_n_scans)       //~ useless by default.
            break;        
    }    
    std::cerr << "# INFO: Total pcd: " << pcdFilenames.size() << std::endl;  
    if (lidar->getTotalPoints() == 0){
        ROS_ERROR_STREAM("No points were loaded, verify that the bag contains populated, messages of type sensor_msgs/PointCloud2");
        return false;
    }         
    return true;
}

bool Loader::loadPointcloudFromROSBag(const std::string &bag_path, const Scan::Config &scan_config, Lidar *lidar){

    rosbag::Bag bag;
    try{
        bag.open(bag_path, rosbag::bagmode::Read);
    }
    catch (rosbag::BagException e){
        ROS_ERROR_STREAM("LOADING BAG FAILED: " << e.what());
        return false;
    }

    std::vector<std::string> types;
    types.push_back(std::string("sensor_msgs/PointCloud2"));
    rosbag::View view(bag, rosbag::TypeQuery(types));

    size_t scan_num = 0;
    for (const rosbag::MessageInstance &m : view){
        std::cout << " Loading scan: \e[1m" << scan_num++ << "\e[0m from ros bag" << '\r' << std::flush;
        LoaderPointcloud pointcloud;
        parsePointcloudMsg(*(m.instantiate<sensor_msgs::PointCloud2>()), &pointcloud);
        lidar->addPointcloud(pointcloud, scan_config);
        if (lidar->getNumberOfScans() >= config_.use_n_scans)       //~ useless by default.
            break;
    }
    if (lidar->getTotalPoints() == 0){
        ROS_ERROR_STREAM("No points were loaded, verify that the bag contains populated, messages of type sensor_msgs/PointCloud2");
        return false;
    }
    return true;
}

bool Loader::loadTformFromROSBag(const std::string &bag_path, Odom *odom){
    std::ofstream of_;
    //判断是否打开
    of_.open("/home/one/test_ws/result/pose.csv", std::ios::in|std::ios::out|std::ios::trunc);

    rosbag::Bag bag;
    try{
        bag.open(bag_path, rosbag::bagmode::Read);
    }
    catch (rosbag::BagException e){
        ROS_ERROR_STREAM("LOADING BAG FAILED: " << e.what());
        return false;
    }

    std::vector<std::string> types;
    types.push_back(std::string("geometry_msgs/TransformStamped"));
    rosbag::View view(bag, rosbag::TypeQuery(types));

    size_t tform_num = 0;
    for (const rosbag::MessageInstance &m : view){
        std::cout << " Loading transform: \e[1m" << tform_num++ << "\e[0m from ros bag" << '\r' << std::flush;
        geometry_msgs::TransformStamped transform_msg = *(m.instantiate<geometry_msgs::TransformStamped>());
        Timestamp stamp = transform_msg.header.stamp.sec * 1000000ll + transform_msg.header.stamp.nsec / 1000ll;

        Transform T(Transform::Translation(transform_msg.transform.translation.x, transform_msg.transform.translation.y, transform_msg.transform.translation.z),
                    Transform::Rotation(transform_msg.transform.rotation.w, transform_msg.transform.rotation.x, transform_msg.transform.rotation.y, transform_msg.transform.rotation.z));
        odom->addTransformData(stamp, T);

        of_ << stamp << ",0," << transform_msg.transform.translation.x << "," << transform_msg.transform.translation.y << "," << transform_msg.transform.translation.z << "," <<
            transform_msg.transform.rotation.w << "," << transform_msg.transform.rotation.x << "," << transform_msg.transform.rotation.y << "," << transform_msg.transform.rotation.z << "\n";
    }

    if (odom->empty()){
        ROS_ERROR_STREAM("No odom messages found!");
        return false;
    }

    of_.close();
    return true;
}

bool Loader::loadTformFromMaplabCSV(const std::string &csv_path, Odom *odom)
{
    std::ifstream file(csv_path, std::ifstream::in);

    size_t tform_num = 0;
    while (file.peek() != EOF)
    {

        Timestamp stamp;
        Transform T;

        if (getNextCSVTransform(file, &stamp, &T))
        {
            odom->addTransformData(stamp, T);
        }
    }
    return true;
}

// lots of potential failure cases not checked
bool Loader::getNextCSVTransform(std::istream &str, Timestamp *stamp,
                                    Transform *T)
{
    std::string line;
    std::getline(str, line);

    // ignore comment lines
    if (line[0] == '#')
    {
        return false;
    }

    std::stringstream line_stream(line);
    std::string cell;

    std::vector<std::string> data;
    while (std::getline(line_stream, cell, ','))
    {
        data.push_back(cell);
    }

    if (data.size() < 9)
    {
        return false;
    }

    constexpr size_t TIME = 0;
    constexpr size_t X = 2;
    constexpr size_t Y = 3;
    constexpr size_t Z = 4;
    constexpr size_t RW = 5;
    constexpr size_t RX = 6;
    constexpr size_t RY = 7;
    constexpr size_t RZ = 8;
    *stamp = std::stoll(data[TIME]);    //  / 1000ll;
    *T = Transform(Transform::Translation(std::stod(data[X]), std::stod(data[Y]), std::stod(data[Z])),
                    Transform::Rotation(std::stod(data[RW]), std::stod(data[RX]), std::stod(data[RY]), std::stod(data[RZ])));

    return true;
}

} // namespace lidar_align
