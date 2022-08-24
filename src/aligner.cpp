#include "lidar_align/aligner.h"

namespace lidar_align
{

Aligner::Aligner(const Config &config) : config_(config){};

Aligner::Config Aligner::getConfig(ros::NodeHandle *nh)
{
    Aligner::Config config;
    nh->param("local", config.local, config.local);
    nh->param("inital_guess", config.inital_guess, config.inital_guess);
    nh->param("max_time_offset", config.max_time_offset, config.max_time_offset);
    nh->param("angular_range", config.angular_range, config.angular_range);
    nh->param("translation_range", config.translation_range,config.translation_range);
    nh->param("max_evals", config.max_evals, config.max_evals);
    nh->param("xtol", config.xtol, config.xtol);
    nh->param("knn_batch_size", config.knn_batch_size, config.knn_batch_size);
    nh->param("knn_k", config.knn_k, config.knn_k);
    nh->param("global_knn_max_dist", config.global_knn_max_dist,config.global_knn_max_dist);
    nh->param("local_knn_max_dist", config.local_knn_max_dist,config.local_knn_max_dist);
    nh->param("time_cal", config.time_cal, config.time_cal);
    nh->param("output_pointcloud_path", config.output_pointcloud_path,config.output_pointcloud_path);
    nh->param("output_calibration_path", config.output_calibration_path,config.output_calibration_path);

    return config;
}

float Aligner::kNNError(const pcl::KdTreeFLANN<Point> &kdtree,
                        const Pointcloud &pointcloud, const size_t k,
                        const float max_dist, const size_t start_idx,
                        const size_t end_idx){

    std::vector<int> kdtree_idx(k);
    std::vector<float> kdtree_dist(k);

    float error = 0;
    for (size_t idx = start_idx; idx < std::min(pointcloud.size(), end_idx); ++idx){
        kdtree.nearestKSearch(pointcloud[idx], k, kdtree_idx, kdtree_dist);
        for (const float &x : kdtree_dist)
            error += std::min(x, max_dist);
    }
    return error;
}

float Aligner::lidarOdomKNNError(const Pointcloud &base_pointcloud, const Pointcloud &combined_pointcloud) const {
    if (!ros::ok())         // kill optimization if node stopped
        throw std::runtime_error("ROS node died, exiting");

    // shared_pointer needed by kdtree, no-op destructor to prevent it trying to clean it up after use
    Pointcloud::ConstPtr combined_pointcloud_ptr(&combined_pointcloud, [](const Pointcloud *) {});
 
    pcl::KdTreeFLANN<Point> kdtree;
    kdtree.setInputCloud(combined_pointcloud_ptr);
    float max_dist = config_.local ? config_.local_knn_max_dist : config_.global_knn_max_dist;  //~ local: 0.1; global: 1.0
    size_t k = config_.knn_k;       //~ knn_k=1;
    // if searching own cloud add one to k as a point will always match to itself
    if (&base_pointcloud == &combined_pointcloud)
        ++k;

    // small amount of threading here to take edge off of bottleneck break knn lookup up into several smaller problems each running in their own thread
    std::vector<std::future<float>> errors;
    for (size_t start_idx = 0; start_idx < base_pointcloud.size(); start_idx += config_.knn_batch_size){
        //~ knn_batch_size: Number of points to send to each thread when finding nearest points (default: 1000);
        size_t end_idx = start_idx + std::min(base_pointcloud.size() - start_idx, static_cast<size_t>(config_.knn_batch_size));
        errors.emplace_back(std::async(std::launch::async, Aligner::kNNError,
                                kdtree, base_pointcloud, k, max_dist, start_idx, end_idx)); //~ params for `kNNError();
    }

    // wait for threads to finish and grab results
    float total_error = 0.0f;
    for (std::future<float> &error : errors){
        total_error += error.get();
    }

    return total_error;
}


float Aligner::lidarOdomKNNError(const Lidar &lidar) const{
    Pointcloud pointcloud;
    lidar.getCombinedPointcloud(&pointcloud);   //~ undistort(by odom interp) transformed(lidar2odom) points in each scan of lidar, 
    return lidarOdomKNNError(pointcloud, pointcloud);
}


//~ TODO: `grad is not used???
double Aligner::LidarOdomMinimizer(const std::vector<double> &x, std::vector<double> &grad, void *f_data){

    OptData *d = static_cast<OptData *>(f_data);    //~ void* can convey any type of pointer, but cannot used for member. Must assign a pointer type.
    if (x.size() > 6)
        d->lidar->setOdomOdomTransforms(*(d->odom), x[6]);  //~ when 'local optimize' with `time_offset, use estimated offset to calculate transform.

    Eigen::Matrix<double, 6, 1> vec;
    vec.setZero();
 
    //~ when "global optimized", state `x only 3 variables, so vec[3:5] from x[rx, ry, rz] (is 'x[3:5]') to avoid OOB.
    const size_t offset = x.size() == 3 ? 3 : 0;
    for (size_t i = offset; i < 6; ++i)         //~ `i in for-loop disables the 'static int i'
        vec[i] = x[i - offset];

    d->lidar->setOdomLidarTransform(Transform::exp(vec.cast<float>())); //~ `exp is user defined.
    double error = d->aligner->lidarOdomKNNError(*(d->lidar));

    static int i = 0;
    std::cout << std::fixed << std::setprecision(2);
    if (x.size() > 3){
        std::cout << " \e[1mx:\e[0m " << std::setw(6) << vec[0];
        std::cout << " \e[1my:\e[0m " << std::setw(6) << vec[1];
        std::cout << " \e[1mz:\e[0m " << std::setw(6) << vec[2];
    }
    std::cout << " \e[1mrx:\e[0m " << std::setw(6) << vec[3];
    std::cout << " \e[1mry:\e[0m " << std::setw(6) << vec[4];
    std::cout << " \e[1mrz:\e[0m " << std::setw(6) << vec[5];
    if (x.size() > 6)
        std::cout << " \e[1mtime:\e[0m " << std::setw(6) << x[6];
    std::cout << " \e[1mError:\e[0m " << std::setw(10) << error;
    std::cout << " \e[1mIteration:\e[0m " << i++ << '\r' << std::flush;

    return error;
}

void Aligner::optimize(const std::vector<double> &lb, const std::vector<double> &ub, OptData *opt_data, std::vector<double> *x){

    nlopt::opt opt;
    if (config_.local)
        opt = nlopt::opt(nlopt::LN_BOBYQA, x->size());      //~ refine: local, [x, y, z, rx, ry, rz]. Then NLOPT find more accurate for all variables.
    else
        opt = nlopt::opt(nlopt::GN_DIRECT_L, x->size());    //~ first time: global, [x,y,z]. NLOPT find global optimized solution for less variables.

    opt.set_lower_bounds(lb);
    opt.set_upper_bounds(ub);
    opt.set_maxeval(config_.max_evals);         //~ iteration max. default: 200.
    opt.set_xtol_abs(config_.xtol);             //~ tolerance. default: 0.0001

    opt.set_min_objective(LidarOdomMinimizer, opt_data);    //~ LidarOdomMinimizer is the objective function.

    double minf;
    std::vector<double> grad;       //~ TODO: `grad is not used???
    nlopt::result result = opt.optimize(*x, minf);          //~ Objective function optimized here!!! `x is calculate iteratively.
    std::cout << "\n ###### optimize result is " << result << std::endl;
    LidarOdomMinimizer(*x, grad, opt_data);
}

std::string Aligner::generateCalibrationString(const Transform &T, const double time_offset){

    Transform::Vector6 T_log = T.log();
    std::stringstream ss;
    ss << "Active Transformation Vector (x,y,z,rx,ry,rz) from the Pose Sensor "
            "Frame to  the Lidar Frame:"
        << std::endl
        << "[";
    ss << T_log[0] << ", ";
    ss << T_log[1] << ", ";
    ss << T_log[2] << ", ";
    ss << T_log[3] << ", ";
    ss << T_log[4] << ", ";
    ss << T_log[5] << "]" << std::endl
        << std::endl;

    ss << "Active Transformation Matrix from the Pose Sensor Frame to  the "
            "Lidar Frame:"
        << std::endl;
    ss << T.matrix() << std::endl
        << std::endl;

    ss << "Active Translation Vector (x,y,z) from the Pose Sensor Frame to  "
            "the Lidar Frame:"
        << std::endl
        << "[";
    ss << T.translation().x() << ", ";
    ss << T.translation().y() << ", ";
    ss << T.translation().z() << "]" << std::endl
        << std::endl;

    ss << "Active Hamiltonen Quaternion (w,x,y,z) the Pose Sensor Frame to  "
            "the Lidar Frame:"
        << std::endl
        << "[";
    ss << T.rotation().w() << ", ";
    ss << T.rotation().x() << ", ";
    ss << T.rotation().y() << ", ";
    ss << T.rotation().z() << "]" << std::endl
        << std::endl;

    if (config_.time_cal)
    {
        ss << "Time offset that must be added to lidar timestamps in seconds:"
            << std::endl
            << time_offset << std::endl
            << std::endl;
    }

    ss << "ROS Static TF Publisher: <node pkg=\"tf\" "
            "type=\"static_transform_publisher\" "
            "name=\"pose_lidar_broadcaster\" args=\"";
    ss << T.translation().x() << " ";
    ss << T.translation().y() << " ";
    ss << T.translation().z() << " ";
    ss << T.rotation().x() << " ";
    ss << T.rotation().y() << " ";
    ss << T.rotation().z() << " ";
    ss << T.rotation().w() << " POSE_FRAME LIDAR_FRAME 100\" />" << std::endl;

    return ss.str();
}

void Aligner::lidarOdomTransform(Lidar *lidar, Odom *odom){

    OptData opt_data;
    opt_data.lidar = lidar;
    opt_data.odom = odom;
    opt_data.aligner = this;
    opt_data.time_cal = config_.time_cal;

    size_t num_params = 6;
    if (config_.time_cal)               //~ calculate time offset-
        ++num_params;

    std::vector<double> x(num_params, 0.0);     //~ state variable: [x, y, z, rx, ry, rz, (ts)];

    if (!config_.local){
        ROS_INFO("Performing Global Optimization...");      //~ globally optimize [x, y, z];
        std::vector<double> lb = {-M_PI_4, -M_PI_4, -M_PI};     //~ lower/upper bound
        std::vector<double> ub = {M_PI_4, M_PI_4, M_PI};
        std::vector<double> global_x(3, 0.0);
        optimize(lb, ub, &opt_data, &global_x);     
        config_.local = true;
        x[3] = global_x[0];
        x[4] = global_x[1];
        x[5] = global_x[2];
    }
    else
        x = config_.inital_guess;

    ROS_INFO("Performing Local Optimization...                                ");
    std::vector<double> lb = {-config_.translation_range, -config_.translation_range, -config_.translation_range,
                                -config_.angular_range, -config_.angular_range, -config_.angular_range};
    std::vector<double> ub = {config_.translation_range, config_.translation_range, config_.translation_range,
                                config_.angular_range, config_.angular_range, config_.angular_range};
    for (size_t i = 0; i < 6; ++i){     //~ added estimated values.
        lb[i] += x[i];
        ub[i] += x[i];
    }
    if (config_.time_cal){
        ub.push_back(config_.max_time_offset);
        lb.push_back(-config_.max_time_offset);
    }

    optimize(lb, ub, &opt_data, &x);
    // calibration brief
    if (!config_.output_pointcloud_path.empty()){
        ROS_INFO("Saving Aligned Pointcloud...");
        lidar->saveCombinedPointcloud(config_.output_pointcloud_path);
    }

    const std::string output_calibration = generateCalibrationString(lidar->getOdomLidarTransform(), x.back());
    if (!config_.output_calibration_path.empty()){
        ROS_INFO("Saving Calibration File...");
        std::ofstream file;
        file.open(config_.output_calibration_path, std::ofstream::out);
        file << output_calibration;
        file.close();
    }
    ROS_INFO("\e[1mFinal Calibration:\e[0m");
    std::cout << output_calibration;
}

} // namespace lidar_align
