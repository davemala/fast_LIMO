#include "ROSutils.hpp"

#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/int32.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>

// loop closure publishers
rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr loop_pub;

// State obj
fast_limo::State st;

// output frames
std::string world_frame, body_frame;

void state_callback(const nav_msgs::msg::Odometry::SharedPtr msg){

    // Save state
    tf_limo::fromROStoLimo(msg, st);

}

void cloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg){

    pcl::PointCloud<PointType>::Ptr pc_ (boost::make_shared<pcl::PointCloud<PointType>>());
    pcl::fromROSMsg(*msg, *pc_);

    // Update iSAM
    fast_limo::Looper& loop = fast_limo::Looper::getInstance();
    loop.update(st, pc_);

}

void gnss_callback(const sensor_msgs::msg::NavSatFix::SharedPtr msg){

    fast_limo::Looper& loop = fast_limo::Looper::getInstance();
    loop.update(msg->latitude, msg->longitude, msg->altitude);

}

void mySIGhandler(int sig){
    rclcpp::shutdown();
}

void load_config(std::shared_ptr<rclcpp::Node> node, fast_limo::LoopConfig* config){

    node->get_parameter_or("topics.input.GPSfix", config->topics.gnss_fix, std::string("/gps/fix"));

    node->get_parameter_or("ScanContext.NUM_EXCLUDE_RECENT", config->scancontext.NUM_EXCLUDE_RECENT, 50);
    node->get_parameter_or("ScanContext.NUM_CANDIDATES_FROM_TREE", config->scancontext.NUM_CANDIDATES_FROM_TREE, 10);
    node->get_parameter_or("ScanContext.PC_NUM_RING", config->scancontext.PC_NUM_RING, 20);
    node->get_parameter_or("ScanContext.PC_NUM_SECTOR", config->scancontext.PC_NUM_SECTOR, 60);
    node->get_parameter_or("ScanContext.PC_MAX_RADIUS", config->scancontext.PC_MAX_RADIUS, 80.0f);
    node->get_parameter_or("ScanContext.SC_THRESHOLD", config->scancontext.SC_THRESHOLD, 0.2f);
    node->get_parameter_or("ScanContext.SEARCH_RATIO", config->scancontext.SEARCH_RATIO, 0.1f);

    node->get_parameter_or("RadiusSearch.RADIUS", config->radiussearch.RADIUS, 10.0f);
    node->get_parameter_or("RadiusSearch.active", config->radiussearch.active, true);

    node->get_parameter_or("PoseGraph.MinNumStates", config->posegraph.min_num_states, 3);
    node->get_parameter_or("PoseGraph.Covariances.Prior", config->posegraph.prior_cov, std::vector<double>(6, 1.e-12));
    node->get_parameter_or("PoseGraph.Covariances.Odom", config->posegraph.odom_cov, std::vector<double>(6, 1.e-6));
    node->get_parameter_or("PoseGraph.Covariances.GPS", config->posegraph.gnss_cov, std::vector<double>{1.e9, 1.e9, 0.01});
    node->get_parameter_or("PoseGraph.Covariances.Loop", config->posegraph.loop_cov, std::vector<double>(6, 0.5));

    node->get_parameter_or("ICP.MAX_DIST", config->icp.MAX_DIST, 150.0f);
    node->get_parameter_or("ICP.TF_EPSILON", config->icp.TF_EPSILON, 1.e-6f);
    node->get_parameter_or("ICP.EUC_FIT_EPSILON", config->icp.EUC_FIT_EPSILON, 1.e-6);
    node->get_parameter_or("ICP.FIT_SCORE", config->icp.FIT_SCORE, 0.3f);
    node->get_parameter_or("ICP.RANSAC_ITERS", config->icp.RANSAC_ITERS, 0);
    node->get_parameter_or("ICP.WINDOW_SIZE", config->icp.WINDOW_SIZE, 20);
    node->get_parameter_or("ICP.MAX_ITERS", config->icp.MAX_ITERS, 100);

    float diff_norm, diff_yaw;
    node->get_parameter_or("KeyFrames.Odom.DIFF_NORM", diff_norm, 1.5f);
    node->get_parameter_or("KeyFrames.Odom.DIFF_YAW", diff_yaw, 0.5f);
    std::pair<float, float> odom_diff = {diff_norm, diff_yaw};
    config->kf.odom_diff = odom_diff;

    node->get_parameter_or("KeyFrames.GPS.DIFF_NORM", config->kf.gnss_diff, 2.5f);

}

int main(int argc, char** argv) {

    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("fast_limo_looper");

    signal(SIGINT, mySIGhandler); // override default ros sigint signal

    // Declare the one and only Looper object
    fast_limo::Looper& LOOP = fast_limo::Looper::getInstance();

    // Load config
    fast_limo::LoopConfig config;
    load_config(node, &config);

    // Read frames names
    node->get_parameter_or("frames.world", world_frame, std::string("map"));
    node->get_parameter_or("frames.body", body_frame, std::string("base_link"));

    // Define subscribers & publishers
    auto state_sub = node->create_subscription<nav_msgs::msg::Odometry>("/fast_limo/state", 1, state_callback);
    auto pc_sub = node->create_subscription<sensor_msgs::msg::PointCloud2>("/fast_limo/pointcloud", 1, cloud_callback);
    auto gnss_sub = node->create_subscription<sensor_msgs::msg::NavSatFix>("/kitti/oxts/gps/fix", 1, gnss_callback);

    loop_pub = node->create_publisher<nav_msgs::msg::Odometry>("state", 1);

    // Debug
    auto kf_cloud_pub = node->create_publisher<sensor_msgs::msg::PointCloud2>("kf/pointcloud", 1);
    auto sc_pub = node->create_publisher<std_msgs::msg::Float32>("scan_context/result", 1);
    auto sc_idx_pub = node->create_publisher<std_msgs::msg::Int32>("scan_context/index", 1);
    auto st_marker_pub = node->create_publisher<visualization_msgs::msg::Marker>("kf/states", 1);

    auto icp_target_pub = node->create_publisher<sensor_msgs::msg::PointCloud2>("icp/target", 1);
    auto icp_source_pub = node->create_publisher<sensor_msgs::msg::PointCloud2>("icp/source", 1);
    auto icp_result_pub = node->create_publisher<sensor_msgs::msg::PointCloud2>("icp/result", 1);

    LOOP.init(config);

    nav_msgs::msg::Odometry state_msg;
    std_msgs::msg::Float32 sc_msg;
    std_msgs::msg::Int32 sc_idx_msg;

    rclcpp::Rate r(10.0);
    while(rclcpp::ok()){
        rclcpp::spin_some(node);

        if(LOOP.solve()){

            tf_limo::fromLimoToROS(LOOP.get_state(), state_msg);
            loop_pub->publish(state_msg);

            // Debug 
                // Accumulated Key Frames
            std::vector<std::pair<State, 
                        pcl::PointCloud<PointType>::Ptr>> kfs = LOOP.getKeyFrames();

            std::vector<State> state_vec;
            state_vec.reserve(kfs.size());

            pcl::PointCloud<PointType> pc;
            for(int i=0; i<kfs.size(); i++){
                pc += *(kfs[i].second);
                state_vec.push_back(kfs[i].first);
            }

                // Pointcloud map from KFs
            sensor_msgs::msg::PointCloud2 pc_msg;
            pcl::toROSMsg(pc, pc_msg);
            pc_msg.header.stamp = node->now();
            pc_msg.header.frame_id = world_frame;
            kf_cloud_pub->publish(pc_msg);

                // Corrected KF states
            visualization_msgs::msg::Marker st_msg = visualize_limo::getKfMarker(state_vec, world_frame);
            st_marker_pub->publish(st_msg);

                // Scan Context result
            sc_msg.data = LOOP.getScanContextResult();
            sc_pub->publish(sc_msg);

            sc_idx_msg.data = LOOP.getScanContextIndex();
            sc_idx_pub->publish(sc_idx_msg);

                // ICP output
            if(LOOP.hasICPconverged()){
                sensor_msgs::msg::PointCloud2 pc_source_msg;
                pcl::toROSMsg(*LOOP.getICPsource(), pc_source_msg);
                pc_source_msg.header.stamp = node->now();
                pc_source_msg.header.frame_id = world_frame;
                icp_source_pub->publish(pc_source_msg);

                sensor_msgs::msg::PointCloud2 pc_target_msg;
                pcl::toROSMsg(*LOOP.getICPtarget(), pc_target_msg);
                pc_target_msg.header.stamp = node->now();
                pc_target_msg.header.frame_id = world_frame;
                icp_target_pub->publish(pc_target_msg);

                sensor_msgs::msg::PointCloud2 pc_result_msg;
                pcl::toROSMsg(*LOOP.getICPresult(), pc_result_msg);
                pc_result_msg.header.stamp = node->now();
                pc_result_msg.header.frame_id = world_frame;
                icp_result_pub->publish(pc_result_msg);
            }
        }

        r.sleep();
    }

    return 0;
}