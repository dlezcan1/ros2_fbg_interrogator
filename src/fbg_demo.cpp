#include "fbg_interrogator/fbg_demo.h"

#include <chrono>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executor.hpp>


FBGDemoPublisher::FBGDemoPublisher(const char* name,int num_chs, int num_aas) :
    FBGInterrogatorNodeInterface(name, num_chs), num_active_areas(num_aas)
{
    using namespace std::placeholders;  
    using namespace std::chrono_literals;

    // add callback groups
    pub_cb_grp = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    srv_cb_grp = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

    auto pub_cb_opts = rclcpp::PublisherOptions();
    pub_cb_opts.callback_group = pub_cb_grp;
    // auto srv_cb_opts = rclcpp::Service

    // start the publishers
    raw_pub       = this->create_publisher<std_msgs::msg::Float64MultiArray>(TOPIC_SENSOR_RAW, 1, pub_cb_opts);
    prc_pub       = this->create_publisher<std_msgs::msg::Float64MultiArray>(TOPIC_SENSOR_PRC, 1, pub_cb_opts);
    connected_pub = this->create_publisher<std_msgs::msg::Bool>(TOPIC_INTERR_CONN, 10, pub_cb_opts);

    // start the services
    calibrate_srv = this->create_service<std_srvs::srv::Trigger>(SERVICE_CALIBRATE, 
                                                                std::bind(&FBGDemoPublisher::srvSensorCalibrateCallback, this, _1, _2),
                                                                ::rmw_qos_profile_default, srv_cb_grp);
    reconnect_srv = this->create_service<std_srvs::srv::Trigger>(SERVICE_RECONNECT,
                                                                std::bind(&FBGDemoPublisher::srvReconnectCallback, this, _1, _2),
                                                                ::rmw_qos_profile_default, srv_cb_grp);

    // start publishing timers
    pub_peak_timer  = this->create_wall_timer(10ms, std::bind(&FBGDemoPublisher::publishPeaks, this));
    pub_conn_timer  = this->create_wall_timer(50ms, std::bind(&FBGDemoPublisher::publishConnected, this));

    RCLCPP_INFO(this->get_logger(), "Running FBG demo.");


} // FBGDemoPublisher constructor

FBGDemoPublisher::~FBGDemoPublisher()
{
    RCLCPP_INFO(this->get_logger(), "Shutting down FBG demo.");

} // FBGDemoPublihser destructor

bool FBGDemoPublisher::connect(){
    return true;

} // FBGDemoPublisher::connect

void FBGDemoPublisher::publishConnected()
{
    std_msgs::msg::Bool msg;
    msg.data = true;

    connected_pub->publish(msg);

} // FBGDemoPublisher::publishConnected


void FBGDemoPublisher::publishPeaks()
{
    // Generate artificial wavelengths
    raw_peaks = PeakContainer(); // reset the container

    double base_wl = 1530.0;
    for (int aa = 0; aa < num_active_areas; aa++)
    {
        for (int ch = 0; ch < num_channels; ch++)
        {
            raw_peaks[ch].push_back(base_wl + aa*10 + 1*ch);

        } // for: channels
    } // for: active areas

    // publish raw peaks
    raw_pub->publish(peaksToMsg(raw_peaks));

    // process the peaks
    if (calibrated)
    {
        PeakContainer prc_peaks = processPeaks(raw_peaks, ref_peaks, true);

        prc_pub->publish(peaksToMsg(prc_peaks));

    } // if

} // publishPeaks

int main(int argc, char** argv)
{   
    using namespace std::literals::chrono_literals;
    rclcpp::init(argc, argv);

    rclcpp::executors::MultiThreadedExecutor executor;

    FBGDemoPublisher::SharedPtr fbgdemo_node = std::make_shared<FBGDemoPublisher>();

    executor.add_node(fbgdemo_node);
    
    // rclcpp::spin(std::make_shared<FBGDemoPublisher>());
    
    executor.spin();
    rclcpp::shutdown();
    

    return 0;

} //main