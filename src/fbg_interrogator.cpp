#include "fbg_interrogator/fbg_interrogator.h"

#include <chrono>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executor.hpp>


/* FBGInterrogator Class begin */
FBGInterrogator::FBGInterrogator(const char* name, int num_chs): FBGInterrogatorNodeInterface(name, num_chs)
{
    using namespace std::placeholders;  
    using namespace std::chrono_literals;

    // connect to the interrogator

    // add callback groups
    pub_cb_grp = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    srv_cb_grp = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

    auto pub_cb_opts = rclcpp::PublisherOptions();
    pub_cb_opts.callback_group = pub_cb_grp;

    // start the publishers
    raw_pub       = this->create_publisher<std_msgs::msg::Float64MultiArray>(TOPIC_SENSOR_RAW, 1, pub_cb_opts);
    prc_pub       = this->create_publisher<std_msgs::msg::Float64MultiArray>(TOPIC_SENSOR_PRC, 1, pub_cb_opts);
    connected_pub = this->create_publisher<std_msgs::msg::Bool>(TOPIC_INTERR_CONN, 10, pub_cb_opts);

    // start the services
    calibrate_srv = this->create_service<std_srvs::srv::Trigger>(SERVICE_CALIBRATE, 
                                                                std::bind(&FBGInterrogator::srvSensorCalibrateCallback, this, _1, _2),
                                                                ::rmw_qos_profile_default, srv_cb_grp);
    reconnect_srv = this->create_service<std_srvs::srv::Trigger>(SERVICE_RECONNECT,
                                                                std::bind(&FBGInterrogator::srvReconnectCallback, this, _1, _2),
                                                                ::rmw_qos_profile_default, srv_cb_grp);

    // start publishing timers
    peak_pub_timer = this->create_wall_timer(10ms, std::bind(&FBGInterrogator::publishPeaks, this));
    conn_pub_timer = this->create_wall_timer(50ms, std::bind(&FBGInterrogator::publishConnected, this));

    RCLCPP_INFO(this->get_logger(), "Running FBG demo.");

} // FBGInterrogator constructor

FBGInterrogator::~FBGInterrogator()
{
    RCLCPP_INFO(this->get_logger(), "Disconnecting interrogator.");
    bool disconnected = this->disconnect();

    if (disconnected)
      RCLCPP_INFO(this->get_logger(), "Disconnected interrogator.");
    
    else
      RCLCPP_ERROR(this->get_logger(), "Error disconnecting interrogator");

    RCLCPP_INFO(this->get_logger(), "Shut down node.");

    
} // FBGInterrogator destructor

bool FBGInterrogator::connect()
{
  std::string host;
  unsigned short port = HOST_PORT;

  this->get_parameter(PARAM_INTERROGATOR_IP, host);
  
  return interrogator.connect(host, port);

} // FBGInterrogator::connect

bool FBGInterrogator::disconnect()
{
  interrogator.disconnect();

  return !interrogator.isConnected();
  
} // FBGInterrogator::disconnect

void FBGInterrogator::publishConnected()
{
  std_msgs::msg::Bool msg;
  msg.data = interrogator.isConnected();

  connected_pub->publish(msg);
  
} // FBGInterrogator::publishConnected

void FBGInterrogator::publishPeaks()
{
  if (!interrogator.isConnected())
    return; // we haven't connected the interrogator yet

  // get and publish the raw peaks
  raw_peaks = interrogator.getPeaks();
  raw_pub->publish(peaksToMsg(raw_peaks));

  // process the peaks
  if (calibrated)
  {
    prc_peaks = processPeaks(raw_peaks, ref_peaks, true);
    prc_pub->publish(peaksToMsg(prc_peaks));

  } // if 
  
} // FBGInterrogator::publishPeaks


/* FBGInterrogator Class end */

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);


  rclcpp::shutdown();
  return 0;
}
