#include <cstdio>

// rclcpp
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/node.hpp"

// messages
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "std_msgs/msg/multi_array_dimension.hpp"

// services
#include "std_srvs/srv/trigger.hpp"

// useful dependencies from ROS
#include <Eigen/Dense>

// Custom headers
#include "Interrogator.h"

#define INTERROGATOR_BIND_PUBLISHER(x) std::bind(&FBGInterrogatorNode::x, this)
#define INTERROGATOR_BIND_SERVICE(x) std::bind(&FBGInterrogatorNode::x, this, std::placeholders::_1, std::placeholders::_2)

/* ================ Preamble ========================================================== */
using namespace std::chrono_literals;
using namespace Interrogator;

/* ================ FBG Interrogator Node ============================================= */
class FBGInterrogatorNode : public rclcpp::Node
{
public:
    FBGInterrogatorNode(const std::string& name = "FBGInterrogator_sm130"): Node(name)
    {
        //  declare parameters
        std::string ip_address = this->declare_parameter("interrogator.ip", "192.168.1.11"); // IP address of the interrogator
        int port = this->declare_parameter("interrogator.port", 1852); // Port of the interrogator connection
        m_numSamples = this->declare_parameter("sensor.num_samples", 200); // number of samples to collect
        
        // connect to interrogator
        RCLCPP_INFO(this->get_logger(), "Connecting to interrogator at: %s:%d", ip_address.c_str(), port);
        m_interrogator = std::make_shared<Interrogator::Interrogator>(ip_address, port);
        RCLCPP_INFO(this->get_logger(), "Connected to interrogator at: %s:%d", ip_address.c_str(), port);
        
        // create publishers
        m_pub_signalsRaw = this->create_publisher<std_msgs::msg::Float64MultiArray>("sensor/raw",       10);
        m_pub_signalsRaw = this->create_publisher<std_msgs::msg::Float64MultiArray>("sensor/processed", 10);
        
        // create services
        m_srv_reconnect = this->create_service<std_srvs::srv::Trigger>("interrogator/reconnect", INTERROGATOR_BIND_SERVICE(service_reconnectInterrogator));
        m_srv_calibrate = this->create_service<std_srvs::srv::Trigger>("interrogator/reconnect", INTERROGATOR_BIND_SERVICE(service_calibrateSensors));
        
        // create timers
        m_peakTimer = this->create_wall_timer(1000ms, INTERROGATOR_BIND_PUBLISHER(publish_peaks));
        
    } // constructor
    
    ~FBGInterrogatorNode()
    {
        RCLCPP_INFO(this->get_logger(), "Shutting down node...");
        
    } // destructor
    
    static Interrogator::PeakMessage msgToPeaks(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
    {
        Interrogator::PeakMessage peak_msg;
        
        // extract number of signals
        peak_msg.header.CH1SensorsDetected = msg->layout.dim[0].size / msg->layout.dim[0].stride;
        peak_msg.header.CH2SensorsDetected = msg->layout.dim[1].size / msg->layout.dim[1].stride;
        peak_msg.header.CH3SensorsDetected = msg->layout.dim[2].size / msg->layout.dim[2].stride;
        peak_msg.header.CH4SensorsDetected = msg->layout.dim[3].size / msg->layout.dim[3].stride;
        
        // iterate through the number of signals
        int idx = 0, offset = 0;
        
        // CH 1
        while (idx < peak_msg.header.CH1SensorsDetected + offset)
            peak_msg.peaks[0].push_back( msg->data[idx++] );
        
        offset = peak_msg.header.CH1SensorsDetected;
        
        // CH 2
        while (idx < peak_msg.header.CH2SensorsDetected + offset)
            peak_msg.peaks[1].push_back( msg->data[idx++] );
        
        offset = peak_msg.header.CH2SensorsDetected;
        
        // CH 3
        while (idx < peak_msg.header.CH3SensorsDetected + offset)
            peak_msg.peaks[2].push_back( msg->data[idx++] );
        
        offset = peak_msg.header.CH3SensorsDetected;
        
        // CH 4
        while (idx < peak_msg.header.CH4SensorsDetected + offset)
            peak_msg.peaks[3].push_back( msg->data[idx++] );
        
        offset = peak_msg.header.CH4SensorsDetected;
        
        return peak_msg;
        
    } // msgToPeaks
    
    static std_msgs::msg::Float64MultiArray peaksToMsg(const Interrogator::PeakMessage& peak_msg)
    {
        std_msgs::msg::Float64MultiArray msg;
        
        // CH 1
        for (int i = 0; i < peak_msg.header.CH1SensorsDetected; i++)
            msg.data.push_back(peak_msg.peaks[0][i]);
        
        msg.layout.dim.push_back(std_msgs::msg::MultiArrayDimension());
        msg.layout.dim[0].label = "CH";
        msg.layout.dim[0].size = peak_msg.peaks[0].size();
        msg.layout.dim[0].stride = 1;
        
        // CH 2
        for (int i = 0; i < peak_msg.header.CH2SensorsDetected; i++)
            msg.data.push_back(peak_msg.peaks[1][i]);
        
        msg.layout.dim.push_back(std_msgs::msg::MultiArrayDimension());
        msg.layout.dim[1].label = "CH2";
        msg.layout.dim[1].size = peak_msg.peaks[1].size();
        msg.layout.dim[1].stride = 1;
        
        // CH 3
        for (int i = 0; i < peak_msg.header.CH3SensorsDetected; i++)
            msg.data.push_back(peak_msg.peaks[2][i]);
        
        msg.layout.dim.push_back(std_msgs::msg::MultiArrayDimension());
        msg.layout.dim[2].label = "CH3";
        msg.layout.dim[2].size = peak_msg.peaks[2].size();
        msg.layout.dim[2].stride = 1;
        
        // CH 4
        for (int i = 0; i < peak_msg.header.CH4SensorsDetected; i++)
            msg.data.push_back(peak_msg.peaks[3][i]);
        
        msg.layout.dim.push_back(std_msgs::msg::MultiArrayDimension());
        msg.layout.dim[3].label = "CH4";
        msg.layout.dim[3].size = peak_msg.peaks[3].size();
        msg.layout.dim[3].stride = 1;
        
        
        return msg;
        
    } // peaksToMsg
    
    static Eigen::VectorXd peaksToEigen(const Interrogator::PeakMessage& peak_msg)
    {
        Eigen::VectorXd peaks;
        peaks.resize(  peak_msg.header.CH1SensorsDetected
                     + peak_msg.header.CH2SensorsDetected
                     + peak_msg.header.CH3SensorsDetected
                     + peak_msg.header.CH4SensorsDetected);
        
        // set-up for looping
        int counter = 0, offset = 0;
        
        // CH 1
        while (counter < peak_msg.header.CH1SensorsDetected + offset)
        {
            peaks[counter] = peak_msg.peaks[0][counter - offset];
            counter++;
            
        } // while
        
        offset += peak_msg.header.CH1SensorsDetected;
        
        // CH 2
        while (counter < peak_msg.header.CH2SensorsDetected + offset)
        {
            peaks[counter] = peak_msg.peaks[1][counter - offset];
            counter++;
            
        } // while
        
        offset += peak_msg.header.CH2SensorsDetected;
        
        // CH 3
        while (counter < peak_msg.header.CH3SensorsDetected + offset)
        {
            peaks[counter] = peak_msg.peaks[2][counter - offset];
            counter++;
            
        } // while
        
        offset += peak_msg.header.CH3SensorsDetected;
        
        // CH 4
        while (counter < peak_msg.header.CH4SensorsDetected + offset)
        {
            peaks[counter] = peak_msg.peaks[3][counter - offset];
            counter++;
            
        } // while
        
        offset += peak_msg.header.CH4SensorsDetected;
        
        
        return peaks;
        
    } // peaksToEigen
    
// public functions
private: // functions
    void publish_peaks()
    {
        RCLCPP_INFO(this->get_logger(), "Publishing peaks!");
        
        //  get peaks
        Interrogator::PeakMessage peaks = m_interrogator->getData();
        
        // convert peaks to a Peak Message
        auto msg = peaksToMsg(peaks);
        
        // publish peaks
        m_pub_signalsRaw->publish(msg);
        
    } // publish_peaks
    
    void service_calibrateSensors(std_srvs::srv::Trigger::Request::SharedPtr  req,
                                  std_srvs::srv::Trigger::Response::SharedPtr res)
    {
        // TODO: implement
        RCLCPP_INFO(this->get_logger(), "Starting to recalibrate the sensor wavelengths for %d samples.", m_numSamples);
        
        // set-up peaks
        int counter = 0;
        auto signal_cb = [this, &counter](std_msgs::msg::Float64MultiArray::SharedPtr msg){
            // TODO: implement subscriber callback function
            counter++;
            RCLCPP_INFO(this->get_logger(), "Calibration Counter: %d", counter);
        }; // lambda: signal_cb
        
        // subscriber node for peak collection
        auto tmp_node = std::make_shared<rclcpp::Node>("TempSignalSubscriber");
        auto tmp_sub = tmp_node->create_subscription<std_msgs::msg::Float64MultiArray>("signals/raw", 10, signal_cb);
        
        // set response parameter
        res->success = true;
        RCLCPP_INFO(this->get_logger(), "Calibration successful.");
        
        
    } // service_calibrateSensors
    
    void service_reconnectInterrogator(std_srvs::srv::Trigger::Request::SharedPtr  req,
                                       std_srvs::srv::Trigger::Response::SharedPtr res)
    {
        // get network configuration
        std::string ip_addr;
        int port;
        
        this->get_parameter("interrogator.ip", ip_addr);
        this->get_parameter("interrogator.port", port);
        
        
        // reconnect to interrogator
        RCLCPP_INFO(this->get_logger(), "Reconnecting interrogator to: %s:%d", ip_addr.c_str(), port);
        m_interrogator = std::make_shared<Interrogator::Interrogator>(ip_addr, port);
        RCLCPP_INFO(this->get_logger(), "Connected interrogator to: %s:%d", ip_addr.c_str(), port);
        
        // set response success parameter
        res->success = true;
        
        
    } // service_reconnectInterrogator
    
// private functions
private: // members
    std::shared_ptr<Interrogator::Interrogator> m_interrogator;
    Eigen::VectorXd m_referencePeaks;
    size_t m_numSamples;
    
    // publishers
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr m_pub_signalsRaw,
                                                                   m_pub_signalsProc; // signals
    
    // services
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr m_srv_reconnect, // reconnect interrogator
                                                       m_srv_calibrate; // calibrate sensor signals
    
    // timers
    rclcpp::TimerBase::SharedPtr m_peakTimer;
    
    
// private members
}; // class: FBGInterrogatorNode

/* ==================== main function ============================================== */
int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    
    rclcpp::spin(std::make_shared<FBGInterrogatorNode>());
    
    rclcpp::shutdown();

    printf("hello world fbg_interrogator package\n");
    return 0;
}
