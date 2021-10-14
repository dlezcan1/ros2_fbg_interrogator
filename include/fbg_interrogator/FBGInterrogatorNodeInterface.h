#pragma once
#include <string>
#include <memory>

#include <Eigen/Dense>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_srvs/srv/trigger.hpp>

#include "fbg_interrogator/Interrogator.h"

#define PARAM_INTERROGATOR_IP "interrogator.ip_address"
#define PARAM_SENSOR_NUM_SAMPLES "sensor.num_samples"
#define TOPIC_SENSOR_RAW "sensor/raw"
#define TOPIC_SENSOR_PRC "sensor/processed"
#define TOPIC_INTERR_CONN "interrogator/connected"
#define SERVICE_CALIBRATE "sensor/calibrate"
#define SERVICE_RECONNECT "interrogator/reconnect"

class FBGInterrogatorNodeInterface : public rclcpp::Node
{
    public:
        FBGInterrogatorNodeInterface(const char* name, int num_chs);
        //virtual ~FBGInterrogatorNodeInterface();
        /**
         * Connect to the FBG interrogator
         * 
         */
        virtual bool connect() = 0;

        /**
         * Process the signals 
         * \param peaks: the raw current raw peak values
         * \param ref_peaks: the reference peak values (non-strained)
         * \param temp_comp: whether to perform temperature compensation or not
         * 
         */
        static PeakContainer processPeaks(const PeakContainer& peaks, const PeakContainer& ref_peaks, bool temp_comp = false);

        /**
         * Publish whether the interrogator is connected or not
         * 
         */
        virtual void publishConnected() = 0;

        /**
         * Publish the current raw and processed peaks
         * 
         */
        virtual void publishPeaks() = 0;

        /** 
         * Calibrate the FBG sensor signals with the reference wavelengths
         *  
         *
        virtual void sensorCalibrate() = 0; 
        */


        // PeakContainer conversions
        static std_msgs::msg::Float64MultiArray peaksToMsg(const PeakContainer& peaks);
        static PeakContainer msgToPeaks(const std_msgs::msg::Float64MultiArray& msg);
        static Eigen::MatrixXd peaksToMatrixXd(const PeakContainer& peaks, int num_chs, int num_aas);
        static PeakContainer matrixXdToPeaks(const Eigen::MatrixXd& peaks_mat);
        
    protected:
        // interrogator parameters
        int num_channels;
        int num_peaks = 200; // number of peaks to collect
        
        // peak containers
        bool calibrated = false;
        PeakContainer raw_peaks; // raw peak values
        PeakContainer prc_peaks; // processed peak values
        PeakContainer ref_peaks; // reference peak values

        // sensor publishing
        rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr raw_pub;
        rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr prc_pub;
        rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr connected_pub;

        // services
        rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reconnect_srv;
        rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr calibrate_srv;

        // service callbacks
        void srvSensorCalibrateCallback(const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
                                        const std::shared_ptr<std_srvs::srv::Trigger::Response> res);
        void srvReconnectCallback(const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
                                  const std::shared_ptr<std_srvs::srv::Trigger::Response> res);

}; // class: FBGInterrogatorNodeInterface