#include "fbg_interrogator/FBGInterrogatorNodeInterface.h"

using namespace Eigen;

FBGInterrogatorNodeInterface::FBGInterrogatorNodeInterface(const char* name, int num_chs = 3):
    Node(name), num_channels(num_chs)
{
    // declare the parameters
    this->declare_parameter(PARAM_INTERROGATOR_IP, HOST_IP); // interrogator IP address
    this->declare_parameter(PARAM_SENSOR_NUM_SAMPLES, 200);

} // FBGInterrogatorNodeInterface constructor

PeakContainer FBGInterrogatorNodeInterface::msgToPeaks(const std_msgs::msg::Float64MultiArray& msg)
{
    PeakContainer peaks;
    int dim_idx = 0;
    int ch_size = msg.layout.dim[dim_idx].size;
    auto temp = msg.data.data();
    for (int i = 0; i < msg.data.size(); i++)
    {
        
    }

} // FBGInterrogatorNodeInterface::msgToPeaks


MatrixXd FBGInterrogatorNodeInterface::peaksToMatrixXd(const PeakContainer& peaks, int num_chs, int num_aas)
{
    // unravel the PeakContainer
    Peaks temp;
    for (int i = 0; i < peaks.size(); i++)
        for (double p : peaks[i])
            temp.push_back(p);
    
    auto temp2 = temp.data();
    MatrixXd peaks_mat(temp.data());
    peaks_mat.resize(num_chs, num_aas);
    
    return peaks_mat;
    
} // FBGInterrogatorNodeInterface::peaksToMatrixXd

PeakContainer FBGInterrogatorNodeInterface::matrixXdToPeaks(const MatrixXd& peaks_mat)
{
    PeakContainer peaks;
    for (int i = 0; i < peaks_mat.rows(); i++)
        for (int j = 0; j < peaks_mat.cols(); j++)
            peaks[i].push_back(peaks_mat(i,j));

    return peaks;

} // FBGInterrogatorNodeInterface::matrixXdToPeaks


std_msgs::msg::Float64MultiArray FBGInterrogatorNodeInterface::peaksToMsg(const PeakContainer& peaks)
{
    auto message = std_msgs::msg::Float64MultiArray();
    for (int i = 0; i < peaks.size(); i++)
    {
        // set-up the message layout
        message.layout.dim.push_back(std_msgs::msg::MultiArrayDimension());
        message.layout.dim[i].size = peaks[i].size();
        message.layout.dim[i].stride = 1;
        message.layout.dim[i].label = "CH" + std::to_string(i);
        
        // push back the peak data
        for (double peak : peaks[i])
        {
            message.data.push_back(peak);
        }
    } // for

    return message;
    
} // FBGInterrogatorNodeInterface::peaksToMsg

void FBGInterrogatorNodeInterface::srvReconnectCallback(const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
                                                        const std::shared_ptr<std_srvs::srv::Trigger::Response> res)
{
    RCLCPP_INFO(this->get_logger(), "Reconnecting to interrogator...");
    bool connected = this->connect();
    res->success = connected;

    if (connected)
    {
        res->message = "Reconnection successful.";
        RCLCPP_INFO(this->get_logger(), res->message);
    } // if
    else
    {
        res->message = "Reconnection unsuccessful.";
        RCLCPP_ERROR(this->get_logger(), res->message);
    } // else
    
} // FBGInterrogatorNodeInterface::srvReconnectCallback

PeakContainer FBGInterrogatorNodeInterface::processPeaks(const PeakContainer& peaks, const PeakContainer& ref_peaks, bool temp_comp = false)
{
    PeakContainer proc_peaks;

    // remove the reference peak wavelengths
    for (int i = 0; i < ref_peaks.size(); i++)
    {
        proc_peaks[i].reserve(ref_peaks[i].size()); // allocate memory for the currect amount of wavelengths
        for (int j = 0; j < ref_peaks[i].size(); j++)
        {
            proc_peaks[i].push_back(peaks[i][j] - ref_peaks[i][j]);
        
        } // for
    } // for

    // ensure all channels have the same size
    if (temp_comp)
    {
        
        int _size;
        for (int i = 0; i < proc_peaks.size(); i++)
        {
            if (i == 0)
                _size = proc_peaks[i].size();
            else if (_size != proc_peaks[i].size())
                temp_comp = false;

        }
    }

    // check to perform temperature compensation or not
    if (temp_comp)
    {
        // calculate mean over the active areas
        double mean_aa;
        for (int j = 0; j < proc_peaks[0].size(); j++)
        {   
            mean_aa = 0;
            for (int i = 0; i < proc_peaks.size(); i++)
            {
                mean_aa += proc_peaks[i][j];

            } // for: channels

            // subtract from the current active area
            for (int i = 0; i < proc_peaks.size(); i++)
                proc_peaks[i][j] -= mean_aa;
                

        } // for: active areas

    } // if: temperature compensation

    return proc_peaks;

} // FBGInterrogatorNodeInterface::processPeaks

void FBGInterrogatorNodeInterface::srvSensorCalibrateCallback(const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
                                                              const std::shared_ptr<std_srvs::srv::Trigger::Response> res)
{
    auto temp_node = rclcpp::Node::make_shared("TempSignalSubscriber");
    
    // initalize subsciber parameters
    MatrixXd update_data; // reference data
    int num_aas = -1;
    int counter = 0;
    auto  calibrate = [&update_data, &num_aas, &counter, this](const std_msgs::msg::Float64MultiArray msg){
        if (num_aas <= 0)
        {
            num_aas = msg.layout.dim.size();
            update_data = MatrixXd::Zero(num_channels, num_aas);
        
        } // if

        update_data += peaksToMatrixXd(msgToPeaks(msg), num_channels, num_aas);
        counter++;

    }; // calibrate lambda
    
    // create the subscriber
    auto temp_raw_sub = temp_node->create_subscription<std_msgs::msg::Float64MultiArray>(raw_pub->get_topic_name(), 10, calibrate);
    
    // run the subscriber and perform the calibration
    int num_samples; // number of samples to get
    this->get_parameter(PARAM_SENSOR_NUM_SAMPLES, num_samples);

    while (counter < num_samples)
        rclcpp::spin_some(temp_node);

    update_data /= (float) counter; // perform the averaging
    ref_peaks = matrixXdToPeaks(update_data); // set the reference peaks

    // release the temporary node
    temp_raw_sub.reset();
    temp_node.reset();

    res->message = "Sensors calibrated.";
    res->success = true;
    
} // FBGInterrogatorNodeInterface::srvSensorCalibrateCallback