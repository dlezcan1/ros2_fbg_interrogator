#include "fbg_interrogator/fbg_demo.h"




FBGDemoPublisher::FBGDemoPublisher(const char* name = "fbg_demo",int num_chs = 3, int num_aas = 4) :
    FBGInterrogatorNodeInterface(name, num_chs), num_active_areas(num_aas)
{
    // start the publishers
    raw_pub = this->create_publisher<std_msgs::msg::Float64MultiArray>("sensor/raw", 10);
    prc_pub = this->create_publisher<std_msgs::msg::Float64MultiArray>("sensor/processed", 10);
    connected_pub = this->create_publisher<std_msgs::msg::Bool>("interrogator/connected", 10);

    // start the services TODO


    // start publishing timers


} // FBGDemoPublisher constructor

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

    double base_wl = 1530;
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
    

} //main