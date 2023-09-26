#pragma once
#include "fbg_interrogator/FBGInterrogatorNodeInterface.h"
#include "fbg_interrogator/Interrogator.h"

class FBGDemoPublisher : public FBGInterrogatorNodeInterface
{
    public:
        FBGDemoPublisher(const char* name = "FBGDemo", int num_chs = 3, int num_aas = 4);
        ~FBGDemoPublisher();

        /**
         * Connect to the FBG interrogator
         * 
         */
        bool connect();

        /**
         * Publish whether the interrogator is connected or not
         * 
         */
        void publishConnected();
        
        /**
         * Publish the current raw and processed peaks
         * 
         */
        void publishPeaks();

        
    protected:
        int num_active_areas;

        // Publisher timeers
        rclcpp::TimerBase::SharedPtr pub_peak_timer;
        rclcpp::TimerBase::SharedPtr pub_conn_timer;

        
    private:

        // Callback groups for Multi-threaded execution
        rclcpp::CallbackGroup::SharedPtr pub_cb_grp;
        rclcpp::CallbackGroup::SharedPtr srv_cb_grp;

}; // class FBGDemoPublisher