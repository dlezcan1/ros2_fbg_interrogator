#pragma once

#define SM_130 // the interrogator version

#include "fbg_interrogator/FBGInterrogatorNodeInterface.h"
#include "fbg_interrogator/interrogator.h"



class FBGInterrogator : public FBGInterrogatorNodeInterface
{
    public:
        FBGInterrogator(const char* name = "FBGInterrogator", int num_chs = 4);
        ~FBGInterrogator();

        /**
         * Connect to the FBG interrogator
         * 
         */
        bool connect();

        /**
         * Disconnect to the FBG interrogator
         * 
         */
        bool disconnect();

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
        rclcpp::TimerBase::SharedPtr peak_pub_timer;
        rclcpp::TimerBase::SharedPtr conn_pub_timer;

        
    private:
        rclcpp::CallbackGroup::SharedPtr pub_cb_grp;
        rclcpp::CallbackGroup::SharedPtr srv_cb_grp;

        Interrogator interrogator;
};