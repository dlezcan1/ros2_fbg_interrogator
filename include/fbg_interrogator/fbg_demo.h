#pragma once
#include "fbg_interrogator/FBGInterrogatorNodeInterface.h"

class FBGDemoPublisher : FBGInterrogatorNodeInterface
{
    public:
        FBGDemoPublisher(const char* name="fbg_demo", int num_chs=3, int num_aas=4);

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
       

        
}; // class FBGDemoPublisher