#include <cstdio>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/node.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

#include "Interrogator.h"

using namespace std::chrono_literals;

class FBGInterrogatorNode : public rclcpp::Node
{
public:
    FBGInterrogatorNode(const std::string& name = "FBGInterrogator_sm130"): Node(name)
    {
        
        std::string ip_address = this->declare_parameter("interrogator.ip", "192.168.1.11"); // IP address of the interrogator
        int port = this->declare_parameter("interrogator.port", 1852); // Port of the interrogator connection
        
    } // constructor
    
}; // class: FBGInterrogatorNode


int main(int argc, char ** argv)
{
  (void) argc;
  (void) argv;

  printf("hello world fbg_interrogator package\n");
  return 0;
}
