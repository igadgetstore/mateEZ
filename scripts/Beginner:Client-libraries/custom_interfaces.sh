#! /bin/bash

#Background
#In a previous tutorial, you learned how to create custom msg and srv interfaces.
#While best practice is to declare interfaces in dedicated interface packages, sometimes it can be convenient to declare, create and use an interface all in one package.
#Recall that interfaces can currently only be defined in CMake packages. It is possible, however, to have Python libraries and nodes in CMake packages (using ament_cmake_python), so you could define interfaces and Python nodes together in one package. We’ll use a CMake package and C++ nodes here for the sake of simplicity.
#This tutorial will focus on the msg interface type, but the steps here are applicable to all interface types.

#1 Create a package
cd Smart-Mobility-Engineering-Lab/ros2_ws/src/ #In your workspace src directory
ros2 pkg create --build-type ament_cmake more_interfaces #create a package more_interfaces
mkdir more_interfaces/msg #make a directory within it for msg files


#2 Create a msg file
#Inside more_interfaces/msg, create a new file AddressBook.msg, and paste the following code to create a message meant to carry information about an individual:
printf "uint8 PHONE_TYPE_HOME=0\nuint8 PHONE_TYPE_WORK=1\nuint8 PHONE_TYPE_MOBILE=2\n\nstring first_name\nstring last_name\nstring phone_number\nuint8 phone_type" >> more_interfaces/msg/AddressBook.msg


#2.1 Build a msg file
#Open package.xml and add the following lines:
nano more_interfaces/package.xml 
#<buildtool_depend>rosidl_default_generators</buildtool_depend>
#
#<exec_depend>rosidl_default_runtime</exec_depend>
#
#<member_of_group>rosidl_interface_packages</member_of_group>

#Open CMakeLists.txt and add the following lines:
#Find the package that generates message code from msg/srv files:
nano more_interfaces/CMakeLists.txt 
#find_package(rosidl_default_generators REQUIRED)

#Declare the list of messages you want to generate:
#set(msg_files
#  "msg/AddressBook.msg"
#)

#By adding the .msg files manually, we make sure that CMake knows when it has to reconfigure the project after you add other .msg files.
#Generate the messages:
#rosidl_generate_interfaces(${PROJECT_NAME}
#  ${msg_files}
#)

#Also make sure you export the message runtime dependency:
#ament_export_dependencies(rosidl_default_runtime)


#3 Use an interface from the same package
#Now we can start writing code that uses this message.

#In more_interfaces/src create a file called publish_address_book.cpp and paste the following code:
nano more_interfaces/src/publish_address_book.cpp
##include <chrono>
##include <memory>
#
##include "rclcpp/rclcpp.hpp"
##include "more_interfaces/msg/address_book.hpp"
#
#using namespace std::chrono_literals;
#
#class AddressBookPublisher : public rclcpp::Node
#{
#public:
#  AddressBookPublisher()
#  : Node("address_book_publisher")
#  {
#    address_book_publisher_ =
#      this->create_publisher<more_interfaces::msg::AddressBook>("address_book", 10);
#
#    auto publish_msg = [this]() -> void {
#        auto message = more_interfaces::msg::AddressBook();
#
#        message.first_name = "John";
#        message.last_name = "Doe";
#        message.phone_number = "1234567890";
#        message.phone_type = message.PHONE_TYPE_MOBILE;
#
#        std::cout << "Publishing Contact\nFirst:" << message.first_name <<
#          "  Last:" << message.last_name << std::endl;
#
#        this->address_book_publisher_->publish(message);
#      };
#    timer_ = this->create_wall_timer(1s, publish_msg);
#  }
#
#private:
#  rclcpp::Publisher<more_interfaces::msg::AddressBook>::SharedPtr address_book_publisher_;
#  rclcpp::TimerBase::SharedPtr timer_;
#};
#
#
#int main(int argc, char * argv[])
#{
#  rclcpp::init(argc, argv);
#  rclcpp::spin(std::make_shared<AddressBookPublisher>());
#  rclcpp::shutdown();
#
#  return 0;
#}


#3.2 Build the publisher
#We need to create a new target for this node in the CMakeLists.txt:
nano more_interfaces/CMakeLists.txt 
#find_package(rclcpp REQUIRED)
#
#add_executable(publish_address_book src/publish_address_book.cpp)
#ament_target_dependencies(publish_address_book rclcpp)
#
#install(TARGETS
#    publish_address_book
#  DESTINATION lib/${PROJECT_NAME})


#3.3 Link against the interface
#In order to use the messages generated in the same package we need to use the following CMake code:
nano more_interfaces/CMakeLists.txt 
#rosidl_get_typesupport_target(cpp_typesupport_target
#  ${PROJECT_NAME} rosidl_typesupport_cpp)
#
#target_link_libraries(publish_address_book "${cpp_typesupport_target}")


#4 Try it out
#Return to the root of the workspace to build the package:
cd ~/Smart-Mobility-Engineering-Lab/ros2_ws/
colcon build --packages-up-to more_interfaces
#Then source the workspace and run the publisher:
gnome-terminal -- bash -c "cd ~/Smart-Mobility-Engineering-Lab/ros2_ws; source install/local_setup.bash; ros2 run more_interfaces publish_address_book; exec bash"
#To confirm the message is being published on the address_book topic, open another terminal, source the workspace, and call topic echo:
gnome-terminal -- bash -c "cd ~/Smart-Mobility-Engineering-Lab/ros2_ws; source install/setup.bash; ros2 topic echo /address_book; exec bash"
