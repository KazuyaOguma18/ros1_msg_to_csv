#ifndef ROS1_MSG_TO_CSV_MSG_STD_MSGS_SINGLE_DATA_HPP
#define ROS1_MSG_TO_CSV_MSG_STD_MSGS_SINGLE_DATA_HPP

#include "../msg.hpp"
#include <std_msgs/Float64.h>


namespace msg_to_csv { 

template<class Msg>
class SingleDataMsg : public BaseMsg {
 public:
  bool onInit(const std::string& filename) override {

    ROS_INFO_STREAM("msg_to_csv::SingleDataMsg::init()");

    csv = std::ofstream(filename);

    if (!csv.is_open()) {
      ROS_ERROR_STREAM("Failed to open '" << filename << "'");
      return false;
    }

    // set fitst row
    csv << "data" << std::endl;


    return true;
  }

  void save(rosbag::MessageInstance const m) override {
    Msg msg = *m.instantiate<Msg>();
    csv << msg.data << std::endl;
  }

  static Registrar<SingleDataMsg<Msg>> registrar;
};

template<>
Registrar<SingleDataMsg<std_msgs::Float64>> SingleDataMsg<std_msgs::Float64>::registrar("std_msgs/Float64");

} // namespace msg_to_csv

#endif // ROS1_MSG_TO_CSV_MSG_STD_MSGS_SINGLE_DATA_HPP