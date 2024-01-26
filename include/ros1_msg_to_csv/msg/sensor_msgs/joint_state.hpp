#ifndef ROS1_MSG_TO_CSV_MSG_SENSOR_MSGS_JOINT_STATE_HPP
#define ROS1_MSG_TO_CSV_MSG_SENSOR_MSGS_JOINT_STATE_HPP

#include "../msg.hpp"
#include <sensor_msgs/JointState.h>

namespace msg_to_csv {

class JointStateMsg : public BaseMsg {
 public:
  JointStateMsg() : BaseMsg(), is_first(true) {}

  bool init(const std::string& filename) override {

    ROS_INFO_STREAM("msg_to_csv::JointStateMsg::init(), filename: " << filename);

    csv = std::ofstream(filename);

    if (!csv.is_open()) {
      ROS_ERROR_STREAM("Failed to open '" << filename << "'");
      return false;
    }

    return true;
  }

  void save(rosbag::MessageInstance const m) override {
    sensor_msgs::JointState msg = *m.instantiate<sensor_msgs::JointState>();

    if (is_first) {
      // set fitst row
      csv << "/header/frame_id" << "," << "/header/stamp" << "," << "/header/seq" << ",";

      for (size_t i = 0; i < std::max(
                              std::max(msg.name.size(), msg.position.size()), 
                              std::max(msg.velocity.size(), msg.effort.size())); i++) {
        csv << "/name[" << i << "]" << "," << "/position[" << i << "]" << "," << "/velocity[" << i << "]" << "," << "/effort[" << i << "]" << ",";
      }

      csv << std::endl;
      is_first = false;
    }

    csv << msg.header.frame_id << "," << msg.header.stamp.toNSec() << "," << msg.header.seq << ",";

    for (size_t i = 0; i < std::max(
                            std::max(msg.name.size(), msg.position.size()), 
                            std::max(msg.velocity.size(), msg.effort.size())); i++) {
      if (i < msg.name.size()) {
        csv << msg.name[i];
      }
      csv << ",";
      if (i < msg.position.size()) {
        csv << msg.position[i];
      }
      csv << ",";
      if (i < msg.velocity.size()) {
        csv << msg.velocity[i];
      }
      csv << ",";
      if (i < msg.effort.size()) {
        csv << msg.effort[i];
      }
      csv << ",";
    }
    csv << std::endl;
  }


  static Registrar<JointStateMsg> registrar;

 private:
  bool is_first;
};

Registrar<JointStateMsg> JointStateMsg::registrar("sensor_msgs/JointState");


} // namespace msg_to_csv

#endif // ROS1_MSG_TO_CSV_MSG_SENSOR_MSGS_JOINT_STATE_HPP