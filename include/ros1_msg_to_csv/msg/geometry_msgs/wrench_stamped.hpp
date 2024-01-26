#ifndef ROS1_MSG_TO_CSV_MSG_GEOMETRY_MSGS_WRENCH_STAMPED_HPP
#define ROS1_MSG_TO_CSV_MSG_GEOMETRY_MSGS_WRENCH_STAMPED_HPP

#include "../msg.hpp"
#include <geometry_msgs/WrenchStamped.h>

namespace msg_to_csv {

class WrenchStampedMsg : public BaseMsg {
 public:
  bool init(const std::string& filename) override {

    ROS_INFO_STREAM("msg_to_csv::WrenchStampedMsg::init()");

    csv = std::ofstream(filename);

    if (!csv.is_open()) {
      ROS_ERROR_STREAM("Failed to open '" << filename << "'");
      return false;
    }

    // set fitst row
    csv << "/header/frame_id" << "," << "/header/stamp" << "," << "/header/seq" << ",";
    csv << "/wrench/force/x" << "," << "/wrench/force/y" << "," << "/wrench/force/z" << ",";
    csv << "/wrench/torque/x" << "," << "/wrench/torque/y" << "," << "/wrench/torque/z" << ",";
    return true;
  }

  void save(rosbag::MessageInstance const m) override {
    geometry_msgs::WrenchStamped msg = *m.instantiate<geometry_msgs::WrenchStamped>();

    csv << msg.header.frame_id << "," << msg.header.stamp.toNSec() << "," << msg.header.seq << ",";
    csv << msg.wrench.force.x << "," << msg.wrench.force.y << "," << msg.wrench.force.z << ",";
    csv << msg.wrench.torque.x << "," << msg.wrench.torque.y << "," << msg.wrench.torque.z << ",";

    csv << std::endl;
  }


  static Registrar<WrenchStampedMsg> registrar;
};

Registrar<WrenchStampedMsg> WrenchStampedMsg::registrar("gemetry_msgs/WrenchStamped");


} // namespace msg_to_csv

#endif // ROS1_MSG_TO_CSV_MSG_GEOMETRY_MSGS_WRENCH_STAMPED_HPP