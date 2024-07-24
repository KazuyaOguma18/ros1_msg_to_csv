#ifndef ROS1_MSG_TO_CSV_MSG_STD_MSGS_MULTI_ARRAY_DATA_HPP
#define ROS1_MSG_TO_CSV_MSG_STD_MSGS_MULTI_ARRAY_DATA_HPP

#include "../msg.hpp"
#include <std_msgs/Float64MultiArray.h>
#include <tf2_eigen/tf2_eigen.h>

namespace msg_to_csv { 

template<class Msg>
class MultiArrayDataMsg : public BaseMsg {
 public:
  bool onInit(const std::string& filename) override {

    ROS_INFO_STREAM("msg_to_csv::MultiArrayDataMsg::init()");

    csv = std::ofstream(filename);

    if (!csv.is_open()) {
      ROS_ERROR_STREAM("Failed to open '" << filename << "'");
      return false;
    }

    // tf option
    namespace rp = ros::param;
    namespace rn = ros::names;
    ref_frame_ = rp::param<std::string>(rn::append(BaseMsg::getTopic(), "reference_frame"), "");
    tar_frame_ = rp::param<std::string>(rn::append(BaseMsg::getTopic(), "target_frame"), "");

    BaseMsg::setUseTF(ref_frame_ != "" && tar_frame_ != "");

    return true;
  }

  void save(rosbag::MessageInstance const m) override {
    Msg msg = *m.instantiate<Msg>();

    if (is_first_) {
      // set fitst row
      if (BaseMsg::isUseTF()) {
        csv << "/header/frame_id" << "," << "/header/stamp" << "," << "/header/seq" << ",";
        csv << "/child_frame_id" << ",";
        csv << "/transform/translation/x" << "," << "/transform/translation/y" << "," << "/transform/translation/z" << ",";
        csv << "/transform/rotation/w" << "," << "/transform/rotation/x" << "," << "/transform/rotation/y" << "," << "/transform/rotation/z" << ",";
      }
      for (size_t i = 0; i < msg.data.size(); i++) {
        csv << "data[" + std::to_string(i) + "]" << ",";
      }
      csv << std::endl;
      is_first_ = false;
    }


    if (BaseMsg::isUseTF()) {
      if (BaseMsg::tf_buf->canTransform(ref_frame_, tar_frame_, ros::Time(0))) {
        geometry_msgs::TransformStamped t = BaseMsg::tf_buf->lookupTransform(ref_frame_, tar_frame_, ros::Time(0), ros::Duration(1.0));
        Eigen::Isometry3d iso = tf2::transformToEigen(t);
        if (last_iso_.matrix() == iso.matrix()) return;

        // write TF
        csv << t.header.frame_id << "," << t.header.stamp.toNSec() << "," << t.header.seq << ",";
        csv << t.child_frame_id << ",";
        csv << t.transform.translation.x << "," << t.transform.translation.y << "," << t.transform.translation.z << ",";
        csv << t.transform.rotation.w << "," << t.transform.rotation.x << "," << t.transform.rotation.y << "," << t.transform.rotation.z << ",";
        last_iso_ = iso;
      }
      else {
        ROS_ERROR_STREAM("Couldn't find the transform between " << ref_frame_ << " - " << tar_frame_);
        return;
      }
    }

    for (const auto & data : msg.data) {
      csv << data << ",";
    }

    csv << std::endl;
    
  }


  static Registrar<MultiArrayDataMsg<Msg>> registrar;

 private:
  Eigen::Isometry3d last_iso_;
  bool is_first_ = true;
  std::string tar_frame_;
  std::string ref_frame_;
};

template<>
Registrar<MultiArrayDataMsg<std_msgs::Float64MultiArray>> MultiArrayDataMsg<std_msgs::Float64MultiArray>::registrar("std_msgs/Float64MultiArray");

} // namespace msg_to_csv

#endif // ROS1_MSG_TO_CSV_MSG_STD_MSGS_MULTI_ARRAY_DATA_HPP