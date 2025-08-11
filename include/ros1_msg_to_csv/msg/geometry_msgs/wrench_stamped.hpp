#ifndef ROS1_MSG_TO_CSV_MSG_GEOMETRY_MSGS_WRENCH_STAMPED_HPP
#define ROS1_MSG_TO_CSV_MSG_GEOMETRY_MSGS_WRENCH_STAMPED_HPP

#include "../msg.hpp"
#include <geometry_msgs/WrenchStamped.h>
#include <tf2_eigen/tf2_eigen.h>
#include "../../utils.hpp"

namespace msg_to_csv {

class WrenchStampedMsg : public BaseMsg {
 public:
  bool onInit(const std::string& filename) override {

    ROS_INFO_STREAM("msg_to_csv::WrenchStampedMsg::init(), filename: " << filename);

    csv = std::ofstream(filename);

    if (!csv.is_open()) {
      ROS_ERROR_STREAM("Failed to open '" << filename << "'");
      return false;
    }
    // tf option
    namespace rp = ros::param;
    namespace rn = ros::names;
    ref_frame_ = rp::param<std::string>(rn::append(BaseMsg::getTopic(), "reference_frame"), "");
    sor_frame_ = rp::param<std::string>(rn::append(BaseMsg::getTopic(), "source_frame"), "");
    BaseMsg::setUseTF(ref_frame_ != "");
    BaseMsg::setUseTransform(sor_frame_ != "");

    // set fitst row
    csv << "/header/frame_id" << "," << "/header/stamp" << "," << "/header/seq" << ",";
    csv << "/wrench/force/x" << "," << "/wrench/force/y" << "," << "/wrench/force/z" << ",";
    csv << "/wrench/torque/x" << "," << "/wrench/torque/y" << "," << "/wrench/torque/z" << ",";

    if (BaseMsg::isUseTF()) {
      csv << "/header/frame_id" << "," << "/header/stamp" << "," << "/header/seq" << ",";
      csv << "/child_frame_id" << ",";
      csv << "/transform/translation/x" << "," << "/transform/translation/y" << "," << "/transform/translation/z" << ",";
      csv << "/transform/rotation/w" << "," << "/transform/rotation/x" << "," << "/transform/rotation/y" << "," << "/transform/rotation/z" << ",";
    }

    csv << std::endl;



    return true;
  }

  void save(rosbag::MessageInstance const m) override {
    geometry_msgs::WrenchStamped msg = *m.instantiate<geometry_msgs::WrenchStamped>();

    Eigen::Vector3d force(msg.wrench.force.x, msg.wrench.force.y, msg.wrench.force.z),
                    torque(msg.wrench.torque.x, msg.wrench.torque.y, msg.wrench.torque.z);
    if (BaseMsg::isUseTransform() && BaseMsg::isUseTF()) {
      if (BaseMsg::tf_buf->canTransform(ref_frame_, sor_frame_, ros::Time(0))) {
        geometry_msgs::TransformStamped t = BaseMsg::tf_buf->lookupTransform(ref_frame_, sor_frame_, ros::Time(0), ros::Duration(1.0));
        Eigen::Isometry3d iso = tf2::transformToEigen(t);
        if (last_iso_.matrix() == iso.matrix()) return;

        // write wrench
        csv << ref_frame_ << "," << msg.header.stamp.toNSec() << "," << msg.header.seq << ",";
        csv << force.x() << "," << force.y() << "," << force.z() << ",";
        csv << torque.x() << "," << torque.y() << "," << torque.z() << ",";      
        // write TF
        csv << t.header.frame_id << "," << t.header.stamp.toNSec() << "," << t.header.seq << ",";
        csv << t.child_frame_id << ",";
        csv << t.transform.translation.x << "," << t.transform.translation.y << "," << t.transform.translation.z << ",";
        csv << t.transform.rotation.w << "," << t.transform.rotation.x << "," << t.transform.rotation.y << "," << t.transform.rotation.z << ",";
        csv << std::endl;

        last_iso_ = iso;
      }
      else {
        ROS_ERROR_STREAM("Couldn't find the transform between " << ref_frame_ << " - " << msg.header.frame_id);
      }  
    }
    else {
      if (BaseMsg::isUseTF()) {
        if (BaseMsg::tf_buf->canTransform(ref_frame_, msg.header.frame_id, ros::Time(0))) {
          geometry_msgs::TransformStamped t = BaseMsg::tf_buf->lookupTransform(ref_frame_, msg.header.frame_id, ros::Time(0), ros::Duration(1.0));
          Eigen::Isometry3d iso = tf2::transformToEigen(t);
          if (last_iso_.matrix() == iso.matrix()) return;

          force = iso.rotation() * force;
          torque = iso.rotation() * torque;
          

          // write wrench
          csv << ref_frame_ << "," << msg.header.stamp.toNSec() << "," << msg.header.seq << ",";
          csv << force.x() << "," << force.y() << "," << force.z() << ",";
          csv << torque.x() << "," << torque.y() << "," << torque.z() << ",";      
          // write TF
          csv << t.header.frame_id << "," << t.header.stamp.toNSec() << "," << t.header.seq << ",";
          csv << t.child_frame_id << ",";
          csv << t.transform.translation.x << "," << t.transform.translation.y << "," << t.transform.translation.z << ",";
          csv << t.transform.rotation.w << "," << t.transform.rotation.x << "," << t.transform.rotation.y << "," << t.transform.rotation.z << ",";
          csv << std::endl;

          last_iso_ = iso;
        }
        else {
          ROS_ERROR_STREAM("Couldn't find the transform between " << ref_frame_ << " - " << msg.header.frame_id);
        }
      }
      else {
        csv << msg.header.frame_id << "," << msg.header.stamp.toNSec() << "," << msg.header.seq << ",";
        csv << force.x() << "," << force.y() << "," << force.z() << ",";
        csv << torque.x() << "," << torque.y() << "," << torque.z() << ",";   
        csv << std::endl;   
      }
    }

  }

  private:
    Eigen::Isometry3d last_iso_;
    std::string ref_frame_, sor_frame_;
    bool use_tf_;

  public: static Registrar<WrenchStampedMsg> registrar;
};

Registrar<WrenchStampedMsg> WrenchStampedMsg::registrar("geometry_msgs/WrenchStamped");


} // namespace msg_to_csv

#endif // ROS1_MSG_TO_CSV_MSG_GEOMETRY_MSGS_WRENCH_STAMPED_HPP