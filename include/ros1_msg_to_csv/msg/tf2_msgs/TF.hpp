#ifndef ROS1_MSG_TO_CSV_MSG_MSG_TF2_MSGS_TF_HPP
#define ROS1_MSG_TO_CSV_MSG_MSG_TF2_MSGS_TF_HPP

#include "../msg.hpp"
#include <tf2_msgs/TFMessage.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>


namespace msg_to_csv {

class TF : public BaseMsg {
 public:
  TF() : BaseMsg(), listener(tf_buffer){}
  bool init(const std::string& filename) override {

    ROS_INFO_STREAM("msg_to_csv::TF::init(), filename: " << filename);

    // get paramter : parent frame & child frames
    namespace rp = ros::param;
    namespace rn = ros::names;
    parent_frame = rp::param<std::string>(rn::append("~", "tf/parent_frame"), "world");
    child_frames = rp::param<std::vector<std::string>>(rn::append("~", "tf/child_frames"), {});

    csv = std::ofstream(filename);

    if (!csv.is_open()) {
      ROS_ERROR_STREAM("Failed to open '" << filename << "'");
      return false;
    }

    // set fitst row
    for (const auto& child_frame : child_frames) {
      csv << child_frame + "/header/frame_id" << "," << child_frame + "/header/stamp" << "," << child_frame + "/header/seq" << ",";
      csv << child_frame + "/child_frame_id" << ",";
      csv << child_frame + "/transform/translation/x" << "," << child_frame + "/transform/translation/y" << "," << child_frame + "/transform/translation/z" << ",";
      csv << child_frame + "/transform/rotation/w" << "," << child_frame + "/transform/rotation/x" << "," << child_frame + "/transform/rotation/y" << "," << child_frame + "/transform/rotation/z" << ",";
    }

    csv << std::endl;


    return true;
  }

  void save(rosbag::MessageInstance const m) override {
    static ros::Time last_stamp(0);
    tf2_msgs::TFMessage msg = *m.instantiate<tf2_msgs::TFMessage>();
    // ROS_INFO_STREAM(m.getTopic());
    // if (m.getTopic() == "/tf_static") {
    //   ROS_INFO_STREAM("Hello");
    // }

    // ツリー構築
    const bool is_static = m.getTopic() == "/tf_static";
    for (const auto& transform : msg.transforms) {
      // if (m.getTopic() == "/tf_static") {
      //   ROS_INFO_STREAM("parent: " << transform.header.frame_id << ", child: " << transform.child_frame_id);
      // }
      // ROS_INFO_STREAM(transform.header.stamp);
      
      tf_buffer.setTransform(transform, "default_authority", is_static);
    }

    // もし有効な変換が存在すれば記録
    ros::Time stamp(0);

    bool is_transform = false;
    // 一つでもフレームが存在するかどうか
    for (const auto& child_frame : child_frames) {
      if(tf_buffer.canTransform(parent_frame, child_frame, ros::Time(0))) {
        is_transform = true;
      }
    }
    if (!is_transform) return;

    for (const auto& child_frame : child_frames) {
      if(tf_buffer.canTransform(parent_frame, child_frame, ros::Time(0))) {
        geometry_msgs::TransformStamped t;
        t = tf_buffer.lookupTransform(parent_frame, child_frame, ros::Time(0), ros::Duration(1.0));
        // if (abs(last_stamp.toSec() - t.header.stamp.toSec()) < 0.001) {
        //   is_transform = false;
        //   break;
        // }

        csv << t.header.frame_id << "," << t.header.stamp.toNSec() << "," << t.header.seq << ",";
        csv << t.child_frame_id << ",";
        csv << t.transform.translation.x << "," << t.transform.translation.y << "," << t.transform.translation.z << ",";
        csv << t.transform.rotation.w << "," << t.transform.rotation.x << "," << t.transform.rotation.y << "," << t.transform.rotation.z << ",";
        stamp = t.header.stamp;
      }
      else {
        ROS_ERROR_STREAM("Can't find the transform between " << parent_frame << " - " << child_frame);
        csv << ",,,,,,,,,,,";
      }
    }

    last_stamp = stamp == ros::Time(0) ? last_stamp : stamp;

    if (is_transform) csv << std::endl;
  }

  static Registrar<TF> registrar;

  tf2_ros::Buffer tf_buffer;
  tf2_ros::TransformListener listener;

  std::string parent_frame;
  std::vector<std::string> child_frames;
};


Registrar<TF> TF::registrar("tf2_msgs/TFMessage");

} // namespace msg_to_csv
#endif // ROS1_MSG_TO_CSV_MSG_MSG_TF2_MSGS_TF_HPP