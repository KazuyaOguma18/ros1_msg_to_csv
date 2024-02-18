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
  TF() : BaseMsg(), tf_buffer(new tf2_ros::Buffer()), listener(new tf2_ros::TransformListener(*tf_buffer)){}
  bool onInit(const std::string& filename) override {

    ROS_INFO_STREAM("msg_to_csv::TF::init(), filename: " << filename);

    // get paramter : parent frame & child frames
    namespace rp = ros::param;
    namespace rn = ros::names;
    parent_frame = rp::param<std::string>(rn::append("~", "tf/parent_frame"), "world");
    child_frames = rp::param<std::vector<std::string>>(rn::append("~", "tf/child_frames"), {});

    save_tf_ = child_frames.size() > 0;
    if (!save_tf_) return true;


    // set-up csv file
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
    static std::vector<ros::Time> last_stamps(child_frames.size(), ros::Time(0));
    tf2_msgs::TFMessage msg = *m.instantiate<tf2_msgs::TFMessage>();

    // ツリー構築
    const bool is_static = m.getTopic() == "/tf_static";
    for (const auto& transform : msg.transforms) {      
      tf_buffer->setTransform(transform, "default_authority", is_static);
    }

    if (!save_tf_) return;

    // もし有効な変換が存在すれば記録
    ros::Time stamp(0);

    bool is_transform = false;
    // 一つでもフレームが存在するかどうか
    for (size_t i = 0; i < child_frames.size(); i++) {
      if(tf_buffer->canTransform(parent_frame, child_frames[i], ros::Time(0))) {  
        geometry_msgs::TransformStamped t;
        t = tf_buffer->lookupTransform(parent_frame, child_frames[i], ros::Time(0), ros::Duration(1.0));
        if (abs((last_stamps[i] - t.header.stamp).toSec()) < 0.001) {
          continue;
        }          
        is_transform = true;
      }
    }
    if (!is_transform) return;

    for (size_t i = 0; i < child_frames.size(); i++) {
      if(tf_buffer->canTransform(parent_frame, child_frames[i], ros::Time(0))) {
        geometry_msgs::TransformStamped t;
        t = tf_buffer->lookupTransform(parent_frame, child_frames[i], ros::Time(0), ros::Duration(1.0));
        if (abs((last_stamps[i] - t.header.stamp).toSec()) < 0.001) {
          csv << ",,,,,,,,,,,";
          continue;
        }

        csv << t.header.frame_id << "," << t.header.stamp.toNSec() << "," << t.header.seq << ",";
        csv << t.child_frame_id << ",";
        csv << t.transform.translation.x << "," << t.transform.translation.y << "," << t.transform.translation.z << ",";
        csv << t.transform.rotation.w << "," << t.transform.rotation.x << "," << t.transform.rotation.y << "," << t.transform.rotation.z << ",";
        stamp = t.header.stamp;
        last_stamps[i] = stamp == ros::Time(0) ? last_stamps[i] : stamp;
      }
      else {
        ROS_ERROR_STREAM("Couldn't find the transform between " << parent_frame << " - " << child_frames[i]);
        csv << ",,,,,,,,,,,";
      }
    }

    

    if (is_transform) csv << std::endl;
  }

  tf2_ros::Buffer* getTFBuffer() const {
    return tf_buffer;
  }

  static Registrar<TF> registrar;

 private:
  tf2_ros::Buffer* tf_buffer;
  tf2_ros::TransformListener* listener;

  std::string parent_frame;
  std::vector<std::string> child_frames;

  bool save_tf_;
};


Registrar<TF> TF::registrar("tf2_msgs/TFMessage");

} // namespace msg_to_csv
#endif // ROS1_MSG_TO_CSV_MSG_MSG_TF2_MSGS_TF_HPP