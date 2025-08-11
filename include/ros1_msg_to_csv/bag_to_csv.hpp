#ifndef ROS1_MSG_TO_CSV_MSG_TO_CSV
#define ROS1_MSG_TO_CSV_MSG_TO_CSV

#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>

#include <ros1_msg_to_csv/msgs.hpp>

#include <boost/foreach.hpp>
#define foreach BOOST_FOREACH

#include <sys/stat.h>
#include <sys/types.h>

#include "utils.hpp"

namespace msg_to_csv {

class BagToCSV {
 public:
  BagToCSV() {
    namespace rp = ros::param;
    namespace rn = ros::names;
    const std::vector<std::string> bag_paths = rp::param<std::vector<std::string>>(rn::append("~", "bags"), {});
    const std::vector<std::string> topics = rp::param<std::vector<std::string>>(rn::append("~", "topics"), {});

    for (const auto& bag_path : bag_paths) {
      ROS_INFO_STREAM("Opening bag : " << bag_path);
      bag.open(bag_path, rosbag::bagmode::Read);

      // Get output directory and create it if it doesn't exist
      std::string output_dir = get_output_directory(bag_path);
      mkdir(output_dir.c_str(), 0755);
      ROS_INFO_STREAM("Saving CSV files to: " << output_dir);

      rosbag::View view(bag, rosbag::TopicQuery(topics));

      std::vector<std::shared_ptr<BaseMsg>> msgs;

      // トピック毎に保存クラスを生成
      for (const auto& topic : topics) {
        // トピックの型を検索＆型の保存クラス呼び出し
        if (topic == "/tf_static") continue;
        std::string topic_type;
        foreach(rosbag::MessageInstance const m, view) {
          if (m.getTopic() == topic) {
            topic_type = m.getDataType();
            break;
          }
        } 
        std::shared_ptr<BaseMsg> msg(createMsg(topic_type));
        if (msg == nullptr) {
          ROS_ERROR_STREAM("Failed to Create Msg '" << topic << "[" << topic_type << "]' passed..");
          continue;
        }
        if (msg->init(output_dir + "/" + get_filename(bag.getFileName()) + replace(topic, "/", "-") + ".csv", topic)) {
          msgs.push_back(msg);
        }

        // tf option
        if (msg->isUseTF()) {
          for (auto tf_msg : msgs) {
            if (tf_msg->getTopic() == "/tf") {
              ROS_INFO_STREAM("setTFBuffer");
              msg->setTFBuffer(
                std::static_pointer_cast<TF>(tf_msg)->getTFBuffer()
              );
            }
          }
        }
      }

      // トピック内のデータをすべて保存
      foreach(rosbag::MessageInstance const m, view) {
        for (auto msg : msgs) {
          if (m.getTopic() == msg->getTopic() && (msg->getTopic() != "/tf" && msg->getTopic() != "/tf_static")) {
            msg->save(m);
          }

          // /tfの場合，/tf_staticも
          if (msg->getTopic() == "/tf" && (m.getTopic() == "/tf" || m.getTopic() == "/tf_static")) {
            msg->save(m);
          }          
        }

      }

      bag.close();
    }
  }

  BaseMsg* createMsg(const std::string& topic_type) {
    auto it = factoryMap.find(topic_type);
    if (it != factoryMap.end()) {
        return it->second();
    }
    return nullptr;      
  }

 private:
  rosbag::Bag bag;

};

} // namespace msg_to_csv

#endif // ROS1_MSG_TO_CSV_MSG_TO_CSV
