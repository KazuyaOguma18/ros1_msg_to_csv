#ifndef ROS1_MSG_TO_CSV_MSG_TO_CSV
#define ROS1_MSG_TO_CSV_MSG_TO_CSV

#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>

#include <ros1_msg_to_csv/msgs.hpp>

#include <boost/foreach.hpp>
#define foreach BOOST_FOREACH

namespace msg_to_csv {

std::string get_filename(const std::string& path) {
    auto pos = path.find_last_of("/");
    if (pos != std::string::npos) {
        return path.substr(pos + 1);
    } else {
        return path;
    }
}

std::string replace(const std::string& str, const std::string& t, const std::string& c) {
  std::string out_str = str;
  auto pos = str.find(t);
  auto len = t.length();
  if (pos != std::string::npos) {
    out_str.replace(pos, len, c);
    out_str = replace(out_str, t, c);
  }
  return out_str;
}

class BagToCSV {
 public:
  BagToCSV() {
    namespace rp = ros::param;
    namespace rn = ros::names;
    const std::string bag_path = rp::param<std::string>(rn::append("~", "bag"), "");
    const std::vector<std::string> topics = rp::param<std::vector<std::string>>(rn::append("~", "topics"), {});
    bag.open(bag_path, rosbag::bagmode::Read);

    rosbag::View view(bag, rosbag::TopicQuery(topics));

    // トピック毎にまわす
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
      BaseMsg* msg = createMsg(topic_type);
      if (msg == nullptr) {
        ROS_ERROR_STREAM("Failed to Create Msg '" << topic_type << "' passed..");
        continue;
      }
      msg->init(get_filename(bag.getFileName()) + replace(topic, "/", "-") + ".csv");

      // トピック内のデータをすべて保存
      foreach(rosbag::MessageInstance const m, view) {
        if (m.getTopic() == topic && (topic != "/tf" && topic != "/tf_static")) {
          msg->save(m);
        }

        // /tfの場合，/tf_staticも
        if (topic == "/tf" && (m.getTopic() == "/tf" || m.getTopic() == "/tf_static")) {
          msg->save(m);
        }
      }
    } 

    bag.close();
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