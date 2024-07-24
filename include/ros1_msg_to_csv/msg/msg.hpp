#ifndef ROS1_MSG_TO_CSV_MSG_MSG_HPP
#define ROS1_MSG_TO_CSV_MSG_MSG_HPP

#include <rosbag/bag.h>
#include <fstream>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

namespace msg_to_csv {

class BaseMsg {
 public:
  BaseMsg() {}

  virtual bool onInit(const std::string& filename) = 0;

  bool init(const std::string& filename, const std::string& _topic) {
    topic_ = _topic;
    return onInit(filename);
  }

  virtual void save(rosbag::MessageInstance const m) = 0;

  std::string getTopic() const {
    return topic_;
  }

  void setUseTF(const bool _use_tf) {
    use_tf = _use_tf;
  }

  void setUseTransform(const bool _use_transform) {
    use_transform = _use_transform;
  }

  bool isUseTF() const {
    return use_tf;
  }

  bool isUseTransform() const {
    return use_transform;
  }

  void setTFBuffer(tf2_ros::Buffer* _tf_buf) {
    tf_buf = _tf_buf;
  }

 private:
  std::string topic_;

 protected:
  std::ofstream csv;
  bool use_tf, use_transform;
  tf2_ros::Buffer* tf_buf;
};

std::map<std::string, std::function<BaseMsg*()>> factoryMap;

template<typename T>
class Registrar {
public:
    Registrar(std::string className) {
        factoryMap[className] = []() -> BaseMsg* { return new T(); };
    }
};

} // namespace msg_to_csv

#endif // ROS1_MSG_TO_CSV_MSG_MSG_HPP