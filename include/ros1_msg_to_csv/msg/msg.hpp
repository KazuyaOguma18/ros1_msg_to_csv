#ifndef ROS1_MSG_TO_CSV_MSG_MSG_HPP
#define ROS1_MSG_TO_CSV_MSG_MSG_HPP

#include <rosbag/bag.h>
#include <fstream>

namespace msg_to_csv {

class BaseMsg {
 public:
  BaseMsg() {}

  virtual bool init(const std::string& filename) = 0;

  virtual void save(rosbag::MessageInstance const m) = 0;

  std::ofstream csv;
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