#ifndef ROS1_MSG_TO_CSV_UTILS_HPP
#define ROS1_MSG_TO_CSV_UTILS_HPP

#include <string>

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

} // namespace msg_to_csv

#endif // ROS1_MSG_TO_CSV_UTILS_HPP