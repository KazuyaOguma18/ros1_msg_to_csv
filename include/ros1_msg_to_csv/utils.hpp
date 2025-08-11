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

std::string get_output_directory(const std::string& bag_path) {
  // Get the directory containing the bag file
  auto dir_pos = bag_path.find_last_of("/");
  std::string bag_dir = (dir_pos != std::string::npos) ? bag_path.substr(0, dir_pos) : ".";
  
  // Get the bag filename without extension
  std::string bag_filename = get_filename(bag_path);
  auto ext_pos = bag_filename.find_last_of(".");
  std::string bag_name = (ext_pos != std::string::npos) ? bag_filename.substr(0, ext_pos) : bag_filename;
  
  // Create output directory path: bag_directory/bag_name/
  std::string output_dir = bag_dir + "/" + bag_name;
  
  return output_dir;
}

} // namespace msg_to_csv

#endif // ROS1_MSG_TO_CSV_UTILS_HPP
