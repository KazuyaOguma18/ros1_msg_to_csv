#include <ros1_msg_to_csv/bag_to_csv.hpp>

int main(int argc, char** argv) {
  ros::init(argc, argv, "msg_to_csv");
  msg_to_csv::BagToCSV();
  return 0;
}