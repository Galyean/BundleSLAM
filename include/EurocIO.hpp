#ifndef UTILITY_EUROC_IO_H
#define UTILITY_EUROC_IO_H
#include <string>
#include <map>
#include <fstream>
#include <iomanip>
#include <Eigen/Core>
namespace utility {
class Euroc_io{
  public:
  Euroc_io(){};
  ~Euroc_io(){};

  /**@brief Load all image timestamps and file name to a map container. **/
  static void loadImgs(const std::string&euroc_dir_,std::map<double,std::string> &images) {
    const std::string imgFolder = euroc_dir_ + "/mav0/cam0/data/";
    const std::string imgtsFileName = euroc_dir_ + "/mav0/cam0/data.csv";
    std::ifstream read_img_time(imgtsFileName);
    if (!read_img_time.is_open()) {
      std::cerr << "can not open file: " << imgtsFileName << std::endl;
      return;
    }
    const char *format = "%lld,%s";
    std::string buffer;
    buffer.resize(2048);
    while (std::getline(read_img_time, buffer)) {
      long long time;
      char file_path[512];
      int available_size = std::sscanf(buffer.c_str(), format, &time, file_path);
      if (available_size == 2) {
        images.insert({(double) time / 1e+9, imgFolder + (std::string) file_path});
      }
    }
  }

  /**@brief Load all imu timestamps and imu values to a map container. **/
  static void loadImus(const std::string&euroc_dir_, std::map<double,Eigen::Vector3d > &gyrs,
                       std::map<double,Eigen::Vector3d > &accs) {
    const std::string imuFileName = euroc_dir_ + "/mav0/imu0/data.csv";
    std::ifstream read_imus(imuFileName);
    if (!read_imus.is_open()) {
      std::cerr << "can not open file: " << imuFileName << std::endl;
      return;
    }
    const char *format = "%lld,%lf,%lf,%lf,%lf,%lf,%lf";
    std::string buffer;
    buffer.resize(2048);
    while (std::getline(read_imus, buffer)) {
      long long time;
      Eigen::Vector3d gyr_data;
      Eigen::Vector3d acc_data;
      int available_size = std::sscanf(buffer.c_str(), format, &time, gyr_data.data(),
                                       gyr_data.data() + 1, gyr_data.data() + 2, acc_data.data(),
                                       acc_data.data() + 1, acc_data.data() + 2);
      if (available_size == 7) {
        gyrs.insert({(double) time / 1e+9, gyr_data});
        accs.insert({(double) time / 1e+9, acc_data});
      }
    }
  }
  /**@brief Check all timestamps. **/
  static void checkTimestamps(const std::map<double,std::string> &images, const std::map<double,Eigen::Vector3d > &gyrs,
                       const std::map<double,Eigen::Vector3d > &accs,const int image_rate, const int imu_rate) {
    double min_image_delta_time = 1.0/image_rate*1.5;
    double min_imu_delta_time = 1.0/imu_rate*1.5;
    auto image_iter = images.begin();
    std::advance(image_iter,1);
    for(auto pre_iter = images.begin();image_iter!=images.end();++image_iter,++pre_iter){
      if(image_iter->first - pre_iter->first> min_image_delta_time)
        std::cout << std::setprecision(9) << "lost image   information between " << pre_iter->first
             << " and " << image_iter->first << std::endl;
    }

    auto imu_iter = gyrs.begin();
    std::advance(imu_iter,1);
    for(auto pre_iter = gyrs.begin();imu_iter!=gyrs.end();++imu_iter,++pre_iter){
      if(imu_iter->first - pre_iter->first> min_imu_delta_time)
        std::cout << std::setprecision(9) << "lost imu   information between " << pre_iter->first
             << " and " << imu_iter->first << std::endl;
    }
  }
  /**@brief syn start time of imu to be consistent with given image id. **/
  static void synTimestamps(std::map<double,std::string> &images, std::map<double,Eigen::Vector3d > &gyrs,
                            std::map<double,Eigen::Vector3d > &accs, const int image_id) {
    if(image_id<0)
      return;
    auto image_start = images.begin();
    std::advance(image_start, image_id);
    images.erase(images.begin(), image_start);
    double image_start_time = images.begin()->first;
    auto acc_start = accs.begin();
    while (acc_start->first < image_start_time)
      acc_start++;
    accs.erase(accs.begin(), acc_start);
    auto gyr_start = gyrs.begin();
    while (gyr_start->first < image_start_time)
      gyr_start++;
    gyrs.erase(gyrs.begin(), gyr_start);
  }
};
}// namespace utility
#endif