#include "LIVMapper.h"

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  options.allow_undeclared_parameters(true);
  options.automatically_declare_parameters_from_overrides(true);

  rclcpp::Node::SharedPtr nh;
  image_transport::ImageTransport it_(nh);
  // 【本地修复】改成堆对象且**不 delete**：run() 返回后如果析构 LIVMapper，
  // 会在退出阶段段错误（实测 exit code -11，通常发生在释放 prior_map / 体素地图时）。
  // 进程马上就结束了，内存交给 OS 回收即可。
  LIVMapper *mapper = new LIVMapper(nh, "laserMapping", options);
  mapper->initializeSubscribersAndPublishers(nh, it_);
  mapper->run(nh);
  rclcpp::shutdown();
  return 0;
}
