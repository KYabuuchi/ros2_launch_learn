#include <rclcpp/rclcpp.hpp>
class Test : public rclcpp::Node
{
public:
  Test() : Node("test")
  {
    auto print = [this](const std::string& label) -> void {
      RCLCPP_INFO_STREAM(get_logger(), label << ": " << declare_parameter<std::string>(label, "default value defined in node"));
    };

    print("before1");
    print("before2");
    print("after1");
    print("after2");
  }
};

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<Test>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}