//首先要包含API的头文件
#include <moveit/move_group_interface/move_group_interface.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "ur_moveit_random_demo", ros::init_options::AnonymousName);
  // 创建一个异步的自旋线程（spinning thread）
  ros::AsyncSpinner spinner(1);
  spinner.start();
  // 连接move_group节点中的机械臂实例组，这里的组名manipulator是我们之前在setup assistant中设置的
  moveit::planning_interface::MoveGroupInterface group("manipulator");

  // 随机产生一个目标位置
  group.setRandomTarget();
  // 开始运动规划，并且让机械臂移动到目标位置
  group.move();
  ros::waitForShutdown();
}
