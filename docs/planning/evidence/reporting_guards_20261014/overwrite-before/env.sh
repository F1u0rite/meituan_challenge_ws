export ROS_DISTRO=jazzy ROS_DOMAIN_ID=221 RMW_IMPLEMENTATION=fixture
AUBO_WS=/unused
MTC_WS=/unused
ros2() { if [ "$1" = node ]; then echo /fixture_occupied; else echo /unused; fi; }
