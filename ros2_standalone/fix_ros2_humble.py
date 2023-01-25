import sys
import os.path as path
import platform

if len(sys.argv) != 2 or platform.system() != "Windows":
    print("""\
This script will apply changes to rclcpp package from https://github.com/ros2/rclcpp/pull/2063
to make it compile with C++20 on Windows.

Usage:  fix_ros2_humble path-to-ros2
""")
    sys.exit(0)

fileToModify = path.join(sys.argv[1], "include\\rclcpp\\rclcpp\\logging.hpp")
print(f"Looking for file: {fileToModify}")

if not path.isfile(fileToModify):
    print("Unable to find file to apply changes. Make sure your ROS2 path is correct.")
    sys.exit(1)

toFind = "::std::is_same<typename std::remove_cv<typename std::remove_reference<decltype(logger)>::type>::type, \\"
toReplace = "::std::is_same<typename std::remove_cv_t<typename std::remove_reference_t<decltype(logger)>>, \\"

with open(fileToModify, "r") as file:
    filedata = file.read()

filedata = filedata.replace(toFind, toReplace)

with open(fileToModify, "w") as file:
    file.write(filedata)

print("Changes has been applied.")
