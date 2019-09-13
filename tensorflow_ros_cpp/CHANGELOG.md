# 3.1.0
- Added exported CMake variable `tensorflow_ros_cpp_USES_CXX11_ABI`
- Added exported CMake variable `tensorflow_ros_cpp_CMAKE_CXX_FLAGS_PRIVATE`
- Added a few CMake messages to sum up properties of the found TF library

# 3.0.1
- Removed dependency on `swig`. It should be handled upstream by `tensorflow_catkin`.
- Added a note about `rosdep --skip-keys` for handling optional dependencies to README.

# 3.0.0
- Renamed package from `tensorflow_ros` to `tensorflow_ros_cpp`. Please, clean your workspace and update references to the package after pulling this version.

# 2.1.0
- Added CI integration, thanks to [Isaac I.Y. Saito](https://github.com/130s).

# 2.0.0
- Big revamp of the code, enabled finding bazel- and tensorflow_catkin-installed Tensorflow, added compatibility tables and more.

# 1.2.1, 1.2.0
- Added support for Tensorflow 1.4.

# 1.1.0
- Added support for Tensorflow 1.1.

# 1.0.0
- Initial version supporting Tensorflow 0.12.1 and 1.0.0.
