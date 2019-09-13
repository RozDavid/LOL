cmake_minimum_required(VERSION 2.8.3)
project(tensorflow_ros_cpp)

find_package(catkin REQUIRED)

# variables affecting the search for the tensorflow library
set(FORCE_TF_PIP_SEARCH OFF CACHE BOOL "Whether to search for pip-installed Tensorflow even on systems using C++11 ABI")
set(DISABLE_TF_PIP_SEARCH OFF CACHE BOOL "Whether to skip search for pip-installed Tensorflow")
set(FORCE_TF_BAZEL_SEARCH OFF CACHE BOOL "Whether to search for bazel-compiled Tensorflow even if tensorflow was already found")
set(DISABLE_TF_BAZEL_SEARCH OFF CACHE BOOL "Whether to skip search for bazel-compiled Tensorflow")
set(FORCE_TF_CATKIN_SEARCH OFF CACHE BOOL "Whether to search for tensorflow_catkin even if tensorflow was already found")
set(DISABLE_TF_CATKIN_SEARCH OFF CACHE BOOL "Whether to skip search for tensorflow_catkin")

# variables affecting pip search
set(TF_PYTHON_VERSION 2.7 CACHE STRING "Python version to be used for searching for Tensorflow")
set(TF_PYTHON_LIBRARY "" CACHE STRING "Python development library of the interpreter used for the found tensorflow. Use only when find_package finds wrong version of PythonLibs, which might be the case if you want to use Tensorflow from Python3.")
set(TF_PIP_EXECUTABLE pip${TF_PYTHON_VERSION} CACHE STRING "Path to pip executable used for searching for tensorflow")
set(TF_PIP_DISABLE_SEARCH_FOR_GPU_VERSION OFF CACHE BOOL "If ON, tensorflow-gpu will be ignored during the search")
set(TF_PIP_PATH "" CACHE STRING "Manually defined path to (site|dist)-packages/tensorflow (should not be needed)")

# variables affecting bazel search
set(TF_BAZEL_LIBRARY "${CATKIN_DEVEL_PREFIX}/../libtensorflow_cc.so" CACHE STRING "Path to the bazel-compiled Tensorflow C++ library")
set(TF_BAZEL_SRC_DIR "${CATKIN_DEVEL_PREFIX}/../tensorflow-include-base" CACHE STRING "Path to the Tensorflow sources directory")
set(TF_BAZEL_USE_SYSTEM_PROTOBUF OFF CACHE BOOL "Whether to use system-installed protobuf includes or those distributed with Tensorflow")


message("-- tensorflow_ros_cpp is probing the system")

# check for a modern c++ compiler, since tensorflow needs support for the c++11 standard
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
if(NOT COMPILER_SUPPORTS_CXX11)
	message(FATAL_ERROR "You must use a compiler that supports the c++11 standard, e.g. GCC 4.9.")
endif()

# detect if the system uses C++11 ABI
execute_process(
		COMMAND ${CMAKE_CURRENT_SOURCE_DIR}/scripts/detect_cxx11_abi.sh ${CMAKE_CXX_COMPILER}
		RESULT_VARIABLE DETECT_CXX11_ABI_RESULT
		OUTPUT_QUIET
)
if("${DETECT_CXX11_ABI_RESULT}" STREQUAL "0")
	set(SYSTEM_USES_CXX11_ABI 1 CACHE BOOL "Whether the system uses C++11 ABI")
	message("-- - The system uses C++11 ABI, the use of tensorflow from pip is possible, but problematic; consider compiling tensorflow yourself")
else()
	set(SYSTEM_USES_CXX11_ABI 0 CACHE BOOL "Whether the system uses C++11 ABI")
	message("-- - The system doesn't use C++11 ABI, you can use tensorflow from pip without problems")
endif()

# start the search

set(HAS_TENSORFLOW_GPU 0)
set(TENSORFLOW_FOUND 0)
set(TENSORFLOW_FOUND_BY "NOTFOUND")

set(tensorflow_ros_cpp_INCLUDE_DIRS "")
set(tensorflow_ros_cpp_LIBRARIES "")
set(tensorflow_ros_cpp_DEPENDS "")
set(tensorflow_ros_cpp_CATKIN_DEPENDS "")
set(tensorflow_ros_cpp_TARGETS "")

set(tensorflow_ros_cpp_USES_CXX11_ABI ${SYSTEM_USES_CXX11_ABI})

# 1. PIP

if((NOT ${SYSTEM_USES_CXX11_ABI} OR ${FORCE_TF_PIP_SEARCH}) AND NOT ${DISABLE_TF_PIP_SEARCH})
	message("-- - Trying to find Tensorflow installed by pip")

	include(cmake/detect_tf_pip.cmake)

	# pip-installed tensorflow is always built without C++11 ABI
	if(${TENSORFLOW_FOUND})
		set(tensorflow_ros_cpp_USES_CXX11_ABI 0)
	endif()

else()
	if(${DISABLE_TF_PIP_SEARCH})
		message("-- - Not searching for Tensorflow installed by pip, it is disabled.")
	else()
		message("-- - Not searching for Tensorflow installed by pip since the system uses C++11 ABI. Set FORCE_TF_PIP_SEARCH to force the search.")
	endif()
endif()

# 2. Bazel

if((NOT ${TENSORFLOW_FOUND} OR ${FORCE_TF_BAZEL_SEARCH}) AND NOT ${DISABLE_TF_BAZEL_SEARCH})
	message("-- - Trying to find Tensorflow compiled by bazel")

	include(cmake/detect_tf_bazel.cmake)

else()
	if(${DISABLE_TF_BAZEL_SEARCH})
		message("-- - Not searching for Tensorflow compiled by bazel, it is disabled.")
	else()
		message("-- - Not searching for Tensorflow compiled by bazel since Tensorflow has already been found. Set FORCE_TF_BAZEL_SEARCH to force the search.")
	endif()
endif()

# 3. tensorflow_catkin

if((NOT ${TENSORFLOW_FOUND} OR ${FORCE_TF_CATKIN_SEARCH}) AND NOT ${DISABLE_TF_CATKIN_SEARCH})
	message("-- - Trying to find Tensorflow from tensorflow_catkin")

	include(cmake/detect_tf_catkin.cmake)

else()
	if(${DISABLE_TF_CATKIN_SEARCH})
		message("-- - Not searching for Tensorflow from tensorflow_catkin, it is disabled.")
	else()
		message("-- - Not searching for Tensorflow from tensorflow_catkin since Tensorflow has already been found. Set FORCE_TF_CATKIN_SEARCH to force the search.")
	endif()
endif()

if(NOT ${TENSORFLOW_FOUND})
  message(FATAL_ERROR "Tensorflow was not found")
else()
	message("-- Using Tensorflow library found by ${TENSORFLOW_FOUND_BY}")
endif()

if(${tensorflow_ros_cpp_USES_CXX11_ABI})
	message("-- The found Tensorflow library uses C++11 ABI.")
else()
	message("-- The found Tensorflow library uses C++03 ABI.")
endif()

if(${HAS_TENSORFLOW_GPU})
	message("-- The found Tensorflow library is compiled with CUDA support.")
else()
	message("-- The found Tensorflow library is compiled without CUDA support.")
endif()

set(tensorflow_ros_cpp_CMAKE_CXX_FLAGS_PRIVATE "-D_GLIBCXX_USE_CXX11_ABI=${tensorflow_ros_cpp_USES_CXX11_ABI}")

catkin_package(
	INCLUDE_DIRS ${tensorflow_ros_cpp_INCLUDE_DIRS}
	LIBRARIES ${tensorflow_ros_cpp_LIBRARIES}
	DEPENDS ${tensorflow_ros_cpp_DEPENDS}
	CATKIN_DEPENDS ${tensorflow_ros_cpp_CATKIN_DEPENDS}
	CFG_EXTRAS tensorflow-extras.cmake  # to add the c++11 compiler flag
)

add_custom_target(run ALL DEPENDS ${TENSORFLOW_TARGETS})

install(
	FILES ${TENSORFLOW_TARGETS}
	DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)
