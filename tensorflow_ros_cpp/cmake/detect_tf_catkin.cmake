find_package(tensorflow_catkin QUIET)

if(NOT tensorflow_catkin_FOUND)
  message(WARNING "tensorflow_catkin was not found")
  return()
endif()

message("-- -- Found tensorflow_catkin")

# detect if it has GPU support
set(HAS_TENSORFLOW_GPU 0)
foreach(lib IN LISTS tensorflow_catkin_LIBRARIES)
  if(NOT "${lib}" MATCHES "^/")
    find_library(lib ${lib} PATHS ${tensorflow_catkin_LIBRARY_DIRS})
  endif()
  execute_process(
      COMMAND ldd "${lib}"
      OUTPUT_VARIABLE LDD_OUTPUT
  )
  if(${LDD_OUTPUT} MATCHES "libcuda.so")
    set(HAS_TENSORFLOW_GPU 1)
    break()
  endif()
endforeach()

if(${HAS_TENSORFLOW_GPU})
  message("-- -- The Tensorflow library is compiled with CUDA support.")
else()
  message("-- -- The Tensorflow library is compiled without CUDA support.")
endif()

# cuSOLVER from CUDA >= 8.0 requires OpenMP
set(ADDITIONAL_LIBS "")
if(${HAS_TENSORFLOW_GPU})
  if (CMAKE_CXX_COMPILER_ID MATCHES "GNU" )
    find_library(libgomp1_LIBRARIES gomp PATHS ${tensorflow_catkin_LIBRARY_DIRS})
    set(ADDITIONAL_LIBS ${libgomp1_LIBRARIES})
  endif()
endif()

set(TENSORFLOW_FOUND 1)
set(TENSORFLOW_FOUND_BY "tensorflow_catkin")

set(tensorflow_ros_cpp_INCLUDE_DIRS ${tensorflow_catkin_INCLUDE_DIRS})
if(EXISTS ${tensorflow_catkin_INCLUDE_DIRS}/external/nsync/public)
  list(APPEND tensorflow_ros_cpp_INCLUDE_DIRS ${tensorflow_catkin_INCLUDE_DIRS}/external/nsync/public)
elseif(NOT EXISTS ${tensorflow_catkin_INCLUDE_DIRS}/nsync.h)
  message(WARNING "nsync directory not found. This is no problem if you have it system-installed.")
endif()

set(tensorflow_ros_cpp_LIBRARIES ${tensorflow_catkin_LIBRARIES} ${ADDITIONAL_LIBS})
set(tensorflow_ros_cpp_CATKIN_DEPENDS tensorflow_catkin)
if(${HAS_TENSORFLOW_GPU})
  set(tensorflow_ros_cpp_DEPENDS libgomp1)
endif()
