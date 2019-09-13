# try finding tensorflow with pip

# find the tensorflow path using some heuristics
set(TENSORFLOW_DIR_FOUND 0)

if(NOT ${TF_PIP_PATH} STREQUAL "")
  if(EXISTS ${TF_PIP_PATH})
    set(TENSORFLOW_PATH "${TF_PIP_PATH}")
    set(TENSORFLOW_DIR_FOUND 1)
    message("-- -- Found user-specified tensorflow pip directory: ${TF_PIP_PATH}")
  else()
    message(WARNING "-- -- User-specified tensorflow pip directory from TF_PIP_PATH not found: ${TF_PIP_PATH}")
  endif()
endif()

# search in pip
if(NOT ${TENSORFLOW_DIR_FOUND})
  execute_process(
      COMMAND ${TF_PIP_EXECUTABLE} -V
      RESULT_VARIABLE PIP_NOT_FOUND
      OUTPUT_VARIABLE PIP_VERSION_OUTPUT
  )
  if(NOT "${PIP_NOT_FOUND}" STREQUAL "0")
    # if pip2.7 is not found, try also pip2
    if(${TF_PIP_EXECUTABLE} MATCHES "pip[0-9]\\.[0-9]$")
      string(REGEX REPLACE "pip([0-9])\\.[0-9]$" "pip\\1" TF_PIP_EXECUTABLE2 ${TF_PIP_EXECUTABLE})
      execute_process(
          COMMAND ${TF_PIP_EXECUTABLE2} -V
          RESULT_VARIABLE PIP_NOT_FOUND
          OUTPUT_VARIABLE PIP_VERSION_OUTPUT
      )
      if("${PIP_NOT_FOUND}" STREQUAL "0")
        set(TF_PIP_EXECUTABLE ${TF_PIP_EXECUTABLE2})
      endif()
    endif()
    if(NOT "${PIP_NOT_FOUND}" STREQUAL "0")
      message(WARNING "${TF_PIP_EXECUTABLE} not found, please install python-pip or python3-pip")
      return()
    endif()
  endif()
  string(REGEX REPLACE "^pip ([0-9.]+) .*$" "\\1" PIP_VERSION ${PIP_VERSION_OUTPUT})
  if(${PIP_VERSION} VERSION_LESS "9.0.0")
    message(WARNING "-- -- ${TF_PIP_EXECUTABLE} version is ${PIP_VERSION}, which is too old for Tensorflow. Run 'sudo ${TF_PIP_EXECUTABLE} install --upgrade pip'.")
    return()
  else()
    message("-- -- Found pip version ${PIP_VERSION}")
  endif()

  set(PIP_DIDNT_FIND_TENSORFLOW 1)
  # try tensorflow-gpu
  if(NOT ${TF_PIP_DISABLE_SEARCH_FOR_GPU_VERSION})
    execute_process(
        COMMAND ${TF_PIP_EXECUTABLE} show tensorflow-gpu
        RESULT_VARIABLE PIP_DIDNT_FIND_TENSORFLOW
        OUTPUT_VARIABLE PIP_SHOW_OUTPUT
    )
  endif()

  # try tensorflow
  if(NOT "${PIP_DIDNT_FIND_TENSORFLOW}" STREQUAL "0")
    execute_process(
        COMMAND ${TF_PIP_EXECUTABLE} show tensorflow
        RESULT_VARIABLE PIP_DIDNT_FIND_TENSORFLOW
        OUTPUT_VARIABLE PIP_SHOW_OUTPUT
    )
  endif()

  if (NOT "${PIP_SHOW_OUTPUT}" STREQUAL "")
    # parse pip output in case it found tensorflow
    string(REGEX MATCH "Location: [^\r\n]*[\r\n]" TENSORFLOW_LOCATION ${PIP_SHOW_OUTPUT})
    string(REGEX REPLACE "^Location: (.*)[\r\n]$" "\\1" TENSORFLOW_LOCATION2 ${TENSORFLOW_LOCATION})
    set(TENSORFLOW_PATH "${TENSORFLOW_LOCATION2}/tensorflow")
    set(TENSORFLOW_DIR_FOUND 1)
    message("-- -- Found pip-installed tensorflow: ${TENSORFLOW_PATH}")
  else()
    # try finding tensorflow on the PYTHON_PACKAGE_PATH and a few other system paths
    find_path(TENSORFLOW_LOCATION tensorflow
        PATHS
          ENV PYTHON_PACKAGE_PATH
          $ENV{HOME}/.local/lib/python${TF_PYTHON_VERSION}/site-packages
          /usr/local/lib/python${TF_PYTHON_VERSION}/dist-packages
          /usr/lib/python${TF_PYTHON_VERSION}/dist-packages
          /usr/local/lib/python${TF_PYTHON_VERSION}/site-packages
          /usr/lib/python${TF_PYTHON_VERSION}/site-packages
        NO_DEFAULT_PATH
        )
    if(NOT ${TENSORFLOW_LOCATION} MATCHES NOTFOUND)
      set(TENSORFLOW_PATH "${TENSORFLOW_LOCATION}/tensorflow")
      set(TENSORFLOW_DIR_FOUND 1)
      message("-- -- Found tensorflow: ${TENSORFLOW_PATH}")
    endif()
  endif()
endif()

if(NOT ${TENSORFLOW_DIR_FOUND})
  message(WARNING "-- -- Tensorflow could not be found via pip. Try installing it manually using 'sudo pip${TF_PYTHON_VERSION} install tensorflow' or 'pip${TF_PYTHON_VERSION} install --user tensorflow', or set the TENSORFLOW_PATH environment variable to point to the *dist-pacakges/tensorflow directory.")
  return()
endif()

# we need to add the `lib` prefix to the library name so that the linker can find it!
# first, test for presence of the Tensorflow 1.1+ library
set(TENSORFLOW_ORIG_LIBRARY ${TENSORFLOW_PATH}/python/_pywrap_tensorflow_internal${CMAKE_SHARED_LIBRARY_SUFFIX})
if(NOT EXISTS ${TENSORFLOW_ORIG_LIBRARY})
  # otherwise, use the filename used formerly
  set(TENSORFLOW_ORIG_LIBRARY ${TENSORFLOW_PATH}/python/_pywrap_tensorflow${CMAKE_SHARED_LIBRARY_SUFFIX})
endif()

if(EXISTS ${TENSORFLOW_ORIG_LIBRARY})
  message("-- -- Using Tensorflow library ${TENSORFLOW_ORIG_LIBRARY}")
else()
  message(WARNING "-- -- Tensorflow library _pywrap_tensorflow(_internal) not found in ${TENSORFLOW_PATH}")
  return()
endif()

# make a link to the library inside the devel directory so that we don't have to pass along the library dir
set(TENSORFLOW_LIBRARY ${CATKIN_DEVEL_PREFIX}/lib/${CMAKE_SHARED_LIBRARY_PREFIX}_pywrap_tensorflow${CMAKE_SHARED_LIBRARY_SUFFIX})
if(NOT EXISTS ${TENSORFLOW_LIBRARY})
  file(MAKE_DIRECTORY ${CATKIN_DEVEL_PREFIX}/lib)
  execute_process(
      COMMAND ln -s ${TENSORFLOW_ORIG_LIBRARY} ${TENSORFLOW_LIBRARY}
      RESULT_VARIABLE LINK_FAILED
      OUTPUT_QUIET
  )
  if("${LINK_FAILED}" STREQUAL "0")
    message("-- -- Created tensorflow library link ${TENSORFLOW_ORIG_LIBRARY} -> ${TENSORFLOW_LIBRARY}.")
  else()
    message(WARNING "-- -- Could not create symlink from ${TENSORFLOW_ORIG_LIBRARY} -> ${TENSORFLOW_LIBRARY}.")
    return()
  endif()
endif()

# make a link to the library also under the original name, since some linkers take the SONAME as the name they link to
get_filename_component(TENSORFLOW_ORIG_LIBRARY_NAME ${TENSORFLOW_ORIG_LIBRARY} NAME)
set(TENSORFLOW_LIBRARY2 ${CATKIN_DEVEL_PREFIX}/lib/${TENSORFLOW_ORIG_LIBRARY_NAME})
if(NOT EXISTS ${TENSORFLOW_LIBRARY2})
  execute_process(
      COMMAND ln -s ${TENSORFLOW_ORIG_LIBRARY} ${TENSORFLOW_LIBRARY2}
      RESULT_VARIABLE LINK_FAILED
      OUTPUT_QUIET
  )
  if("${LINK_FAILED}" STREQUAL "0")
    message("-- -- Created tensorflow library link ${TENSORFLOW_ORIG_LIBRARY} -> ${TENSORFLOW_LIBRARY2}.")
  else()
    message(WARNING "-- -- Could not create symlink from ${TENSORFLOW_ORIG_LIBRARY} -> ${TENSORFLOW_LIBRARY2}.")
    return()
  endif()
endif()

execute_process(
    COMMAND ldd ${TENSORFLOW_LIBRARY}
    OUTPUT_VARIABLE LDD_OUTPUT
)
if(${LDD_OUTPUT} MATCHES "libcudart.so")
  set(HAS_TENSORFLOW_GPU 1)
  message("-- -- The Tensorflow library is compiled with CUDA support.")
else()
  set(HAS_TENSORFLOW_GPU 0)
  message("-- -- The Tensorflow library is compiled without CUDA support.")
endif()

set(TENSORFLOW_LIBRARIES _pywrap_tensorflow)
set(TENSORFLOW_TARGETS ${TENSORFLOW_LIBRARY} ${TENSORFLOW_LIBRARY2})

# Added in tensorflow 1.4.0
set(TENSORFLOW_FRAMEWORK_LIBRARY ${CATKIN_DEVEL_PREFIX}/lib/${CMAKE_SHARED_LIBRARY_PREFIX}tensorflow_framework${CMAKE_SHARED_LIBRARY_SUFFIX})
set(TENSORFLOW_FRAMEWORK_ORIG_LIBRARY ${TENSORFLOW_PATH}/${CMAKE_SHARED_LIBRARY_PREFIX}tensorflow_framework${CMAKE_SHARED_LIBRARY_SUFFIX})
if(EXISTS ${TENSORFLOW_FRAMEWORK_ORIG_LIBRARY})
  if(NOT EXISTS ${TENSORFLOW_FRAMEWORK_LIBRARY})
    file(MAKE_DIRECTORY ${CATKIN_DEVEL_PREFIX}/lib)
    execute_process(
        COMMAND ln -s ${TENSORFLOW_FRAMEWORK_ORIG_LIBRARY} ${TENSORFLOW_FRAMEWORK_LIBRARY}
        RESULT_VARIABLE LINK_FAILED
        OUTPUT_QUIET
    )
    if("${LINK_FAILED}" STREQUAL "0")
      message("-- -- Created tensorflow library link ${TENSORFLOW_FRAMEWORK_ORIG_LIBRARY} -> ${TENSORFLOW_FRAMEWORK_LIBRARY}.")
    else()
      message(WARNING "-- -- Could not create symlink from ${TENSORFLOW_FRAMEWORK_ORIG_LIBRARY} -> ${TENSORFLOW_FRAMEWORK_LIBRARY}.")
      return()
    endif()
  endif()
  list(APPEND TENSORFLOW_LIBRARIES tensorflow_framework)
  list(APPEND TENSORFLOW_TARGETS ${TENSORFLOW_FRAMEWORK_LIBRARY})
endif()

if(NOT "${TF_PYTHON_LIBRARY}" STREQUAL "")
  if(EXISTS "${TF_PYTHON_LIBRARY}")
    set(PYTHON_LIBRARIES "${TF_PYTHON_LIBRARY}")
    message("-- -- Using Python${TF_PYTHON_VERSION} development library from ${TF_PYTHON_LIBRARY}")
  else()
    message(WARNING "-- -- Could not find python${TF_PYTHON_VERSION} development library at ${TF_PYTHON_LIBRARY}. Unset TF_PYTHON_LIBRARY and build again.")
    return()
  endif()
else()
  find_package(PythonLibs ${TF_PYTHON_VERSION})
  if (NOT ${PYTHONLIBS_FOUND})
    message(WARNING "-- -- Could not find python${TF_PYTHON_VERSION} development libraries, install python${TF_PYTHON_VERSION}-dev. If it is already installed and you still get this message, you have to provide the absolute path to libpythonx.y.so manually in TF_PYTHON_LIBRARY.")
    return()
  endif()
endif()

list(APPEND TENSORFLOW_LIBRARIES ${PYTHON_LIBRARIES})

set(TENSORFLOW_INCLUDE_DIRS ${TENSORFLOW_PATH}/include)
# Added in tensorflow 1.4.0
if (EXISTS ${TENSORFLOW_PATH}/include/external/nsync/public)
  list(APPEND TENSORFLOW_INCLUDE_DIRS ${TENSORFLOW_PATH}/include/external/nsync/public)
endif()

if(${SYSTEM_USES_CXX11_ABI})
  message(WARNING "-- -- Tensorflow library and system C++ ABI differ. You should consider building Tensorflow yourself. In case you still want to use the python TF library, itcan only be used from targets that do not link to any system libraries that have a C++ API (e.g. a method with std::string argument). Your library using Tensorflow must expose all methods using a C API, or a limited C++ API with only primitive data types (numerics, arrays, chars, but not std::vectors or std::strings). See the kinetic-devel branch of github.com/tradr-project/tensorflow_ros_test for an example.")
endif()

set(TENSORFLOW_FOUND 1)
set(TENSORFLOW_FOUND_BY "pip")
set(tensorflow_ros_cpp_INCLUDE_DIRS ${TENSORFLOW_INCLUDE_DIRS})
set(tensorflow_ros_cpp_LIBRARIES ${TENSORFLOW_LIBRARIES})
