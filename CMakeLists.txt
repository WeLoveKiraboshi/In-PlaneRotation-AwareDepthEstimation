cmake_minimum_required(VERSION 2.8)
project(CNN_MonoFusion)

IF(NOT CMAKE_BUILD_TYPE)
  SET(CMAKE_BUILD_TYPE Release)
ENDIF()

SET(CMAKE_BUILD_TYPE Release)

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -g -Wall -O3 -march=native ")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g -Wall -O3 -march=native")

# Check C++11 or C++0x support
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
SET(DBUILD_SHARED true)
if(COMPILER_SUPPORTS_CXX11)
   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
   add_definitions(-DCOMPILEDWITHC11)
   message(STATUS "Using flag -std=c++11.")
elseif(COMPILER_SUPPORTS_CXX0X)
   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
   add_definitions(-DCOMPILEDWITHC0X)
   message(STATUS "Using flag -std=c++0x.")
else()
   message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()



LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)

set(OpenCV_DIR "/home/yukisaito/opencv-3.4.9/build")
find_package(OpenCV QUIET)

find_package(Eigen3 3.1.0 REQUIRED)
find_package(Pangolin REQUIRED)
#find_package(OpenGL REQUIRED)
#find_package(GLUT REQUIRED)

#PythonLibs
set(PYTHONLIBS_VERSION_STRING "3.6")
set(PYTHON_LIBRARIES "/home/yukisaito/anaconda3/envs/py36/lib/libpython3.6m.so")
set(PYTHON_INCLUDE_DIRS "/home/yukisaito/anaconda3/envs/py36/include/python3.6m")
message("Using python:  ${PYTHONLIBS_VERSION_STRING} ")
message("Using PYTHON_LIBRARIES:  ${PYTHON_LIBRARIES} ")
message("Using PYTHON_INCLUDE_DIRS:  ${PYTHON_INCLUDE_DIRS} ")
include_directories("/home/yukisaito/anaconda3/envs/py36/lib/python3.6/site-packages/numpy/core/include") # for numpy

#[[
set(PANGOLIN_INCLUDE ${PROJECT_SOURCE_DIR}/Thirdparty/Pangolin/include)
set(PANGOLIN_LIB ${PROJECT_SOURCE_DIR}/Thirdparty/Pangolin/pangolin.lib)
set(PANGOLIN_external ${PROJECT_SOURCE_DIR}/Thirdparty/Pangolin/external)
set(PANGOLIN_BUILD_GENERATE ${PROJECT_SOURCE_DIR}/Thirdparty/Pangolin/build_generate)
]]



include_directories(
#${OPENGL_INCLUDE_DIRS}
#${GLUT_INCLUDE_DIRS}
${PROJECT_SOURCE_DIR}
${PROJECT_SOURCE_DIR}/include
${PROJECT_SOURCE_DIR}/src/dense_map
${EIGEN3_INCLUDE_DIR}
${PANGOLIN_INCLUDE}
#${PANGOLIN_external}/glew/include
#${PANGOLIN_external}/libjpeg/include
#${PANGOLIN_external}/libpng/include
#${PANGOLIN_external}/zlib/include
#${PANGOLIN_BUILD_GENERATE}/include
${PYTHON_INCLUDE_DIRS}
)

set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)


#file(GLOB ORBSLAM_SRCS src/*.h include/*.h src/*.cpp src/*.cc)
#file(GLOB DenseMap_SRCS src/dense_map/*.h src/dense_map/*.cpp src/dense_map/*.cc)
file(GLOB ORBSLAM_SRCS src/*.cpp src/*.cc)
file(GLOB DenseMap_SRCS src/dense_map/*.cpp src/dense_map/*.cc)
add_library(${PROJECT_NAME} STATIC
${ORBSLAM_SRCS}
${DenseMap_SRCS}
)



target_link_libraries(${PROJECT_NAME}
${OpenCV_LIBS}
${EIGEN3_LIBS}
${Pangolin_LIBRARIES}
${PROJECT_SOURCE_DIR}/Thirdparty/DBoW2/lib/libDBoW2.so
${PROJECT_SOURCE_DIR}/Thirdparty/g2o/lib/libg2o.so
#${PANGOLIN_external}/libpng/lib/libpng16_static.lib
#${PANGOLIN_external}/zlib/lib/zlibstatic.lib
#${PANGOLIN_external}/libjpeg/lib/jpeg.lib
#${PANGOLIN_BUILD_GENERATE}/Release/pangolin.lib
#${PANGOLIN_external}/glew/lib/glew.lib
#glu32.lib
#opengl32.lib
${PYTHON_LIBRARIES}
#${OPENGL_LIBRARIES}
#${GLUT_LIBRARY}
)


set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/Examples/rgbd_monodepth)
#file(GLOB SRCS Examples/rgbd_monodepth/*.h Examples/rgbd_monodepth/*.cpp Examples/rgbd_monodepth/*.cc)
add_executable(rgbd_monodepth Examples/rgbd_monodepth/rgbd_monodepth.cc Examples/rgbd_monodepth/rgbd_monodepth.h)
target_link_libraries(rgbd_monodepth ${PROJECT_NAME})

add_executable(webcam_monodepth Examples/rgbd_monodepth/webcam.cc)
target_link_libraries(webcam_monodepth ${PROJECT_NAME})

