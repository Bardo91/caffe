###############################################################
######                                                   ######
######  	RGBD SLAM                                    ######
######			Author: Pablo R.S. (aka. Bardo91)        ######
######                                                   ######
###############################################################

cmake_minimum_required (VERSION 2.8 FATAL_ERROR)
project(caffetest)

set(PROJECT_VERSION "0.1.0")

IF(NOT CMAKE_BUILD_TYPE )
   SET( CMAKE_BUILD_TYPE "Debug" )
ENDIF()

if(WIN32)
    # Postfix of DLLs:
    SET(PROJECT_DLLVERSION "${PROJECT_VERSION_MAJOR}${PROJECT_VERSION_MINOR}${PROJECT_VERSION_PATCH}")
    SET(RUNTIME_OUTPUT_PATH ${PROJECT_BINARY_DIR}/bin CACHE PATH "Directory for dlls and binaries")
    SET(EXECUTABLE_OUTPUT_PATH ${PROJECT_BINARY_DIR}/bin CACHE PATH "Directory for binaries")
    SET(LIBRARY_OUTPUT_PATH ${PROJECT_BINARY_DIR}/bin CACHE PATH "Directory for dlls")
else()
    # Postfix of so's:
    set(PROJECT_DLLVERSION)
    SET(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_INSTALL_PREFIX}/lib/cmake/ /usr/lib/cmake)

endif()


if(CMAKE_COMPILER_IS_GNUCXX)
	add_definitions(-std=c++11)
endif()

# Clasify sources according to folder structure.
# This creates a nice layout for the project in visual studio
# This macro is courtesy of http://www.cmake.org/pipermail/cmake/2013-November/056336.html
macro(GroupSources curdir)
	file(GLOB children RELATIVE ${PROJECT_SOURCE_DIR}/${curdir}
		${PROJECT_SOURCE_DIR}/${curdir}/*)
	foreach(child ${children})
		if(IS_DIRECTORY ${PROJECT_SOURCE_DIR}/${curdir}/${child})
			GroupSources(${curdir}/${child})
		else()
			string(REPLACE "/" "\\" groupname ${curdir})
			source_group(${groupname} FILES ${PROJECT_SOURCE_DIR}/${curdir}/${child})
		endif()
	endforeach()
endmacro()

GroupSources(rgbd_slam)

#########################################
###### Loading 3rd party libraries.######
#########################################
set(CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake/modules/" ${CMAKE_MODULE_PATH})

## Load opencv
find_package(OpenCV REQUIRED HINTS "/opt/ros/kinetic/share/OpenCV-3.2.0-dev")
set_target_properties(${OpenCV_LIBS} PROPERTIES MAP_IMPORTED_CONFIG_RELWITHDEBINFO RELEASE)


## Find CUDA and ZED SDKs
find_package(Caffe REQUIRED HINTS "/usr/local/share/Caffe/CaffeConfig.cmake")
find_package(Boost REQUIRED COMPONENTS system)

#########################################
#########################################
# Collect all sources
file(GLOB_RECURSE CAFFETEST_SOURCE_FILES "src/*.cpp" "src/*.h" "src/*.inl" "src/*.txt")

set(PROJECT_NAME caffetest)

add_definitions(-DUSE_OPENCV)
add_definitions(${Caffe_DEFINITIONS})

add_executable(${PROJECT_NAME} ${CAFFETEST_SOURCE_FILES})

target_include_directories(${PROJECT_NAME} PUBLIC ${OpenCV_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME} LINK_PUBLIC ${OpenCV_LIBS})

target_include_directories(${PROJECT_NAME} PUBLIC ${CUDA_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME} LINK_PUBLIC ${CUDA_LIBRARIES})

target_include_directories(${PROJECT_NAME} PUBLIC ${CUDNN_INCLUDE})
target_link_libraries(${PROJECT_NAME} LINK_PUBLIC ${CUDNN_LIBRARY})

target_include_directories(${PROJECT_NAME} PUBLIC ${Caffe_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME} LINK_PUBLIC ${Caffe_LIBRARIES})

target_include_directories(${PROJECT_NAME} PUBLIC ${GLOG_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME} LINK_PUBLIC ${GLOG_LIBRARIES})

target_include_directories(${PROJECT_NAME} PUBLIC ${Boost_INCLUDE_DIRS} )
target_link_libraries(${PROJECT_NAME} LINK_PUBLIC ${Boost_LIBRARIES})

target_include_directories(${PROJECT_NAME} PUBLIC ${GPUNMS_INCLUDE_DIR} )
target_link_libraries(${PROJECT_NAME} LINK_PUBLIC ${GPUNMS_LIBRARY})

