cmake_minimum_required(VERSION 3.0.0)
project(biped_v2 VERSION 0.1.0)

set(CMAKE_POSITION_INDEPENDENT_CODE ON)

set(biped_v2_include_dir
  ./
  ../common
  ../planner
  ../hardware
  ../estimator
  ../controller
  ../opensource/biped_v2
  ../opensource/common
  ../../lib/xsens_ros_mti_driver/src
  ../../lib/xsens_ros_mti_driver/lib/xspublic/
  ../../lib/Predict_Fp
  ../../lib/Math
  ../../lib/EC_Master/src/Common
  ../../lib/EC_Master/src/SDK/INC 
  ../../lib/EC_Master/src/SDK/INC/Linux
  ../../lib/EC_Master/src/Sharelib/Common/ 
  ../../lib/EC_Master/src/Sharelib/EcMasterDemo/
  ../../lib/EC_Master/src/Sharelib/Linux/
  ../../lib/motioncapture/
  ../../lib/motioncapture/include/
  ../../lib/DynamixelSDK/include/dynamixel_sdk
  ../../lib/DynamixelSDK/src/
  ../../lib/ruierman_controller
  ../../lib/jodell_claw_driver/include
)
set(biped_v2_link_dir
../../lib/xsens_ros_mti_driver/lib/xspublic/xscontroller
../../lib/xsens_ros_mti_driver/lib/xspublic/xscommon
../../lib/xsens_ros_mti_driver/lib/xspublic/xstypes
../../lib/
../../lib/jodell_claw_driver/lib
../../lib/EC_Master/src/SDK/LIB/Linux/x64/
../../lib/motioncapture/lib/
)
set(target_objs
  # $<TARGET_OBJECTS:pb_controller_obj>
  # $<TARGET_OBJECTS:plantIK_obj>
  # $<TARGET_OBJECTS:Predict_Fp_obj>
  ${CMAKE_CURRENT_SOURCE_DIR}/../controller/pb_controller.cc.o
  ${CMAKE_CURRENT_SOURCE_DIR}/../planner/plantIK.cc.o
  ${CMAKE_CURRENT_SOURCE_DIR}/../../lib/Predict_Fp/Predict_Fp.cpp.o
)

# add_library(pb_controller_obj OBJECT ../common/pb_controller.cc)
# target_link_libraries(pb_controller_obj pthread gflags lcm drake::drake elmo xsens_mti_driver casadi ncurses)
# # 移除所有符号  信息
# set_target_properties(pb_controller_obj PROPERTIES LINK_FLAGS "-s")

# HighlyDynamicRobot_lib
set(OPENSOURCE_SRC
  ${CMAKE_CURRENT_SOURCE_DIR}/biped_v2/HighlyDynamicRobot.cc
  ${CMAKE_CURRENT_SOURCE_DIR}/common/config.cpp
  )
add_library(HighlyDynamicRobot_lib STATIC ${OPENSOURCE_SRC} )
target_include_directories(HighlyDynamicRobot_lib PUBLIC ${biped_v2_include_dir})  
target_link_directories(HighlyDynamicRobot_lib PUBLIC ${biped_v2_link_dir})
set(RobotLink_LIBS 
  hardware
  planner
  controllers
  estimators
  ankle_controller
  ankle_estimator
  arm_ankle_estimator 
  arm_ankle_controller
  common
  # ${target_objs} 
  xsens_mti_driver
  pbcwalk-lcm-types 
  ec_app
  ec_master_main
  EcMaster
  AtemRasSrv
  gflags
  xscontroller 
  xscommon 
  xstypes 
  lcm 
  mathtools
  dl 
  pthread  
  drake::drake 
  stdc++fs
  motionClient
  nokov_sdk
  dxl_x64_cpp
  jodell_claw_driver
  jodellTool2
  ruierman_actuatorLib
  ${Boost_LIBRARIES}
  Python3::Python
  )
if(OPENSOURCE)
  list(APPEND RobotLink_LIBS 
  ${target_objs} 
  casadi 
  ncurses
  )
endif()

message(STATUS "Dumping robotlink_libs: ${RobotLink_LIBS}")

if(DEFINED CATKIN_DEVEL_PREFIX)
  target_link_libraries(HighlyDynamicRobot_lib ${RobotLink_LIBS} ${catkin_LIBRARIES})
else()
  target_link_libraries(HighlyDynamicRobot_lib ${RobotLink_LIBS})
endif()
# if(NOT KUAVO_INTERNAL_CONTROLLER)
#   target_link_libraries(HighlyDynamicRobot_lib controllers)
# endif()

# Define the output directory for the library file
set(LIBRARY_OUTPUT_PATH "${CMAKE_SOURCE_DIR}/lib")

# Ensure the target directory exists before the build starts
add_custom_command(TARGET HighlyDynamicRobot_lib PRE_BUILD
  COMMAND ${CMAKE_COMMAND} -E make_directory ${LIBRARY_OUTPUT_PATH})

# Add a post-build command to copy the library file
add_custom_command(TARGET HighlyDynamicRobot_lib POST_BUILD
  COMMAND ${CMAKE_COMMAND} -E copy
  $<TARGET_FILE:HighlyDynamicRobot_lib>
  ${LIBRARY_OUTPUT_PATH})



file(WRITE ${CMAKE_CURRENT_BINARY_DIR}/CURRENT_SOURCE_DIR "${CMAKE_SOURCE_DIR}")
# Add the executable with the project name and the collected source files
add_executable(kuavoController ${CMAKE_CURRENT_SOURCE_DIR}/../../src/opensource/biped_v2/main.cc)
target_link_directories(kuavoController PUBLIC ${biped_v2_link_dir})
target_include_directories(kuavoController PUBLIC ${biped_v2_include_dir})

# Link the executable with the required library
target_link_libraries(kuavoController HighlyDynamicRobot_lib)
if(KUAVO_INTERNAL_CONTROLLER)
  target_link_libraries(kuavoController controllers)
endif()


## test 
# add_executable(csv_test ../../example/biped/csvloader_test.cc  ${SRCS})
# target_link_directories(csv_test PUBLIC ${biped_v2_link_dir})
# target_include_directories(csv_test PUBLIC ${biped_v2_include_dir})  
# target_link_libraries(csv_test pthread ${OBJS} gflags lcm drake::drake xsens_mti_driver casadi ncurses ec_master_main ec_app pbcwalk-lcm-types)

# add_executable(csv_test ../../example/biped/csv_player_test.cc  ${SRCS})
# target_link_directories(csv_test PUBLIC ${biped_v2_link_dir})
# target_include_directories(csv_test PUBLIC ${biped_v2_include_dir})  
# target_link_libraries(csv_test pthread ${OBJS} gflags lcm drake::drake xsens_mti_driver casadi ncurses ec_master_main ec_app pbcwalk-lcm-types)

set(EXECUTABLE_OUTPUT_PATH ${CMAKE_BINARY_DIR}/src/biped_v2)

add_executable(${PROJECT_NAME}_play_back ./biped_v2/drake_play_back.cc)
# target_link_directories(csv_test PUBLIC ${biped_v2_link_dir})
# target_include_directories(csv_test PUBLIC ${biped_v2_include_dir})  
target_link_libraries(${PROJECT_NAME}_play_back HighlyDynamicRobot_lib)

if(KUAVO_INTERNAL_CONTROLLER)
  target_link_libraries(${PROJECT_NAME}_play_back controllers)
endif()

add_executable(bin2csv ./biped_v2/logBin2Csv.cc)
target_include_directories(bin2csv PUBLIC ${biped_v2_include_dir})
target_link_directories(bin2csv PUBLIC ${biped_v2_link_dir})
target_link_libraries(bin2csv common drake::drake stdc++fs)


