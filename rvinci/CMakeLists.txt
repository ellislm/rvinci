cmake_minimum_required(VERSION 2.8.3)
project(rvinci)

# Qt
find_package(Qt4 COMPONENTS QtCore QtGui REQUIRED)
include(${QT_USE_FILE})
add_definitions(-DQT_NO_KEYWORDS -fPIC)

# Catkin
find_package(catkin REQUIRED COMPONENTS 
  rviz
  roscpp
  tf
  rvinci_input_msg
)

# Boost
find_package(Boost REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES rvinci
  CATKIN_DEPENDS rviz
#  DEPENDS OculusSDK
)

find_package(PkgConfig)
pkg_check_modules(OGRE OGRE)

include_directories(include
  ${catkin_INCLUDE_DIRS}
#  ${OculusSDK_INCLUDE_DIRS}
  ${OGRE_INCLUDE_DIRS}
  ${BOOST_INCLUDE_DIRS}
)

# run Qt moc generator
qt4_wrap_cpp(MOC_FILES
  include/${PROJECT_NAME}/rvinci_display.h
# include/${PROJECT_NAME}/rvinci_cursor.h
 include/${PROJECT_NAME}/fixed_view_controller.h
#  include/oculus_rviz_plugins/fixed_view_controller.h
)

add_library(rvinci
  src/rvinci_display.cpp
  src/fixed_view_controller.cpp
  src/rvinci_pose.cpp
  ${MOC_FILES}
)

target_link_libraries(rvinci
  ${catkin_LIBRARIES}
  ${QT_LIBRARIES}
#  ${OculusSDK_LIBRARIES}
  ${BOOST_LIBRARIES}
)


#############
## Install ##
#############

install(TARGETS rvinci
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)

install(DIRECTORY ogre_media
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(FILES
  plugin_description.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY
  icons
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
