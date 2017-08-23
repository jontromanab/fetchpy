cmake_minimum_required(VERSION 2.8.3)
project(fetchpy)

find_package(catkin REQUIRED)
catkin_package()
catkin_python_setup()

install(DIRECTORY config/
  DESTINATION "${CATKIN_PACKAGE_SHARE_DESTINATION}/config"
)
install(PROGRAMS scripts/console.py
DESTINATION "${CATKIN_PACKAGE_BIN_DESTINATION}"
)
                 

##if (CATKIN_ENABLE_TESTING)
  ##catkin_add_nosetests(tests)
##endif()