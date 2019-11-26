add_executable (grasp_driver main.cpp canAPI.cpp RockScissorsPaper.cpp)

# - jsoncpp
if (CMAKE_SYSTEM_NAME MATCHES Darwin) 
    find_library(JSONCPP_LIBRARY jsoncpp)
elseif (CMAKE_SYSTEM_NAME MATCHES Linux)
    find_package(PkgConfig REQUIRED)
    pkg_check_modules(JSONCPP jsoncpp)
    include_directories(${JSONCPP_INCLUDE_DIRS})
    link_directories(${JSONCPP_LIBRARY_DIRS})
    set(JSONCPP_LIBRARY ${JSONCPP_LIBRARIES})
endif ()

target_link_libraries (grasp_driver BHand pcanbasic pthread hiredis ${JSONCPP_LIBRARY})
#install (TARGETS grasp DESTINATION ${PROJECT_SOURCE_DIR}/bin)
install (TARGETS grasp_driver DESTINATION ${PROJECT_BINARY_DIR}/bin)
