cmake_minimum_required(VERSION 2.8)

project(fcontrol)

configure_file(${CMAKE_CURRENT_SOURCE_DIR}/files/phi.m_exp.csv ${CMAKE_CURRENT_BINARY_DIR} COPYONLY)

# Pick up cmake modules - they all must be in the cmake subdirectory
# let cmake use them
list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

#deps
#1-fftw
find_package(FFTW REQUIRED)
#2-eigen
find_package(Eigen3 REQUIRED NO_MODULE)
message(STATUS "EIGEN: " ${EIGEN3_VERSION} " (internal)")
include_directories(${EIGEN3_INCLUDE_DIR})

# source files for fcontrol
aux_source_directory(./src FCONTROL_SRCS)
# include files for fcontrol
FILE(GLOB_RECURSE FCONTROL_INCLUDE_FILES src/*.h src/*.hpp)

set(SUBDIR_LINK_NAMES ${SUBDIR_LINK_NAMES} ${PROJECT_NAME} PARENT_SCOPE)
set(SUBDIR_INCLUDE_DIRECTORIES ${SUBDIR_INCLUDE_DIRECTORIES}
    ${CMAKE_CURRENT_SOURCE_DIR}/src
    ${EIGEN3_INCLUDE_DIR}
    PARENT_SCOPE)

ADD_LIBRARY(${PROJECT_NAME} SHARED ${FCONTROL_SRCS} ${FCONTROL_INCLUDE_FILES})



#Install Library targets if top level project
if("${CMAKE_PROJECT_NAME}" STREQUAL "${PROJECT_NAME}")
    #install target
    message(STATUS "Configuring " ${PROJECT_NAME} " as main project")
    install(TARGETS ${PROJECT_NAME}
        RUNTIME DESTINATION bin/${PROJECT_NAME}
        LIBRARY DESTINATION lib/${PROJECT_NAME})
    install(FILES ${FCONTROL_INCLUDE_FILES}
        DESTINATION include/${PROJECT_NAME})

    # uninstall target
    configure_file(
        "${CMAKE_CURRENT_SOURCE_DIR}/cmake/cmake_uninstall.cmake.in"
        "${CMAKE_CURRENT_BINARY_DIR}/cmake_uninstall.cmake"
        IMMEDIATE @ONLY)
    add_custom_target(uninstall
        COMMAND ${CMAKE_COMMAND} -P ${CMAKE_CURRENT_BINARY_DIR}/cmake_uninstall.cmake)

#Set Library variables if subdirectory (called from add_subdirectory() )
else("${CMAKE_PROJECT_NAME}" STREQUAL "${PROJECT_NAME}")
    message(STATUS "Configuring " ${PROJECT_NAME} " as sub-project in " ${CMAKE_CURRENT_SOURCE_DIR})
#    find_path(FCONTROL_SOURCE_DIR fcontrol.h
#        HINTS ${CMAKE_CURRENT_SOURCE_DIR} PATH_SUFFIXES include src)
#    find_path(FCONTROL_INCLUDE_FILES fcontrol.h
#        HINTS ${CMAKE_CURRENT_SOURCE_DIR} PATH_SUFFIXES include src)

  #  set(SUBDIR_INCLUDE_DIRECTORIES ${SUBDIR_INCLUDE_DIRECTORIES} ${FCONTROL_INCLUDE_FILES} PARENT_SCOPE)

#    find_library(FCONTROL_LIBRARY NAMES ${PROJECT_NAME} lib${PROJECT_NAME}
#        HINTS ${CMAKE_CURRENT_BINARY_DIR} )
#message(STATUS "found " ${FCONTROL_INCLUDE_FILES} " in " ${CMAKE_CURRENT_BINARY_DIR})
#message(STATUS "found " ${FCONTROL_LIBRARY} " in " ${CMAKE_CURRENT_BINARY_DIR})

endif()


target_link_libraries( ${PROJECT_NAME} ${FFTW_LIBRARIES} ${Eigen3_LIBRARIES})

