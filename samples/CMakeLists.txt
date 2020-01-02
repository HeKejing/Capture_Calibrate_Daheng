cmake_minimum_required(VERSION 2.8)
project( DualCalib_Camera )


#---------------------------------------------------------------------------------------
# add source files
#---------------------------------------------------------------------------------------
include_directories("../")
AUX_SOURCE_DIRECTORY(../ DIR_SRCS)
set(DIR_SRCS ${DIR_SRCS} main.cpp KJCalibration.cpp KJCalibration.h)
MESSAGE(STATUS ${DIR_SRCS})

FILE(GLOB DIR_HEADERS ../*.h)

#---------------------------------------------------------------------------------------
# OpenCV
#---------------------------------------------------------------------------------------
find_package( OpenCV REQUIRED )
include_directories(${OpenCV_INCLUDE_DIRS})


#---------------------------------------------------------------------------------------
# daheng
#---------------------------------------------------------------------------------------
set(DAHENG_INCLUDE "${CMAKE_SOURCE_DIR}/../3rdParty/windows/daheng/VC SDK/inc")
set(DAHENG_LIBS_DIR "${CMAKE_SOURCE_DIR}/../3rdParty/windows/daheng/VC SDK/lib/x64")

include_directories(${DAHENG_INCLUDE})
link_directories(${DAHENG_LIBS_DIR})


#---------------------------------------------------------------------------------------
# CameraDemo
#---------------------------------------------------------------------------------------
add_executable( DualCalib_Camera ${DIR_SRCS} ${DIR_HEADERS} )
target_link_libraries( DualCalib_Camera ${OpenCV_LIBS} )
target_link_libraries( DualCalib_Camera GxIAPI.lib DxImageProc.lib )


#---------------------------------------------------------------------------------------
# copy daheng dlls to the exe directory
#---------------------------------------------------------------------------------------
add_custom_command(TARGET CameraDemo PRE_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_directory
    "${CMAKE_SOURCE_DIR}/../3rdParty/windows/daheng/VC SDK/bin" $<TARGET_FILE_DIR:CameraDemo>)


#---------------------------------------------------------------------------------------
# set groups
#---------------------------------------------------------------------------------------
source_group("Header Files" FILES ${DIR_HEADERS})
