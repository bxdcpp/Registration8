CMAKE_MINIMUM_REQUIRED(VERSION 3.0)

PROJECT(ImageRegistration8Test)

set(Slicer_DIR T:/Slicer-build/Slicer-build)
#set(BRAINSCommonLib_SRC  ${Slicer_DIR}/../BRAINSTools/BRAINSCommonLib)
#set(BRAINSCommonLib_DIR  ${Slicer_DIR}/E/BRAINSTools/BRAINSCommonLib)
set(ITK_DIR ${Slicer_DIR}/../ITK-build)



FIND_PACKAGE(ITK)

IF(ITK_FOUND) 
    INCLUDE(${ITK_USE_FILE})

ELSE(ITK_FOUND) 
    MESSAGE(FATAL_ERROR 
            "ITK not found. Please set ITK_DIR.") 
ENDIF(ITK_FOUND)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/../bin)



include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}
  ${CMAKE_CURRENT_BINARY_DIR}
  #${BRAINSCommonLib_SRC}
  #${BRAINSCommonLib_DIR}
  )
  message(${CMAKE_CURRENT_BINARY_DIR})




set(${PROJECT_NAME}_SRCS
ImageRegistration8.cxx
VisualITKImageRegistration.h
VisualITKImageRegistration.cpp
)

set(lib_link
    ${ITK_LIBRARIES}
    #${Slicer_DIR}/lib/Visual3D-4.11/Release/BRAINSCommonLib.lib
	#${Slicer_DIR}/lib/Visual3D-4.11/DeBug/BRAINSCommonLib.lib
)

ADD_EXECUTABLE(${PROJECT_NAME} ${${PROJECT_NAME}_SRCS} )
TARGET_LINK_LIBRARIES(${PROJECT_NAME} ${lib_link})