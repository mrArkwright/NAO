cmake_minimum_required(VERSION 2.8)
project(Sample)

include("${CMAKE_CURRENT_SOURCE_DIR}/tuhhsdk/sdk.cmake")
include_directories("${CMAKE_CURRENT_SOURCE_DIR}/tuhhsdk/src")

# compile mode independent source files
SET (SRCS
	src/sample.h
	src/sample.cpp
)

option(SDK_INSTALLED "is Aldebaran SDK installed?" OFF)
option(WEBOTS "compile for WEBOTS" ON)

IF (WEBOTS)
	add_definitions(-DWEBOTS)
ENDIF(WEBOTS)

IF (SDK_INSTALLED)
	#source files for cross-compilation
	SET (SRCS
		${SRCS}
		main.cpp
	)

	find_package(qibuild)
	qi_create_lib(Sample ${TUHHSDK_SRCS} ${SRCS} SUBFOLDER naoqi)
	qi_use_lib(Sample ALCOMMON)
ELSE (SDK_INSTALLED)
	add_library(Sample ${SRCS} ${TUHHSDK_SRCS})
ENDIF (SDK_INSTALLED)
