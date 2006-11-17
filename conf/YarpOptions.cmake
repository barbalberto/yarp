
##################################################
#ask user to specify build type (linux only)
IF(NOT CMAKE_BUILD_TYPE)
  SET(CMAKE_BUILD_TYPE "Release" CACHE STRING 
	"Choose the type of build, recommanded options are: Debug or Release")
ENDIF(NOT CMAKE_BUILD_TYPE)
# hide variable to WINDOWS users (CMAKE_BUILD_TYPE is not used on win)
IF (WIN32 AND NOT CYGWIN)
  MARK_AS_ADVANCED(CMAKE_BUILD_TYPE)
ENDIF(WIN32 AND NOT CYGWIN)
#################################################


SET(CREATE_DEVICE_LIBRARY FALSE CACHE BOOL "Do you want to compile the device library")
SET(CREATE_GUIS FALSE CACHE BOOL "Do you want to compile GUIs")

SET(CREATE_SHARED_LIBRARY FALSE CACHE BOOL "Compile shared libraries rather than linking statically")
IF (WIN32)
	MARK_AS_ADVANCED(CREATE_SHARED_LIBRARY)
ENDIF (WIN32)


# Flag for device testing and documentation - not really for end-user,
# but instead the library developers
SET(CREATE_DEVICE_TESTS FALSE CACHE BOOL "Do you want to create device tests")
MARK_AS_ADVANCED(CREATE_DEVICE_TESTS)
