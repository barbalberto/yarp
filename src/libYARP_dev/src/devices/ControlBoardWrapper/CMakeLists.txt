# Copyright (C) 2014 Istituto Italiano di Tecnologia (IIT)
# Author: Alberto Cardellino <alberto.cardellino@iit.it>
# CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT


# use PARENT_SCOPE to make the following variables available at the 'parent' cmake

# Test using yarp_add_idl
include(YarpIDL)

# message("msg folder is ${yarp_msgs_SOURCE_DIR}")
# set(IDL_FILES_2 ${yarp_msgs_SOURCE_DIR}/ros/msg_file/robotology_msgs/controlBoardMsg.msg)
# yarp_add_idl(IDL_GEN_FILES_2 ${IDL_FILES_2})
#
# message("generated files ares ${IDL_GEN_FILES_2}")


set(cbw2_core_srcs      src/devices/ControlBoardWrapper/ControlBoardWrapper.cpp
                        src/devices/ControlBoardWrapper/RPCMessagesParser.cpp
                        src/devices/ControlBoardWrapper/StreamingMessagesParser.cpp
                        src/devices/ControlBoardWrapper/ROSCmdMessagesParser.cpp
                        src/devices/ControlBoardWrapper/SubDevice.cpp
                        PARENT_SCOPE)

set(cbw2_core_hrds      src/devices/ControlBoardWrapper/ControlBoardWrapper.h
                        src/devices/ControlBoardWrapper/RPCMessagesParser.h
                        src/devices/ControlBoardWrapper/StreamingMessagesParser.h
                        src/devices/ControlBoardWrapper/ROSCmdMessagesParser.h
                        src/devices/ControlBoardWrapper/SubDevice.h
                        ${IDL_GEN_FILES_2}
                        PARENT_SCOPE)


set(cbw2_core_incl_dirs ${CMAKE_CURRENT_SOURCE_DIR} PARENT_SCOPE)
