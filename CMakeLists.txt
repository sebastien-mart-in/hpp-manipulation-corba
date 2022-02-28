# Copyright (c) 2012, 2020 CNRS
# Author: Florent Lamiraux, Guilhem Saurel
#

# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are
# met:
#
# 1. Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#
# 2. Redistributions in binary form must reproduce the above copyright
# notice, this list of conditions and the following disclaimer in the
# documentation and/or other materials provided with the distribution.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
# A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
# HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
# SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
# LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
# DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
# THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH
# DAMAGE.

CMAKE_MINIMUM_REQUIRED(VERSION 3.1)

SET(PROJECT_NAME hpp-manipulation-corba)
SET(PROJECT_DESCRIPTION "Corba server for manipulation planning")

SET(CLIENT_ONLY FALSE CACHE BOOL "Set to true to install the client only")
SET(CUSTOM_HEADER_DIR hpp/corbaserver/manipulation)
SET(PROJECT_USE_CMAKE_EXPORT TRUE)
SET(PROJECT_USE_KEYWORD_LINK_LIBRARIES TRUE)
SET(CXX_DISABLE_WERROR true)

INCLUDE(cmake/hpp.cmake)
INCLUDE(cmake/idl.cmake)
INCLUDE(cmake/python.cmake)

COMPUTE_PROJECT_ARGS(PROJECT_ARGS LANGUAGES CXX)
PROJECT(${PROJECT_NAME} ${PROJECT_ARGS})

LIST(APPEND PKG_CONFIG_ADDITIONAL_VARIABLES cmake_plugin)
CONFIG_FILES(doc/main.hh)

SET(${PROJECT_NAME}_HEADERS
  include/hpp/corbaserver/manipulation/client.hh
  )

IF(NOT CLIENT_ONLY)
  SET(${PROJECT_NAME}_HEADERS
    include/hpp/corbaserver/manipulation/server.hh
    include/hpp/corbaserver/manipulation/fwd.hh
    ${${PROJECT_NAME}_HEADERS}
    )
ENDIF(NOT CLIENT_ONLY)

# search for python
FINDPYTHON()

ADD_PROJECT_DEPENDENCY("hpp-corbaserver" REQUIRED)
IF(NOT CLIENT_ONLY)
    ADD_PROJECT_DEPENDENCY("hpp-manipulation" REQUIRED)
    ADD_PROJECT_DEPENDENCY("hpp-manipulation-urdf" REQUIRED)
ENDIF(NOT CLIENT_ONLY)
ADD_REQUIRED_DEPENDENCY("omniORB4 >= 4.1.4")

ADD_SUBDIRECTORY(src)

PKG_CONFIG_APPEND_LIBS(${PROJECT_NAME})

INSTALL(FILES package.xml DESTINATION share/${PROJECT_NAME})
