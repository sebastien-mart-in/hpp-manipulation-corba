# Copyright (c) 2012 CNRS
# Author: Florent Lamiraux
#
# This file is part of hpp-manipulation-corba.
# hpp-manipulation-corba is free software: you can redistribute it
# and/or modify it under the terms of the GNU Lesser General Public
# License as published by the Free Software Foundation, either version
# 3 of the License, or (at your option) any later version.
#
# hpp-manipulation-corba is distributed in the hope that it will be
# useful, but WITHOUT ANY WARRANTY; without even the implied warranty
# of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
# General Lesser Public License for more details.  You should have
# received a copy of the GNU Lesser General Public License along with
# hpp-manipulation-corba.  If not, see
# <http://www.gnu.org/licenses/>.

SET(LIBRARY_NAME ${PROJECT_NAME})
SET(IDL_SOURCES
  robot
)

SET(HPP_CORBASERVER_IDL_SOURCES
  common
)

OMNIIDL_INCLUDE_DIRECTORIES(
  ${HPP_CORBASERVER_DATAROOTDIR}/idl ${CMAKE_SOURCE_DIR}/idl
  )

INCLUDE_DIRECTORIES(${CMAKE_BINARY_DIR}/src)
FOREACH(IDL ${IDL_SOURCES})
  GENERATE_IDL_FILE (${IDL}
    ${CMAKE_SOURCE_DIR}/idl/hpp/corbaserver/manipulation
    )
ENDFOREACH()

FOREACH(IDL ${HPP_CORBASERVER_IDL_SOURCES})
  GENERATE_IDL_FILE (${IDL}
    ${HPP_CORBASERVER_DATAROOTDIR}/idl/hpp/corbaserver
    )
ENDFOREACH()

ADD_LIBRARY(${LIBRARY_NAME} SHARED
  common.hh
  commonSK.cc
  robot.hh
  robotSK.cc
  robot.impl.cc
  robot.impl.hh
  server.cc
  ${CMAKE_CURRENT_BINARY_DIR}/hpp/corbaserver/manipulation/robot_idl.py
  ${CMAKE_CURRENT_BINARY_DIR}/hpp/corbaserver/manipulation/common_idl.py
)

PKG_CONFIG_USE_DEPENDENCY(${LIBRARY_NAME} hpp-manipulation)
PKG_CONFIG_USE_DEPENDENCY(${LIBRARY_NAME} omniORB4)

INSTALL(TARGETS ${LIBRARY_NAME} DESTINATION lib)

# search for python
FINDPYTHON()

# Generate and install python code for each idl file
FOREACH (IDL ${IDL_SOURCES})
  ADD_CUSTOM_COMMAND(
    OUTPUT
    ${CMAKE_CURRENT_BINARY_DIR}/hpp/corbaserver/manipulation/${IDL}_idl.py
    COMMAND ${OMNIIDL}
    ARGS -bpython -Wbpackage="" -I${CMAKE_SOURCE_DIR}/idl
    -I${HPP_CORBASERVER_DATAROOTDIR}/idl
    ${CMAKE_SOURCE_DIR}/idl/hpp/corbaserver/manipulation/${IDL}.idl
    MAIN_DEPENDENCY
    ${CMAKE_SOURCE_DIR}/idl/hpp/corbaserver/manipulation/${IDL}.idl
    )
  SET_SOURCE_FILES_PROPERTIES(
    ${CMAKE_CURRENT_BINARY_DIR}/${IDL}_idl.py
    GENERATED=ON
    )
  INSTALL(
    FILES
    ${CMAKE_CURRENT_BINARY_DIR}/${IDL}_idl.py
    DESTINATION ${PYTHON_SITELIB}/hpp/corbaserver/manipulation
    )
ENDFOREACH()

# Generate and install python code for each dependency idl file
FOREACH (IDL ${HPP_CORBASERVER_IDL_SOURCES})
  ADD_CUSTOM_COMMAND(
    OUTPUT
    ${CMAKE_CURRENT_BINARY_DIR}/hpp/corbaserver/manipulation/${IDL}_idl.py
    COMMAND ${OMNIIDL}
    ARGS -bpython -Wbpackage=""
    -I${HPP_CORBASERVER_DATAROOTDIR}/idl
    ${HPP_CORBASERVER_DATAROOTDIR}/idl/hpp/corbaserver/${IDL}.idl
    MAIN_DEPENDENCY
    ${HPP_CORBASERVER_DATAROOTDIR}/idl/hpp/corbaserver/${IDL}.idl
    )
  SET_SOURCE_FILES_PROPERTIES(
    ${CMAKE_CURRENT_BINARY_DIR}/${IDL}_idl.py
    GENERATED=ON
    )
  INSTALL(
    FILES
    ${CMAKE_CURRENT_BINARY_DIR}/${IDL}_idl.py
    DESTINATION ${PYTHON_SITELIB}/hpp/corbaserver/manipulation
    )
ENDFOREACH()

INSTALL(
  FILES
  ${CMAKE_CURRENT_SOURCE_DIR}/hpp/corbaserver/manipulation/__init__.py
  DESTINATION ${PYTHON_SITELIB}/hpp/corbaserver/manipulation
  )
INSTALL(
  FILES
  ${CMAKE_CURRENT_SOURCE_DIR}/hpp/corbaserver/manipulation/client.py
  ${CMAKE_CURRENT_SOURCE_DIR}/hpp/corbaserver/manipulation/srdf_parser.py
  ${CMAKE_CURRENT_SOURCE_DIR}/hpp/corbaserver/manipulation/problem_solver.py
  DESTINATION ${PYTHON_SITELIB}/hpp/corbaserver/manipulation
  )

# Stand alone corba server
ADD_EXECUTABLE (hpp-manipulation-server hpp-manipulation-corba.cc)
TARGET_LINK_LIBRARIES (hpp-manipulation-server ${LIBRARY_NAME})
PKG_CONFIG_USE_DEPENDENCY (hpp-manipulation-server hpp-manipulation)
PKG_CONFIG_USE_DEPENDENCY (hpp-manipulation-server hpp-wholebody-step-corba)

INSTALL (TARGETS hpp-manipulation-server DESTINATION ${CMAKE_INSTALL_BINDIR})
