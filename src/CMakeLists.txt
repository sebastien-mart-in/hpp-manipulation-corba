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
  manipulation
)

OMNIIDL_INCLUDE_DIRECTORIES(
  ${CMAKE_SOURCE_DIR}/idl
  ${HPP_CORBASERVER_DATAROOTDIR}/idl
  )
INCLUDE_DIRECTORIES(${CMAKE_BINARY_DIR}/src)
FOREACH(IDL ${IDL_SOURCES})
  GENERATE_IDL_FILE (${IDL}
    ${CMAKE_SOURCE_DIR}/idl/hpp/corbaserver/manipulation
    )
ENDFOREACH()

ADD_LIBRARY(${LIBRARY_NAME} SHARED
  manipulation.hh
  manipulationSK.cc
  manipulation.impl.cc
  manipulation.impl.hh
  server.cc
  ${CMAKE_CURRENT_BINARY_DIR}/hpp/corbaserver/manipulation/manipulation_idl.py
)

PKG_CONFIG_USE_DEPENDENCY(${LIBRARY_NAME} hpp-manipulation)
PKG_CONFIG_USE_DEPENDENCY(${LIBRARY_NAME} omniORB4)

INSTALL(TARGETS ${LIBRARY_NAME} DESTINATION lib)

# search for python
FINDPYTHON()

FOREACH (IDL ${IDL_SOURCES})
  ADD_CUSTOM_COMMAND(
    OUTPUT
    ${CMAKE_CURRENT_BINARY_DIR}/hpp/corbaserver/manipulation/${IDL}_idl.py
    COMMAND ${OMNIIDL}
    ARGS -bpython -Wbpackage="hpp.corbaserver.manipulation"
    -I${CMAKE_SOURCE_DIR}/idl
    ${CMAKE_SOURCE_DIR}/idl/hpp/corbaserver/manipulation/${IDL}.idl
    MAIN_DEPENDENCY
    ${CMAKE_SOURCE_DIR}/idl/hpp/corbaserver/manipulation/${IDL}.idl
    )
  SET_SOURCE_FILES_PROPERTIES(
    ${CMAKE_CURRENT_BINARY_DIR}/hpp/corbaserver/manipulation/${IDL}_idl.py
    GENERATED=ON
    )
  INSTALL(
    FILES
    ${CMAKE_CURRENT_BINARY_DIR}/hpp/corbaserver/manipulation/${IDL}_idl.py
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
  ${CMAKE_CURRENT_BINARY_DIR}/hpp/corbaserver/manipulation/hpp/__init__.py
  DESTINATION ${PYTHON_SITELIB}/hpp/corbaserver/manipulation/hpp
  )
INSTALL(
  FILES
  ${CMAKE_CURRENT_BINARY_DIR}/hpp/corbaserver/manipulation/hpp__POA/__init__.py
  DESTINATION ${PYTHON_SITELIB}/hpp/corbaserver/manipulation/hpp__POA
  )
INSTALL(
  FILES
  ${CMAKE_CURRENT_SOURCE_DIR}/hpp/corbaserver/manipulation/client.py
  DESTINATION ${PYTHON_SITELIB}/hpp/corbaserver/manipulation
  )

# Stand alone corba server
ADD_EXECUTABLE (hpp-manipulation-server hpp-manipulation-corba.cc)
TARGET_LINK_LIBRARIES (hpp-manipulation-server ${LIBRARY_NAME})
PKG_CONFIG_USE_DEPENDENCY (hpp-manipulation-server hpp-manipulation)
PKG_CONFIG_USE_DEPENDENCY (hpp-manipulation-server hpp-wholebody-step-corba)

INSTALL (TARGETS hpp-manipulation-server DESTINATION ${CMAKE_INSTALL_BINDIR})
