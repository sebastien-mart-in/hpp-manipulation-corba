// Copyright (C) 2019 by Joseph Mirabel, LAAS-CNRS.
//
// This file is part of the hpp-corbaserver.
//
// This software is provided "as is" without warranty of any kind,
// either expressed or implied, including but not limited to the
// implied warranties of fitness for a particular purpose.
//
// See the COPYING file for more information.

#include "hpp/manipulation_idl/steering-methods.hh"

namespace hpp
{
  namespace corbaServer
  {
    namespace manipulation_idl
    {
      HPP_CORBASERVER_ADD_DOWNCAST_OBJECT(EndEffectorTrajectory, core_idl::SteeringMethod, 1)
    } // end of namespace manipulation.
  } // end of namespace corbaServer.
} // end of namespace hpp.
