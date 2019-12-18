// Copyright (c) 2015, Joseph Mirabel
// Authors: Joseph Mirabel (joseph.mirabel@laas.fr)
//
// This file is part of hpp-manipulation-corba.
// hpp-manipulation-corba is free software: you can redistribute it
// and/or modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation, either version
// 3 of the License, or (at your option) any later version.
//
// hpp-manipulation-corba is distributed in the hope that it will be
// useful, but WITHOUT ANY WARRANTY; without even the implied warranty
// of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
// General Lesser Public License for more details.  You should have
// received a copy of the GNU Lesser General Public License along with
// hpp-manipulation-corba. If not, see <http://www.gnu.org/licenses/>.

#include "hpp/corbaserver/manipulation/client.hh"

#include <iostream>

namespace hpp
{
  namespace corbaServer
  {
    namespace manipulation
    {
      using CORBA::Exception;
      using CORBA::Object_var;
      using CORBA::SystemException;
      using CORBA::ORB_init;
      using CORBA::PolicyList;
      using omniORB::fatalException;

      Client::Client(int argc, char *argv[])
        : ClientBase (argc, argv)
      {}

      void Client::connect (const char* iiop, const char* context)
      {
        ClientBase::connect(iiop);

        CORBA::Object_var obj;
        const char* plugin = "manipulation";

        obj = tools()->getServer (context, plugin, "robot");
        robot_ = hpp::corbaserver::manipulation::Robot::_narrow(obj.in());

        obj = tools()->getServer (context, plugin, "problem");
        problem_ = hpp::corbaserver::manipulation::Problem::_narrow(obj.in());

        obj = tools()->getServer (context, plugin, "graph");
        graph_ = hpp::corbaserver::manipulation::Graph::_narrow(obj.in());
      }

      /// \brief Shutdown CORBA server
      Client::~Client()
      {
      }
    } // end of namespace manipulation.
  } // end of namespace corbaServer.
} // end of namespace hpp.
