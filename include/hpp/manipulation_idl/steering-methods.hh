// Copyright (C) 2019 by Joseph Mirabel, LAAS-CNRS.
//
// This file is part of the hpp-corbaserver.
//
// This software is provided "as is" without warranty of any kind,
// either expressed or implied, including but not limited to the
// implied warranties of fitness for a particular purpose.
//
// See the COPYING file for more information.

#ifndef HPP_MANIPULATION_IDL_STEERING_METHODS_HH
# define HPP_MANIPULATION_IDL_STEERING_METHODS_HH

# include <vector>
# include <stdlib.h>

# include <hpp/manipulation/steering-method/end-effector-trajectory.hh>

# include <hpp/corbaserver/fwd.hh>
# include <hpp/corbaserver/conversions.hh>
# include <hpp/constraints_idl/constraints.hh>
# include <hpp/core_idl/steering_methods.hh>
# include <hpp/manipulation_idl/steering_methods-idl.hh>

# include "hpp/corbaserver/servant-base.hh"

namespace hpp
{
  namespace corbaServer
  {
    namespace manipulation_impl
    {
      template <typename _Base, typename _Storage>
      class EndEffectorTrajectoryServant : public core_impl::SteeringMethodServant<_Base, _Storage>
      {
          SERVANT_BASE_TYPEDEFS(hpp::manipulation_idl::EndEffectorTrajectory, core::SteeringMethod);

        public:
          typedef core_impl::SteeringMethodServant<Base, Storage> Parent;

          EndEffectorTrajectoryServant (Server* server, const Storage& s) :
            Parent (server, s) {}

          virtual ~EndEffectorTrajectoryServant () {}

          void trajectoryConstraint (hpp::constraints_idl::Implicit_ptr c) throw (Error)
          {
            constraints::ImplicitPtr_t cc =
              reference_to_servant_base<constraints::Implicit>(server_, c)->get();
            getT()->trajectoryConstraint (cc);
          }

          void trajectory (hpp::core_idl::Path_ptr eeTraj, CORBA::Boolean se3Output) throw (Error)
          {
            core::PathPtr_t path =
              reference_to_servant_base<core::Path>(server_, eeTraj)->get();
            getT()->trajectory (path, se3Output);
          }
      };

      typedef EndEffectorTrajectoryServant<POA_hpp::manipulation_idl::EndEffectorTrajectory,
              manipulation::steeringMethod::EndEffectorTrajectoryPtr_t> EndEffectorTrajectory;
    } // end of namespace core.
  } // end of namespace corbaServer.
} // end of namespace hpp.

#endif // HPP_MANIPULATION_IDL_STEERING_METHODS_HH
