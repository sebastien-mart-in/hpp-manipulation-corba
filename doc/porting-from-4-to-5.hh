/// \page hpp_manipulation_corba_porting_notes Porting notes
///
/// \section hpp_manipulation_corba_porting_4_to_5 Porting your code from version 4 to version 5
/// \subsection hpp_manipulation_corba_porting_4_to_5_1 Modification in idl API
///
/// \subsubsection hpp_manipulation_corba_porting_4_to_5_1_1 Methods
/// hpp::corbaserver::manipulation::Robot::insertRobotModel,
/// hpp::corbaserver::manipulation::Robot::insertRobotModelOnFrame, and
/// hpp::corbaserver::manipulation::Robot::insertHumanoidModel
///
/// Arguments
/// \li in string packageName, in string modelName, in string urdfSuffix, in string srdfSuffix have been replaced by
/// \li in string urdfname, in string srdfname.
///
/// These latter arguments contain the full filename to the urdf and srdf files.
/// They may include "package://" pattern.
///
/// \subsection hpp_manipulation_corba_porting_4_to_5_2 Modification in python API
/// \subsubsection hpp_manipulation_corba_porting_4_to_5_2_1 Methods
///
/// \li insertRobotModel,
/// \li insertRobotModelOnFrame,
/// \li insertHumanoidModel,
/// \li loadHumanoidModel, and
/// \li loadEnvironmentModel
///
/// of class manipulation.robot.Robot
///
/// Arguments
/// \li packageName, modelName, urdfSuffix, srdfSuffix have been replaced by
/// \li urdfName, srdfName.
///

