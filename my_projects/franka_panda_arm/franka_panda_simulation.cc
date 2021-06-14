/// @file
///
/// Franka Panda simulation

#include <limits>
#include <memory>

#include <gflags/gflags.h>

#include "drake/common/drake_assert.h"
#include "drake/common/find_resource.h"
#include "drake/common/text_logging.h"
#include "drake/common/type_safe_index.h"
#include "drake/geometry/drake_visualizer.h"
#include "drake/geometry/scene_graph.h"
#include "drake/lcm/drake_lcm.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/multibody/tree/weld_joint.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/primitives/constant_vector_source.h"
#include "drake/systems/controllers/inverse_dynamics_controller.h"

using Eigen::VectorXd;

using drake::geometry::SceneGraph;
using drake::lcm::DrakeLcm;
using drake::multibody::Body;
using drake::multibody::MultibodyPlant;
using drake::multibody::Parser;

DEFINE_double(simulation_time, std::numeric_limits<double>::infinity(),
              "Desired duration of the simulation in seconds");
DEFINE_double(max_time_step, 1.0e-3,
              "Simulation time step used for integrator.");
DEFINE_double(target_realtime_rate, 1,
              "Desired rate relative to real time.  See documentation for "
              "Simulator::set_target_realtime_rate() for details.");

DEFINE_double(Kp, 10.0, "Kp");
DEFINE_double(Ki, 0.1, "Ki");
DEFINE_double(Kd, 5.0, "Kd");

DEFINE_bool(add_gravity, true,
            "Indicator for whether terrestrial gravity"
            " (9.81 m/s²) is included or not.");

const char kUrdfPath[] =
    "drake/manipulation/models/"
    "panda_description/urdf/panda_arm_custom.urdf";

namespace drake {
namespace my_projects {
namespace franka_panda_arm {
namespace {

int DoMain() {
  systems::DiagramBuilder<double> builder;

  SceneGraph<double>* scene_graph = builder.AddSystem<SceneGraph>();

  // Add plant
  MultibodyPlant<double>* panda_plant =
      builder.AddSystem<MultibodyPlant>(FLAGS_max_time_step);
  panda_plant->RegisterAsSourceForSceneGraph(scene_graph);

  // Load URDF
  const std::string urdf = FindResourceOrThrow(kUrdfPath);
  const multibody::ModelInstanceIndex panda_id =
      Parser(panda_plant, scene_graph).AddModelFromFile(
          urdf);

  // Weld base link to world frame
  panda_plant->WeldFrames(panda_plant->world_frame(),
                          panda_plant->GetFrameByName("base_link"));

  // Plant is complete
  panda_plant->Finalize();

  // Connect plant with scene_graph to get collision information.
  builder.Connect(
      panda_plant->get_geometry_poses_output_port(),
      scene_graph->get_source_pose_port(
      panda_plant->get_source_id().value()));
  builder.Connect(
      scene_graph->get_query_output_port(),
      panda_plant->get_geometry_query_input_port());

  // Defind desired states, here we choose 0 positions and 0 velocities.
  VectorX<double> desired_state =
      VectorX<double>::Zero(panda_plant->num_multibody_states());
  desired_state(2) = 1.0;
  auto desired_state_source =
      builder.AddSystem<systems::ConstantVectorSource<double>>(desired_state);
  desired_state_source->set_name("constant_source");

  // Constant zero load for plant actuation_input_port.
  auto constant_zero_torque =
      builder.AddSystem<systems::ConstantVectorSource<double>>(
          Eigen::VectorXd::Zero(panda_plant->num_actuators()));
  builder.Connect(constant_zero_torque->get_output_port(),
                  panda_plant->get_actuation_input_port());

  drake::log()->info(
      "positions numbers: " + std::to_string(panda_plant->num_positions()) +
      ", velocities numbers: " + std::to_string(panda_plant->num_velocities()) +
      ", actuators numbers: " + std::to_string(panda_plant->num_actuators())
    );

  drake::log()->info("Desired joint state: ");
  drake::log()->info(desired_state.transpose());

  // Create inverse dynamics controller.
  const int num_actuators = panda_plant->num_actuators();
  auto IDC = builder.AddSystem<systems::controllers::InverseDynamicsController<double>>(
              *panda_plant, Eigen::VectorXd::Ones(num_actuators) * FLAGS_Kp,
              Eigen::VectorXd::Ones(num_actuators) * FLAGS_Ki,
              Eigen::VectorXd::Ones(num_actuators) * FLAGS_Kd, false);
  builder.Connect(IDC->get_output_port_control(),
                  panda_plant->get_applied_generalized_force_input_port());
  builder.Connect(panda_plant->get_state_output_port(),
                  IDC->get_input_port_estimated_state());
  builder.Connect(desired_state_source->get_output_port(),
                  IDC->get_input_port_desired_state());


  // Add visualizer
  geometry::DrakeVisualizerd::AddToBuilder(&builder, *scene_graph);

  std::unique_ptr<systems::Diagram<double>> diagram = builder.Build();
  // Create a context for this system, with default values for all systems
  std::unique_ptr<systems::Context<double>> diagram_context =
      diagram->CreateDefaultContext();
  diagram->SetDefaultContext(diagram_context.get());

  systems::Context<double>& plant_context =
      diagram->GetMutableSubsystemContext(*panda_plant, diagram_context.get());
  Eigen::VectorXd q = panda_plant->GetPositions(plant_context, panda_id);
    
  // Set up simulator
  systems::Simulator<double> simulator(*diagram);
  simulator.set_publish_every_time_step(false);
  simulator.set_target_realtime_rate(FLAGS_target_realtime_rate);
  simulator.Initialize();
  simulator.AdvanceTo(FLAGS_simulation_time);
  return 0;

} // DoMain()

} // namespace
} // franka_panda_arm
} // my_projects
} // drake

int main(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  return drake::my_projects::franka_panda_arm::DoMain();
}
