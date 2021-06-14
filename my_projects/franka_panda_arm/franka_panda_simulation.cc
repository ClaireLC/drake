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

using Eigen::VectorXd;

using drake::geometry::SceneGraph;
using drake::lcm::DrakeLcm;
using drake::multibody::Body;
using drake::multibody::MultibodyPlant;
using drake::multibody::Parser;

DEFINE_double(simulation_time, 5,
              "Desired duration of the simulation in seconds");
DEFINE_double(max_time_step, 1.0e-3,
              "Simulation time step used for integrator.");
DEFINE_double(target_realtime_rate, 1,
              "Desired rate relative to real time.  See documentation for "
              "Simulator::set_target_realtime_rate() for details.");

DEFINE_bool(add_gravity, true,
            "Indicator for whether terrestrial gravity"
            " (9.81 m/s²) is included or not.");

const char kUrdfPath[] =
    "drake/manipulation/models/"
    "franka_description/urdf/panda_arm_custom.urdf";

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
