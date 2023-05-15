/*
 * Copyright 2016 The Cartographer Authors
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "absl/memory/memory.h"
#include "cartographer/mapping/map_builder.h"
#include "cartographer_ros/node.h"
#include "cartographer_ros/node_options.h"
#include "cartographer_ros/ros_log_sink.h"
#include "gflags/gflags.h"
#include "tf2_ros/transform_listener.h"
#include "time.h"

DEFINE_bool(collect_metrics, false,
            "Activates the collection of runtime metrics. If activated, the "
            "metrics can be accessed via a ROS service.");
DEFINE_string(configuration_directory, "",
              "First directory in which configuration files are searched, "
              "second is always the Cartographer installation to allow "
              "including files from there.");
DEFINE_string(configuration_basename, "",
              "Basename, i.e. not containing any directory prefix, of the "
              "configuration file.");
DEFINE_string(load_state_filename, "",
              "If non-empty, filename of a .pbstream file to load, containing "
              "a saved SLAM state.");
DEFINE_bool(load_frozen_state, true,
            "Load the saved state as frozen (non-optimized) trajectories.");
DEFINE_bool(
    start_trajectory_with_default_topics, true,
    "Enable to immediately start the first trajectory with default topics.");
DEFINE_string(
    save_state_filename, "",
    "If non-empty, serialize state and write it to disk before shutting down.");

namespace cartographer_ros {
namespace {

void Run() {
  constexpr double kTfBufferCacheTimeInSeconds = 10.;
  tf2_ros::Buffer tf_buffer{::ros::Duration(kTfBufferCacheTimeInSeconds)};
  tf2_ros::TransformListener tf(tf_buffer);
  NodeOptions node_options;
  TrajectoryOptions trajectory_options;

  std::clock_t t0, t1, t2, t3, t4, t5, t6, t7, t8;
  t0 = clock();

  std::tie(node_options, trajectory_options) =
      LoadOptions(FLAGS_configuration_directory, FLAGS_configuration_basename);

  t1 = clock();

  double delta_t1 = (double)(t1 - t0) / CLOCKS_PER_SEC;
  std::cout << "LoadOptions delta_t1 = " << delta_t1 << std::endl;

  auto map_builder =
      cartographer::mapping::CreateMapBuilder(node_options.map_builder_options);
  
  t2 = clock();
  double delta_t2 = (double)(t2 - t1) / CLOCKS_PER_SEC;
  std::cout << "CreateMapBuilder delta_t2 = " << delta_t2 << std::endl;

  Node node(node_options, std::move(map_builder), &tf_buffer,
            FLAGS_collect_metrics);

  t3 = clock();
  double delta_t3 = (double)(t3 - t2) / CLOCKS_PER_SEC;
  std::cout << "node delta_t3 = " << delta_t3 << std::endl;


  if (!FLAGS_load_state_filename.empty()) {
    node.LoadState(FLAGS_load_state_filename, FLAGS_load_frozen_state);
  }

  t4 = clock();
  double delta_t4 = (double)(t4 - t3) / CLOCKS_PER_SEC;
  std::cout << "LoadState delta_t4 = " << delta_t4 << std::endl;

  if (FLAGS_start_trajectory_with_default_topics) {
    node.StartTrajectoryWithDefaultTopics(trajectory_options);
  }

  t5 = clock();
  double delta_t5 = (double)(t5 - t4) / CLOCKS_PER_SEC;
  std::cout << "StartTrajectoryWithDefaultTopics delta_t5 = " << delta_t5 << std::endl;

  ::ros::spin();

  node.FinishAllTrajectories();

 t6 = clock();
  double delta_t6 = (double)(t6 - t5) / CLOCKS_PER_SEC;
  std::cout << "FinishAllTrajectories delta_t6 = " << delta_t6 << std::endl;

  node.RunFinalOptimization();
  t7 = clock();
  double delta_t7 = (double)(t7 - t6) / CLOCKS_PER_SEC;
  std::cout << "RunFinalOptimization delta_t7 = " << delta_t7 << std::endl;


  if (!FLAGS_save_state_filename.empty()) {
    node.SerializeState(FLAGS_save_state_filename,
                        true /* include_unfinished_submaps */);
  }

  t8 = clock();
  double delta_t8 = (double)(t8 - t0) / CLOCKS_PER_SEC;
  std::cout << "SerializeState delta_t8 = " << delta_t8 << std::endl;

  // LOG(INFO) << "delta_t1 = '" << delta_t1 << "'...";
  // std::cout << std::endl;

}

}  // namespace
}  // namespace cartographer_ros

int main(int argc, char** argv) {
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, true);

  CHECK(!FLAGS_configuration_directory.empty())
      << "-configuration_directory is missing.";
  CHECK(!FLAGS_configuration_basename.empty())
      << "-configuration_basename is missing.";

  ::ros::init(argc, argv, "cartographer_node");
  ::ros::start();

  cartographer_ros::ScopedRosLogSink ros_log_sink;
  cartographer_ros::Run();
  ::ros::shutdown();
}
