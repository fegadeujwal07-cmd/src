#include <cstdio>
#include <fstream>
#include <regex>
#include <sstream>
#include <stdexcept>
#include <string>

#include <gtest/gtest.h>

#include "ecbs_planner/ecbs_solver.hpp"

namespace
{

/// test/ → package root → config/ecbs_params.yaml
std::string ecbs_params_yaml_path()
{
  return std::string(__FILE__).substr(0, std::string(__FILE__).find_last_of('/')) +
         "/../config/ecbs_params.yaml";
}

std::string read_file(const std::string & path)
{
  std::ifstream f(path);
  EXPECT_TRUE(f.good()) << "cannot open " << path;
  return std::string(
    (std::istreambuf_iterator<char>(f)), std::istreambuf_iterator<char>());
}

/// evaluate_path_server.ros__parameters.<key> as written in the YAML text
/// (independent of yaml-cpp, so the loader is checked against the raw file).
double yaml_param(const std::string & text, const std::string & key)
{
  const std::regex re("^\\s*" + key + ":\\s*(-?[0-9]+(?:\\.[0-9]+)?)\\s*(#.*)?$");
  std::smatch m;
  std::string line;
  std::istringstream in(text);
  while (std::getline(in, line)) {
    if (std::regex_match(line, m, re)) {
      return std::stod(m[1].str());
    }
  }
  ADD_FAILURE() << "missing yaml key: " << key;
  return 0.0;
}

}  // namespace

// The running system takes every SolverConfig value from ecbs_params.yaml
// (evaluate_path_server loads it with load_solver_config_yaml). Every field
// must be present in the YAML and come back exactly as written.
TEST(SolverConfigDefaults, LoadedFromEcbsParamsYaml)
{
  const auto path = ecbs_params_yaml_path();
  const auto text = read_file(path);
  ASSERT_FALSE(text.empty());
  const ecbs_planner::SolverConfig cfg = ecbs_planner::load_solver_config_yaml(path);

  EXPECT_DOUBLE_EQ(cfg.safety_distance_m, yaml_param(text, "safety_distance_m"));
  EXPECT_DOUBLE_EQ(cfg.clearance_m, yaml_param(text, "clearance_m"));
  EXPECT_DOUBLE_EQ(cfg.amr_length_m, yaml_param(text, "amr_length_m"));
  EXPECT_DOUBLE_EQ(cfg.amr_width_m, yaml_param(text, "amr_width_m"));
  EXPECT_DOUBLE_EQ(cfg.position_epsilon_m, yaml_param(text, "position_epsilon_m"));
  EXPECT_DOUBLE_EQ(
    cfg.evaluate_lookahead_margin_m,
    yaml_param(text, "evaluate_lookahead_margin_m"));
  EXPECT_DOUBLE_EQ(cfg.safety_margin_sec, yaml_param(text, "safety_margin_sec"));
  EXPECT_DOUBLE_EQ(cfg.max_wait_sec, yaml_param(text, "max_wait_sec"));
  EXPECT_EQ(cfg.max_iters, static_cast<int>(yaml_param(text, "max_iters")));
  EXPECT_DOUBLE_EQ(cfg.junction_release_m, yaml_param(text, "junction_release_m"));
}

// A key missing from the file keeps the fallback value; the rest are read.
TEST(SolverConfigDefaults, MissingKeyKeepsFallback)
{
  const std::string path = ::testing::TempDir() + "ecbs_params_partial.yaml";
  {
    std::ofstream f(path);
    f << "evaluate_path_server:\n  ros__parameters:\n    clearance_m: 0.25\n    max_iters: 7\n";
  }
  ecbs_planner::SolverConfig fallback;
  fallback.safety_distance_m = 4.5;
  const auto cfg = ecbs_planner::load_solver_config_yaml(path, fallback);
  EXPECT_DOUBLE_EQ(cfg.clearance_m, 0.25);
  EXPECT_EQ(cfg.max_iters, 7);
  EXPECT_DOUBLE_EQ(cfg.safety_distance_m, 4.5);
  std::remove(path.c_str());
}

// No params block / unreadable file is an error, not silent defaults.
TEST(SolverConfigDefaults, BadFileThrows)
{
  EXPECT_THROW(
    ecbs_planner::load_solver_config_yaml("/nonexistent/ecbs_params.yaml"),
    std::runtime_error);
  const std::string path = ::testing::TempDir() + "ecbs_params_wrong.yaml";
  {
    std::ofstream f(path);
    f << "some_other_node:\n  ros__parameters:\n    clearance_m: 1.0\n";
  }
  EXPECT_THROW(ecbs_planner::load_solver_config_yaml(path), std::runtime_error);
  std::remove(path.c_str());
}
