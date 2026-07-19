#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <memory>
#include <optional>
#include <random>
#include <string>
#include <unordered_map>
#include <vector>

#include <ompl/base/StateSpace.h>
#include <ompl/base/spaces/SO2StateSpace.h>

#include "two_dof_planar_arm.hpp"
#include "ompl_planner.hpp"
#include "fcl_collision_checker.hpp"

namespace ob = ompl::base;
namespace fs = std::filesystem;
using motion_planning_examples::TwoDOFPlanarArm;
using motion_planning_examples::FCLCollisionChecker;
using motion_planning_examples::OMPLPlanner;
using motion_planning_examples::RRTStarSettings;
using motion_planning_examples::SquareObstacle;

namespace
{
struct Config
{
    double solveTime{3.0};
    double range{0.30};
    double goalBias{0.05};
    double rewireFactor{1.1};

    int pathInterpolationPoints{220};
    int collisionGridResolution{120};
    unsigned int randomSeed{7};

    int obstacleCount{4};
    double obstacleMinSize{0.20};
    double obstacleMaxSize{0.38};
    double workspaceMin{-1.6};
    double workspaceMax{1.6};

    double link1Length{1.0};
    double link2Length{0.9};
    double linkThickness{0.08};
    double objectHeight{0.20};

    // Note: Rendering configurations (radii, video params, output names) 
    // have been entirely deferred to the Python visualization script.

    double startTheta1{0.20};
    double startTheta2{-0.60};
    double goalTheta1{2.40};
    double goalTheta2{1.00};
};

std::string trim(const std::string &s)
{
    const auto begin = s.find_first_not_of(" \t\r\n");
    if (begin == std::string::npos)
    {
        return "";
    }
    const auto end = s.find_last_not_of(" \t\r\n");
    return s.substr(begin, end - begin + 1);
}

std::optional<std::pair<double, double>> parseAnglePair(const std::string &value)
{
    std::string s = trim(value);
    if (s.size() < 5 || s.front() != '[' || s.back() != ']')
    {
        return std::nullopt;
    }

    s = s.substr(1, s.size() - 2);
    const auto comma = s.find(',');
    if (comma == std::string::npos)
    {
        return std::nullopt;
    }

    try
    {
        const double a = std::stod(trim(s.substr(0, comma)));
        const double b = std::stod(trim(s.substr(comma + 1)));
        return std::make_pair(a, b);
    }
    catch (...)
    {
        return std::nullopt;
    }
}

std::unordered_map<std::string, std::string> loadSimpleYaml(const fs::path &path)
{
    std::ifstream in(path);
    if (!in)
    {
        throw std::runtime_error("Could not open config file: " + path.string());
    }

    std::unordered_map<std::string, std::string> kv;
    std::string line;
    while (std::getline(in, line))
    {
        const auto commentPos = line.find('#');
        if (commentPos != std::string::npos)
        {
            line = line.substr(0, commentPos);
        }

        line = trim(line);
        if (line.empty())
        {
            continue;
        }

        const auto colonPos = line.find(':');
        if (colonPos == std::string::npos)
        {
            continue;
        }

        const std::string key = trim(line.substr(0, colonPos));
        const std::string value = trim(line.substr(colonPos + 1));
        if (!key.empty() && !value.empty())
        {
            kv[key] = value;
        }
    }

    return kv;
}

Config loadConfig(const fs::path &configPath)
{
    Config cfg;
    const auto kv = loadSimpleYaml(configPath);

    auto readDouble = [&](const char *key, double &target) {
        const auto it = kv.find(key);
        if (it != kv.end())
        {
            target = std::stod(it->second);
        }
    };

    auto readInt = [&](const char *key, int &target) {
        const auto it = kv.find(key);
        if (it != kv.end())
        {
            target = std::stoi(it->second);
        }
    };

    auto readUnsigned = [&](const char *key, unsigned int &target) {
        const auto it = kv.find(key);
        if (it != kv.end())
        {
            target = static_cast<unsigned int>(std::stoul(it->second));
        }
    };

    readDouble("solve_time", cfg.solveTime);
    readDouble("range", cfg.range);
    readDouble("goal_bias", cfg.goalBias);
    readDouble("rewire_factor", cfg.rewireFactor);

    readInt("path_interpolation_points", cfg.pathInterpolationPoints);
    readInt("collision_grid_resolution", cfg.collisionGridResolution);
    readUnsigned("random_seed", cfg.randomSeed);

    readInt("obstacle_count", cfg.obstacleCount);
    readDouble("obstacle_min_size", cfg.obstacleMinSize);
    readDouble("obstacle_max_size", cfg.obstacleMaxSize);
    readDouble("workspace_min", cfg.workspaceMin);
    readDouble("workspace_max", cfg.workspaceMax);

    readDouble("link1_length", cfg.link1Length);
    readDouble("link2_length", cfg.link2Length);
    readDouble("link_thickness", cfg.linkThickness);
    readDouble("object_height", cfg.objectHeight);

    const auto startIt = kv.find("start");
    if (startIt != kv.end())
    {
        if (const auto parsed = parseAnglePair(startIt->second))
        {
            cfg.startTheta1 = parsed->first;
            cfg.startTheta2 = parsed->second;
        }
    }

    const auto goalIt = kv.find("goal");
    if (goalIt != kv.end())
    {
        if (const auto parsed = parseAnglePair(goalIt->second))
        {
            cfg.goalTheta1 = parsed->first;
            cfg.goalTheta2 = parsed->second;
        }
    }

    cfg.pathInterpolationPoints = std::max(2, cfg.pathInterpolationPoints);
    cfg.collisionGridResolution = std::max(8, cfg.collisionGridResolution);
    cfg.obstacleCount = std::max(0, cfg.obstacleCount);
    cfg.obstacleMinSize = std::max(0.02, cfg.obstacleMinSize);
    cfg.obstacleMaxSize = std::max(cfg.obstacleMinSize, cfg.obstacleMaxSize);
    cfg.objectHeight = std::max(0.01, cfg.objectHeight);
    cfg.linkThickness = std::max(0.01, cfg.linkThickness);

    return cfg;
}

std::vector<SquareObstacle> generateObstacles(const Config &cfg)
{
    std::mt19937 rng(cfg.randomSeed);
    std::uniform_real_distribution<double> sideDist(cfg.obstacleMinSize, cfg.obstacleMaxSize);

    std::vector<SquareObstacle> obstacles;
    obstacles.reserve(static_cast<std::size_t>(cfg.obstacleCount));

    for (int i = 0; i < cfg.obstacleCount; ++i)
    {
        const double side = sideDist(rng);
        const double low = cfg.workspaceMin + 0.5 * side;
        const double high = cfg.workspaceMax - 0.5 * side;
        if (low >= high)
        {
            break;
        }

        std::uniform_real_distribution<double> centerDist(low, high);
        SquareObstacle obstacle;
        obstacle.cx = centerDist(rng);
        obstacle.cy = centerDist(rng);
        obstacle.size = side;
        obstacles.push_back(obstacle);
    }

    return obstacles;
}

std::optional<fs::path> firstExistingPath(const std::vector<fs::path> &candidates)
{
    for (const auto &candidate : candidates)
    {
        if (!candidate.empty() && fs::exists(candidate))
        {
            return candidate;
        }
    }
    return std::nullopt;
}

void writeObstacleCsv(const fs::path &path, const std::vector<SquareObstacle> &obstacles)
{
    std::ofstream out(path);
    out << "cx,cy,size\n";
    for (const auto &obstacle : obstacles)
    {
        out << obstacle.cx << ',' << obstacle.cy << ',' << obstacle.size << '\n';
    }
}

void writePathCsv(const fs::path &path, const std::vector<std::pair<double, double>> &pathAngles)
{
    std::ofstream out(path);
    out << "theta1,theta2\n";

    for (const auto &angles : pathAngles)
    {
        out << angles.first << ',' << angles.second << '\n';
    }
}

void writeCollisionMapCsv(const fs::path &path,
                          const std::shared_ptr<ob::StateSpace> &space,
                          const FCLCollisionChecker &checker,
                          int gridResolution)
{
    std::ofstream out(path);
    out << "theta1,theta2,valid\n";

    const double pi = std::acos(-1.0);

    ob::State *state = space->allocState();
    auto *compound = state->as<ob::CompoundStateSpace::StateType>();

    for (int i = 0; i < gridResolution; ++i)
    {
        const double theta1 = -pi + (2.0 * pi * i) / static_cast<double>(gridResolution - 1);
        for (int j = 0; j < gridResolution; ++j)
        {
            const double theta2 = -pi + (2.0 * pi * j) / static_cast<double>(gridResolution - 1);
            compound->as<ob::SO2StateSpace::StateType>(0)->value = theta1;
            compound->as<ob::SO2StateSpace::StateType>(1)->value = theta2;

            const bool valid = checker.isStateValid(state);
            out << theta1 << ',' << theta2 << ',' << (valid ? 1 : 0) << '\n';
        }
    }

    space->freeState(state);
}

}  // namespace

int main(int argc, char **argv)
{
    const auto configPath = firstExistingPath({
        argc > 1 ? fs::path(argv[1]) : fs::path(),
        "rrtstar_config.yaml",
        "../rrtstar_config.yaml",
    });

    if (!configPath)
    {
        std::cerr << "Could not find rrtstar_config.yaml. Pass a path as first argument.\n";
        return 1;
    }

    Config cfg;
    try
    {
        cfg = loadConfig(*configPath);
    }
    catch (const std::exception &e)
    {
        std::cerr << "Config parse error: " << e.what() << '\n';
        return 1;
    }

    const auto obstacles = generateObstacles(cfg);

    auto planar_arm = std::make_shared<TwoDOFPlanarArm>(
        cfg.link1Length, cfg.link2Length, cfg.linkThickness, cfg.objectHeight);

    FCLCollisionChecker checker(planar_arm, cfg.objectHeight, obstacles);

    OMPLPlanner planner(planar_arm->getStateSpace());
    
    planner.setStateValidityChecker([&checker](const ob::State *state) {
        return checker.isStateValid(state);
    });
    
    planner.setStartGoal(cfg.startTheta1, cfg.startTheta2, cfg.goalTheta1, cfg.goalTheta2);

    RRTStarSettings settings;
    settings.range = cfg.range;
    settings.goalBias = cfg.goalBias;
    settings.rewireFactor = cfg.rewireFactor;
    settings.pathInterpolationPoints = cfg.pathInterpolationPoints;
    planner.configureRRTStar(settings);

    const bool solved = planner.solve(cfg.solveTime);
    if (!solved)
    {
        std::cout << "No solution found.\n";
        return 1;
    }

    std::cout << "Solution found! Smoothing path...\n";
    planner.simplifyPath(1.5);

    const auto pathAngles = planner.getInterpolatedPath(cfg.pathInterpolationPoints);

    const fs::path pathCsv = "path_angles.csv";
    const fs::path collisionCsv = "collision_map.csv";
    const fs::path obstaclesCsv = "obstacles.csv";

    writePathCsv(pathCsv, pathAngles);
    writeCollisionMapCsv(collisionCsv, planner.getStateSpace(), checker, cfg.collisionGridResolution);
    writeObstacleCsv(obstaclesCsv, obstacles);

    std::cout << "Found solution with " << pathAngles.size() << " states.\n";
    std::cout << "Path length in configuration space: " << planner.getPathLength() << "\n";
    std::cout << "Successfully generated: " << pathCsv << ", " << collisionCsv << ", and " << obstaclesCsv << '\n';
    
    return 0;
}
