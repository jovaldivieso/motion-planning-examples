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
#include "ceres_planner.hpp"
#include "fcl_collision_checker.hpp"

namespace ob = ompl::base;
namespace fs = std::filesystem;
using motion_planning_examples::TwoDOFPlanarArm;
using motion_planning_examples::FCLCollisionChecker;
using motion_planning_examples::OMPLPlanner;
using motion_planning_examples::CeresPlanner;
using motion_planning_examples::RRTStarSettings;
using motion_planning_examples::SquareObstacle;

namespace
{
struct Config
{
    std::string plannerType{"ompl"};

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

    double startTheta1{0.20};
    double startTheta2{-0.60};
    double goalTheta1{2.40};
    double goalTheta2{1.00};
};

std::string trim(const std::string &s)
{
    const auto begin = s.find_first_not_of(" \t\r\n");
    if (begin == std::string::npos) return "";
    const auto end = s.find_last_not_of(" \t\r\n");
    return s.substr(begin, end - begin + 1);
}

std::optional<std::pair<double, double>> parseAnglePair(const std::string &value)
{
    std::string s = trim(value);
    if (s.size() < 5 || s.front() != '[' || s.back() != ']') return std::nullopt;

    s = s.substr(1, s.size() - 2);
    const auto comma = s.find(',');
    if (comma == std::string::npos) return std::nullopt;

    try
    {
        return std::make_pair(std::stod(trim(s.substr(0, comma))),
                              std::stod(trim(s.substr(comma + 1))));
    }
    catch (...) { return std::nullopt; }
}

std::unordered_map<std::string, std::string> loadSimpleYaml(const fs::path &path)
{
    std::ifstream in(path);
    if (!in) throw std::runtime_error("Could not open config file: " + path.string());

    std::unordered_map<std::string, std::string> kv;
    std::string line;
    while (std::getline(in, line))
    {
        const auto commentPos = line.find('#');
        if (commentPos != std::string::npos) line = line.substr(0, commentPos);

        line = trim(line);
        if (line.empty()) continue;

        const auto colonPos = line.find(':');
        if (colonPos == std::string::npos) continue;

        const std::string key = trim(line.substr(0, colonPos));
        const std::string value = trim(line.substr(colonPos + 1));
        if (!key.empty() && !value.empty()) kv[key] = value;
    }
    return kv;
}

Config loadConfig(const fs::path &configPath)
{
    Config cfg;
    const auto kv = loadSimpleYaml(configPath);

    auto readString = [&](const char *key, std::string &target) {
        if (auto it = kv.find(key); it != kv.end()) target = it->second;
    };
    auto readDouble = [&](const char *key, double &target) {
        if (auto it = kv.find(key); it != kv.end()) target = std::stod(it->second);
    };
    auto readInt = [&](const char *key, int &target) {
        if (auto it = kv.find(key); it != kv.end()) target = std::stoi(it->second);
    };
    auto readUnsigned = [&](const char *key, unsigned int &target) {
        if (auto it = kv.find(key); it != kv.end()) target = static_cast<unsigned int>(std::stoul(it->second));
    };

    readString("planner", cfg.plannerType);
    std::transform(cfg.plannerType.begin(), cfg.plannerType.end(), cfg.plannerType.begin(), ::tolower);

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

    if (auto startIt = kv.find("start"); startIt != kv.end())
    {
        if (auto parsed = parseAnglePair(startIt->second))
        {
            cfg.startTheta1 = parsed->first;
            cfg.startTheta2 = parsed->second;
        }
    }

    if (auto goalIt = kv.find("goal"); goalIt != kv.end())
    {
        if (auto parsed = parseAnglePair(goalIt->second))
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
        if (low >= high) break;

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
        if (!candidate.empty() && fs::exists(candidate)) return candidate;
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
        "config.yaml",
        "../config.yaml",
    });

    if (!configPath)
    {
        std::cerr << "Could not find config.yaml. Pass a path as first argument.\n";
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

    auto checker = std::make_shared<FCLCollisionChecker>(planar_arm, cfg.objectHeight, obstacles);

    std::vector<std::pair<double, double>> pathAngles;
    double pathLength = 0.0;

    if (cfg.plannerType == "ceres")
    {
        std::cout << "Using Ceres Straight-Line Planner...\n";
        
        // Forward Kinematics to extract Cartesian workspace goals for Ceres
        double startX = cfg.link1Length * std::cos(cfg.startTheta1) + 
                        cfg.link2Length * std::cos(cfg.startTheta1 + cfg.startTheta2);
        double startY = cfg.link1Length * std::sin(cfg.startTheta1) + 
                        cfg.link2Length * std::sin(cfg.startTheta1 + cfg.startTheta2);
                        
        double goalX = cfg.link1Length * std::cos(cfg.goalTheta1) + 
                       cfg.link2Length * std::cos(cfg.goalTheta1 + cfg.goalTheta2);
        double goalY = cfg.link1Length * std::sin(cfg.goalTheta1) + 
                       cfg.link2Length * std::sin(cfg.goalTheta1 + cfg.goalTheta2);

        // Natively pass the elbow hint to Ceres based on the start configuration
        bool elbowUp = (cfg.startTheta2 >= 0.0);

        CeresPlanner planner(planar_arm, checker, cfg.link1Length, cfg.link2Length, cfg.pathInterpolationPoints);
        planner.setStartGoalWorkspace(startX, startY, goalX, goalY, elbowUp);

        if (!planner.solve())
        {
            std::cout << "Ceres failed to find a valid straight-line trajectory.\n";
            return 1;
        }

        std::cout << "Ceres solution found!\n";
        pathAngles = planner.getPathAngles();

        // Calculate path length in configuration space strictly for logging
        for (std::size_t i = 1; i < pathAngles.size(); ++i)
        {
            double d1 = pathAngles[i].first - pathAngles[i-1].first;
            double d2 = pathAngles[i].second - pathAngles[i-1].second;
            pathLength += std::sqrt(d1 * d1 + d2 * d2);
        }
    }
    else
    {
        std::cout << "Using OMPL RRT* Planner...\n";
        OMPLPlanner planner(planar_arm->getStateSpace());
        
        planner.setStateValidityChecker([&checker](const ob::State *state) {
            return checker->isStateValid(state);
        });
        
        planner.setStartGoal(cfg.startTheta1, cfg.startTheta2, cfg.goalTheta1, cfg.goalTheta2);

        RRTStarSettings settings;
        settings.range = cfg.range;
        settings.goalBias = cfg.goalBias;
        settings.rewireFactor = cfg.rewireFactor;
        settings.pathInterpolationPoints = cfg.pathInterpolationPoints;
        planner.configureRRTStar(settings);

        if (!planner.solve(cfg.solveTime))
        {
            std::cout << "No OMPL solution found.\n";
            return 1;
        }

        std::cout << "OMPL solution found! Smoothing path...\n";
        planner.simplifyPath(1.5);
        pathAngles = planner.getInterpolatedPath(cfg.pathInterpolationPoints);
        pathLength = planner.getPathLength();
    }

    const fs::path pathCsv = "path_angles.csv";
    const fs::path collisionCsv = "collision_map.csv";
    const fs::path obstaclesCsv = "obstacles.csv";

    writePathCsv(pathCsv, pathAngles);
    writeCollisionMapCsv(collisionCsv, planar_arm->getStateSpace(), *checker, cfg.collisionGridResolution);
    writeObstacleCsv(obstaclesCsv, obstacles);

    std::cout << "Found solution with " << pathAngles.size() << " states.\n";
    std::cout << "Path length in configuration space: " << pathLength << "\n";
    std::cout << "Successfully generated: " << pathCsv << ", " << collisionCsv << ", and " << obstaclesCsv << '\n';
    
    return 0;
}
