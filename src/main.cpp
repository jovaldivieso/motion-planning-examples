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

#include "manifold_types.hpp"
#include "robot_mechanism.hpp"
#include "two_dof_planar_arm.hpp"
#include "planner.hpp"
#include "ompl_planner.hpp"
#include "ceres_planner.hpp"
#include "fcl_collision_checker.hpp"

namespace fs = std::filesystem;
using motion_planning_examples::JointManifoldState;
using motion_planning_examples::RobotMechanism;
using motion_planning_examples::TwoDOFPlanarArm;
using motion_planning_examples::FCLCollisionChecker;
using motion_planning_examples::Planner;
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

// Automatically unpacks the generic manifold vectors back to visualization angles
void writePathCsv(const fs::path &path, const motion_planning_examples::ManifoldPath &manifoldStates)
{
    std::ofstream out(path);
    out << "theta1,theta2\n";
    for (const auto &state : manifoldStates)
    {
        // Knowing it's a 2DOF arm for python visualization, we extract the angles
        double t1 = std::atan2(state[1], state[0]);
        double t2 = std::atan2(state[3], state[2]);
        out << t1 << ',' << t2 << '\n';
    }
}

void writeCollisionMapCsv(const fs::path &path,
                          const FCLCollisionChecker &checker,
                          int gridResolution)
{
    std::ofstream out(path);
    out << "theta1,theta2,valid\n";
    const double pi = std::acos(-1.0);

    for (int i = 0; i < gridResolution; ++i)
    {
        const double theta1 = -pi + (2.0 * pi * i) / static_cast<double>(gridResolution - 1);
        const double c1 = std::cos(theta1);
        const double s1 = std::sin(theta1);
        for (int j = 0; j < gridResolution; ++j)
        {
            const double theta2 = -pi + (2.0 * pi * j) / static_cast<double>(gridResolution - 1);
            const double c2 = std::cos(theta2);
            const double s2 = std::sin(theta2);
            const JointManifoldState manifoldState = {c1, s1, c2, s2};

            const bool valid = checker.isManifoldStateValid(manifoldState);
            out << theta1 << ',' << theta2 << ',' << (valid ? 1 : 0) << '\n';
        }
    }
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

    std::shared_ptr<RobotMechanism> robot = std::make_shared<TwoDOFPlanarArm>(
        cfg.link1Length, cfg.link2Length, cfg.linkThickness, cfg.objectHeight);

    auto checker = std::make_shared<FCLCollisionChecker>(robot, cfg.objectHeight, obstacles);
    std::shared_ptr<Planner> planner;

    if (cfg.plannerType == "ceres")
    {
        std::cout << "Configuring Ceres Straight-Line Task Space Planner...\n";
        planner = std::make_shared<CeresPlanner>(robot, checker, cfg.pathInterpolationPoints);
    }
    else
    {
        std::cout << "Configuring OMPL RRT* C-Space Planner...\n";
        auto ompl = std::make_shared<OMPLPlanner>(robot); 
        ompl->setStateValidityChecker([&](const JointManifoldState &state) { return checker->isManifoldStateValid(state); });
        
        RRTStarSettings settings;
        settings.range = cfg.range;
        settings.goalBias = cfg.goalBias;
        settings.rewireFactor = cfg.rewireFactor;
        settings.pathInterpolationPoints = cfg.pathInterpolationPoints;
        ompl->configureRRTStar(settings);
        planner = ompl;
    }

    // Convert input config angles dynamically into the generic manifold representation
    JointManifoldState startState = {std::cos(cfg.startTheta1), std::sin(cfg.startTheta1), 
                                     std::cos(cfg.startTheta2), std::sin(cfg.startTheta2)};
    JointManifoldState goalState = {std::cos(cfg.goalTheta1), std::sin(cfg.goalTheta1), 
                                    std::cos(cfg.goalTheta2), std::sin(cfg.goalTheta2)};

    planner->setStartGoal(startState, goalState);

    if (!planner->solve(cfg.solveTime))
    {
        std::cout << "No solution found.\n";
        return 1;
    }

    std::cout << "Solution found! Extracting path...\n";
    
    auto pathManifoldStates = planner->getPathManifoldStates();
    double pathLength = planner->getPathLength();

    const fs::path pathCsv = "path_angles.csv";
    const fs::path collisionCsv = "collision_map.csv";
    const fs::path obstaclesCsv = "obstacles.csv";

    writePathCsv(pathCsv, pathManifoldStates);
    writeCollisionMapCsv(collisionCsv, *checker, cfg.collisionGridResolution);
    writeObstacleCsv(obstaclesCsv, obstacles);

    std::cout << "Found solution with " << pathManifoldStates.size() << " states.\n";
    std::cout << "Path length in configuration space: " << pathLength << "\n";
    std::cout << "Successfully generated: " << pathCsv << ", " << collisionCsv << ", and " << obstaclesCsv << '\n';
    
    return 0;
}
