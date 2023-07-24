
# Introduction

`Motion planning` plans the state sequence of the robot without conflict between the start and goal. 

`Motion planning` mainly includes `Path planning` and `Trajectory planning`.

* `Path Planning`: It's based on path constraints (such as obstacles), planning the optimal path sequence for the robot to travel without conflict between the start and goal.
* `Trajectory planning`: It plans the motion state to approach the global path based on kinematics, dynamics constraints and path sequence.

This repository provides the implement of common `Motion planning` algorithm, welcome your star & fork & PR.

This repository provides the implementation of common Motion Planning algorithms. The theory analysis can be found at [motion-planning](https://blog.csdn.net/frigidwinter/category_11410243.html). Furthermore, we provide [ROS C++](https://github.com/ai-winter/ros_motion_planning) and [Python](https://github.com/ai-winter/python_motion_planning) version.


# Quick Start

The file structure is shown below

```
├─gif
├─examples
│   ├─simulation_global.mlx
│   ├─simulation_local.mlx
│   ├─simulation_total.mlx
├─global_planner
│   ├─graph_search
│   ├─sample_search
│   └─evolutionary_search
├─local_planner
└─utils
```

The global planning algorithm implementation is in the folder `global_planner` with `graph_search`, `sample_search` and `evolutionary search`; The local planning algorithm implementation is in the folder `local_planner`.

To start simulation, open `./simulation_global.mlx` or `./simulation_local.mlx` and select the algorithm, for example

```matlab
clear all;
clc;

% load environment
load("gridmap_20x20_scene1.mat");
map_size = size(grid_map);
G = 1;

% start and goal
start = [3, 2];
goal = [18, 29];

% planner
planner_name = "rrt";

planner = str2func(planner_name);
[path, flag, cost, expand] = planner(grid_map, start, goal);

% visualization
clf;
hold on

% plot grid map
plot_grid(grid_map);
% plot expand zone
plot_expand(expand, map_size, G, planner_name);
% plot path
plot_path(path, G);
% plot start and goal
plot_square(start, map_size, G, "#f00");
plot_square(goal, map_size, G, "#15c");
% title
title([planner_name, "cost:" + num2str(cost)]);

hold off
```

# Version
## Global Planner

Planner      |    Version    | Animation   
------------ | --------- | --------- 
**GBFS**     | [![Status](https://img.shields.io/badge/done-v1.0-brightgreen)](https://github.com/ai-winter/matlab_motion_planning/blob/master/global_planner/graph_search/gbfs.m)     | ![gbfs_matlab.png](gif/gbfs_matlab.png)
**Dijkstra**     | [![Status](https://img.shields.io/badge/done-v1.0-brightgreen)](https://github.com/ai-winter/matlab_motion_planning/blob/master/global_planner/graph_search/dijkstra.m) | ![dijkstra_matlab.png](gif/dijkstra_matlab.png)
**A***     | [![Status](https://img.shields.io/badge/done-v1.0-brightgreen)](https://github.com/ai-winter/matlab_motion_planning/blob/master/global_planner/graph_search/a_star.m) | ![a_star.png](gif/a_star_matlab.png)
**JPS**                 | [![Status](https://img.shields.io/badge/done-v1.0-brightgreen)](https://github.com/ai-winter/matlab_motion_planning/blob/master/global_planner/graph_search/jps.m) |  ![jps_matlab.png](gif/jps_matlab.png)
**D***              | [![Status](https://img.shields.io/badge/done-v1.0-brightgreen)](https://github.com/ai-winter/matlab_motion_planning/blob/master/global_planner/graph_search/d_star.m) | ![d_star_matlab.png](gif/d_star_matlab.png) 
**LPA***                 | ![Status](https://img.shields.io/badge/develop-v1.0-red) | ![Status](https://img.shields.io/badge/gif-none-yellow) 
**D\* Lite**                 | ![Status](https://img.shields.io/badge/develop-v1.0-red) |![Status](https://img.shields.io/badge/gif-none-yellow)
**Voronoi**                 | [![Status](https://img.shields.io/badge/done-v1.0-brightgreen)](https://github.com/ai-winter/matlab_motion_planning/blob/master/global_planner/graph_search/voronoi_plan.m) |  ![voronoi_matlab.png](gif/voronoi_matlab.png)
**RRT**                 | [![Status](https://img.shields.io/badge/done-v1.0-brightgreen)](https://github.com/ai-winter/matlab_motion_planning/blob/master/global_planner/sample_search/rrt.m) | ![rrt_matlab.png](gif/rrt_matlab.png)
**RRT***               | [![Status](https://img.shields.io/badge/done-v1.0-brightgreen)](https://github.com/ai-winter/matlab_motion_planning/blob/master/global_planner/sample_search/rrt_star.m) |![rrt_star_matlab.png](gif/rrt_star_matlab.png)
**Informed RRT**        | [![Status](https://img.shields.io/badge/done-v1.0-brightgreen)](https://github.com/ai-winter/matlab_motion_planning/blob/master/global_planner/sample_search/informed_rrt.m) |![informed_rrt_matlab.png](gif/informed_rrt_matlab.png)
**RRT-Connect**               | [![Status](https://img.shields.io/badge/done-v1.0-brightgreen)](https://github.com/ai-winter/matlab_motion_planning/blob/master/global_planner/sample_search/rrt_connect.m) |![rrt_connect_matlab.png](gif/rrt_connect_matlab.png)

## Local Planner
| Planner |  Version    | Animation                                             |
| ------- | -------------------------------------------------------- | -------------------------------------------------------- | 
| **PID**   | [![Status](https://img.shields.io/badge/done-v1.0-brightgreen)](https://github.com/ai-winter/matlab_motion_planning/blob/master/local_planner/pid.m) | ![pid_matlab.gif](gif/pid_matlab.gif)
| **APF**   | ![Status](https://img.shields.io/badge/develop-v1.0-red) | ![Status](https://img.shields.io/badge/gif-none-yellow) 
| **DWA**  | [![Status](https://img.shields.io/badge/done-v1.0-brightgreen)](https://github.com/ai-winter/matlab_motion_planning/blob/master/local_planner/dwa.m) | ![dwa_matlab.gif](gif/dwa_matlab.gif)
| **TEB** | ![Status](https://img.shields.io/badge/develop-v1.0-red) | ![Status](https://img.shields.io/badge/gif-none-yellow) 
| **MPC** | ![Status](https://img.shields.io/badge/develop-v1.0-red) | ![Status](https://img.shields.io/badge/gif-none-yellow) 
| **Lattice** | ![Status](https://img.shields.io/badge/develop-v1.0-red) | ![Status](https://img.shields.io/badge/gif-none-yellow) 

## Intelligent Algorithm

| Planner | Version    | Animation                                                 |
| ------- | -------------------------------------------------------- | -------------------------------------------------------- 
| **ACO** | [![Status](https://img.shields.io/badge/done-v1.0-brightgreen)](https://github.com/ai-winter/matlab_motion_planning/blob/master/global_planner/evolutionary_search/aco.m) | ![aco_matlab.png](gif/aco_matlab.png)
| **GA**  | ![Status](https://img.shields.io/badge/develop-v1.0-red) | ![Status](https://img.shields.io/badge/gif-none-yellow) 
| **PSO** | ![Status](https://img.shields.io/badge/develop-v1.0-red) | ![Status](https://img.shields.io/badge/gif-none-yellow) 
| **ABC** | ![Status](https://img.shields.io/badge/develop-v1.0-red) | ![Status](https://img.shields.io/badge/gif-none-yellow) 


# Papers
## Search-based Planning
* [A*: ](https://ieeexplore.ieee.org/document/4082128) A Formal Basis for the heuristic Determination of Minimum Cost Paths
* [JPS:](https://ojs.aaai.org/index.php/AAAI/article/view/7994) Online Graph Pruning for Pathfinding On Grid Maps
* [Lifelong Planning A*: ](https://www.cs.cmu.edu/~maxim/files/aij04.pdf) Lifelong Planning A*
* [D*: ](http://web.mit.edu/16.412j/www/html/papers/original_dstar_icra94.pdf) Optimal and Efficient Path Planning for Partially-Known Environments
* [D* Lite: ](http://idm-lab.org/bib/abstracts/papers/aaai02b.pdf) D* Lite

## Sample-based Planning
* [RRT: ](http://msl.cs.uiuc.edu/~lavalle/papers/Lav98c.pdf) Rapidly-Exploring Random Trees: A New Tool for Path Planning
* [RRT-Connect: ](http://www-cgi.cs.cmu.edu/afs/cs/academic/class/15494-s12/readings/kuffner_icra2000.pdf) RRT-Connect: An Efficient Approach to Single-Query Path Planning
* [RRT*: ](https://journals.sagepub.com/doi/abs/10.1177/0278364911406761) Sampling-based algorithms for optimal motion planning
* [Informed RRT*: ](https://arxiv.org/abs/1404.2334) Optimal Sampling-based Path Planning Focused via Direct Sampling of an Admissible Ellipsoidal heuristic

## Evolutionary-based Planning
* [ACO: ](http://www.cs.yale.edu/homes/lans/readings/routing/dorigo-ants-1999.pdf) Ant Colony Optimization: A New Meta-Heuristic

## Local Planning

* [DWA: ](https://www.ri.cmu.edu/pub_files/pub1/fox_dieter_1997_1/fox_dieter_1997_1.pdf) The Dynamic Window Approach to Collision Avoidance