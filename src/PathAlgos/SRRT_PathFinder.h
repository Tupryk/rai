/*  ------------------------------------------------------------------
    Copyright (c) 2011-2024 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#pragma once

#include "PathResult.h"
#include "ConfigurationProblem.h"
#include "../Kin/simulation.h"
#include "../Optim/NLP.h"
#include "../Algo/ann.h"

namespace rai {

  struct SimState
  {
    arr joint_poses;
    arr frame_poses;
    arr joint_vels;
    arr frame_vels;
  };

  struct SRRT_SingleTree
  {
    rai::Array<SimState> nodes;                          // Each node stores a simulation state
    ANN ann;                                             // Stores all joint states in a spatial datastructure
    uint frame_state_nearest_id = UINT_MAX;
    double frame_state_nearest_score = 1e6;
    uintA relevant_frames;
    bool use_rotations = false;
    rai::Array<shared_ptr<QueryResult>> collision_info;  // For each point we store the query result
    Mutex drawMutex;
    uintA parents;                                       // For each point we store the index of the parent node
    SimState goal_state;

    SRRT_SingleTree(SimState& start_state, const SimState& goal_state_, const shared_ptr<QueryResult>& c_info, const uintA& relevant_frames_);

    arr getPathFromNode(uint fromID);
    std::tuple<arr, uint> getNewSample(const arr& target_joint_state, const SimState& goal_state, double stepsize, uintA spherical_coors, double p_connect_best_state, int k_best_states);
    
    double add(const arr& joint_state, uint parentID, const shared_ptr<QueryResult>& c_info, SimState sim_state);
    uint getParent(uint i) { return parents(i); }
    
    SimState getNodeSim(uint i) { return nodes(i); }

    void addANN(const arr& new_state);
    uint queryANN(const arr& state);
    uintA queryKANN(const arr& state, int k);
    arr preprocessStateNode(const arr& state_node);
    double measureError(const arr& sim_state_a, const arr& sim_state_b);
  };

  struct SRRT_PathFinder_Options
  {
    RAI_PARAM("srrt/", int, verbose, 0)
    RAI_PARAM("srrt/", double, stepsize, .1)
    RAI_PARAM("srrt/", int, subsamples, 4)
    RAI_PARAM("srrt/", int, maxIters, 5000)
    RAI_PARAM("srrt/", double, p_connect_joint_goal, .5)
    RAI_PARAM("srrt/", double, p_connect_best_state, .5)
    RAI_PARAM("srrt/", int, k_best_states, 10) // This should addapt to the size of the tree. All of the variables should adapth actually. Also delete branches (nodes) that lead to nowhere (don't forget to delete them in ANN).
    RAI_PARAM("srrt/", double, collisionTolerance, 1e-4)
    RAI_PARAM("srrt/", bool, useBroadCollisions, false)
    RAI_PARAM("srrt/", bool, use_rotations, true)
    RAI_PARAM("srrt/", int, show_every, 100)
    RAI_PARAM("srrt/", double, error_thresh, 1e-3)
  };

  struct SRRT_PathFinder : NonCopyable
  {
    SRRT_PathFinder_Options options;

    shared_ptr<ConfigurationProblem> problemConfig;
    shared_ptr<Simulation> sim;

    // SRRT from the start simulation state
    shared_ptr<SRRT_SingleTree> srrt0;
    
    // Results from the SRRT run
    shared_ptr<SolverReturn> results;

    SimState goal_state;
    
    uint iters=0;

    // output
    arr path;

    StringA relevant_frame_names;
    StringA collision_pairs;

    SRRT_PathFinder();
    void setInfo(const StringA& collision_pairs_, const StringA& relevant_frame_names_);
    void setStartGoal(Configuration& config_0, const Configuration& config_T);
    shared_ptr<SolverReturn> solve();

  private:
    int stepConnect();
    bool growTreeToGoal();
    arr sample_target_joint_state();
  };
} //namespace
