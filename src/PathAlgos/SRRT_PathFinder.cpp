/*  ------------------------------------------------------------------
    Copyright (c) 2011-2024 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#include "SRRT_PathFinder.h"
#include "RRT_utils.h"

namespace rai {
  
  arr SRRT_SingleTree::preprocessStateNode(const arr& state_node)
  {
    // Sorry about this...
    arr new_state_node;
    if (relevant_frames.N)
    {
      int vector_dim = use_rotations ? 7 : 3;
      new_state_node.resize(relevant_frames.N * vector_dim);
      for (int i = 0; i < relevant_frames.N; i++)
      {
        auto idx = relevant_frames(i);
        new_state_node.elem(i*vector_dim    ) = state_node(idx, 0);
        new_state_node.elem(i*vector_dim + 1) = state_node(idx, 1);
        new_state_node.elem(i*vector_dim + 2) = state_node(idx, 2);
        if (use_rotations) {
          new_state_node.elem(i*vector_dim + 3) = state_node(idx, 3);
          new_state_node.elem(i*vector_dim + 4) = state_node(idx, 4);
          new_state_node.elem(i*vector_dim + 5) = state_node(idx, 5);
          new_state_node.elem(i*vector_dim + 6) = state_node(idx, 6);
        }
      }
    }
    else
    {
      int new_size = use_rotations ? state_node.N*7 : state_node.N*3;
      new_state_node.resize(new_size);
      if (use_rotations) {
        new_state_node = state_node;
      } else {
        for (int i = 0; i < state_node.N; i++) {
          int idx = i*3;
          new_state_node.elem(idx    ) = state_node(i, 0);
          new_state_node.elem(idx + 1) = state_node(i, 1);
          new_state_node.elem(idx + 2) = state_node(i, 2);
        }
      }
    }
    return new_state_node;
  }

  double SRRT_SingleTree::measureError(const arr& sim_state_a, const arr& sim_state_b)
  {
    // TODO: rotations
    arr relevant_state_a = preprocessStateNode(sim_state_a);
    arr relevant_state_b = preprocessStateNode(sim_state_b);
    
    double sum_ = 0.0;
    
    // MSE of relevant frames
    for (int i = 0; i < relevant_state_a.N; i+=3)
    {
      arr err = arr{
        relevant_state_a(i    ) - relevant_state_b(i    ),
        relevant_state_a(i + 1) - relevant_state_b(i + 1),
        relevant_state_a(i + 2) - relevant_state_b(i + 2)
      };
      double e = length(err);
      sum_ += e*e;
    }

    return sum_;
  }
  
  void SRRT_SingleTree::addANN(const arr& new_state)
  {
    arr ann_node = preprocessStateNode(new_state);
    ann.append(ann_node);
  }
    
  uint SRRT_SingleTree::queryANN(const arr& state)
  {
    arr query = preprocessStateNode(state);
    uint result = ann.getNN(query.reshape(-1));
    return result;
  }

  uintA SRRT_SingleTree::queryKANN(const arr& state, int k)
  {
    arr query = preprocessStateNode(state);
    uintA result;
    ann.getkNN(result, query.reshape(1, -1), k);
    return result;
  }

  SRRT_SingleTree::SRRT_SingleTree(SimState& start_state,
                                   const SimState& goal_state_,
                                   const shared_ptr<QueryResult>& c_info,
                                   const uintA& relevant_frames_=uintA{})
  {
    goal_state = goal_state_;
    relevant_frames = relevant_frames_;
    add(start_state.joint_poses, 0, c_info, start_state);
    frame_state_nearest_id = 0;
  }

  double SRRT_SingleTree::add(const arr& joint_state,
                            uint parentID,
                            const shared_ptr<QueryResult>& c_info,
                            SimState sim_state)
  {
    drawMutex.lock(RAI_HERE);

    addANN(sim_state.frame_poses);
    collision_info.append(c_info);
    parents.append(parentID);
    nodes.append(sim_state);
  
    CHECK_EQ(parents.N, ann.X.d0, "");
    CHECK_EQ(collision_info.N, ann.X.d0, "");

    double err = measureError(goal_state.frame_poses, sim_state.frame_poses);
    if (err < frame_state_nearest_score) {
      frame_state_nearest_id = nodes.N-1;
      frame_state_nearest_score = err;
    }

    drawMutex.unlock();
    return err;
  }

  arr SRRT_SingleTree::getPathFromNode(uint fromID)
  {
    arr path;
    uint node = fromID;
    while (true) {
      path.append(nodes(node).frame_poses);
      if(!node) break;
      node = getParent(node);
    }
    path.reshape(-1, nodes(0).frame_poses.N);
    return path;
  }

  std::tuple<arr, uint> SRRT_SingleTree::getNewSample(
      const arr& target_joint_state,
      const SimState& goal_state,
      double stepsize,
      uintA spherical_coors,
      double p_connect_best_state,
      int k_best_states)
  {
    // Nearest neighbour to target_joint_state in tree
    uint parent_node_id;
    if (rnd.uni() < p_connect_best_state && nodes.N > k_best_states) {
      uintA best_parents_ids = queryKANN(goal_state.frame_poses, k_best_states); // This should store the configs from the other dist metric
      parent_node_id = best_parents_ids.rndElem();
    } else {
      parent_node_id = rndInt(nodes.N);
    }
    
    SimState parent_node = getNodeSim(parent_node_id);

    // Direction and distance
    arr delta = target_joint_state - parent_node.joint_poses;
    double dist = length(delta);
    if(dist > stepsize)
    {
      // Scale if the distance is greater than the stepsize
      delta *= stepsize / dist;
    }
    
    arr new_sample = parent_node.joint_poses + delta;

    // Fix new joint state if there are spherical coordinates
    if (spherical_coors.N)
    {
      for (int i = 0; i < spherical_coors.d0; i++)
      {
        normalizeSphericalCoordinates(new_sample, spherical_coors[i]);
      }
    }

    return std::make_tuple(new_sample, parent_node_id);
  }

  int SRRT_PathFinder::stepConnect()
  {
    // Count iterations
    iters++;
    
    if(iters > (uint) options.maxIters) {
      return -1;
    }
    if (iters % options.show_every == 0) {
      std::cout << "SRRT iter: " << iters << " Tree size: " << srrt0->nodes.N << std::endl;
      std::cout << "Best score so far: " << srrt0->frame_state_nearest_score << std::endl;
      problemConfig->C.view();
    }

    // Grow trees (for now just one)
    bool success = growTreeToGoal();

    // If we reach the goal, store the path
    if(success)
    {
      // The variable nearestID stores the ID
      // of the node nearest to the goal
      std::cout << "Found a solution, yayy :)" << std::endl;
      path = srrt0->getPathFromNode(srrt0->frame_state_nearest_id);
      revertPath(path);
      path.append(goal_state.frame_poses);
      return 1;
    }

    // Goal not reached yet
    return 0;
  }

  bool SRRT_PathFinder::growTreeToGoal()
  {
    // Choose a target joint state: random or goal state
    arr joint_target = sample_target_joint_state();

    // Extend tree towards target
    auto sample = srrt0->getNewSample(joint_target, goal_state, options.stepsize, problemConfig->sphericalCoordinates, options.p_connect_best_state, options.k_best_states);
    arr new_joint_state = std::get<0>(sample);
    uint parent_id = std::get<1>(sample);

    // Special rule: if parent is already in collision, isFeasible = smaller collisions
    shared_ptr<QueryResult>& c_info_parent = srrt0->collision_info(parent_id);
    double org_collisionTolerance = problemConfig->collisionTolerance;
    if (c_info_parent->totalCollision > problemConfig->collisionTolerance)
    {
      problemConfig->collisionTolerance = c_info_parent->totalCollision + 1e-6;
    }
    // Evaluate new joint sample collisions
    problemConfig->C.setFrameState(srrt0->getNodeSim(parent_id).frame_poses);
    auto c_info_new = problemConfig->query(new_joint_state);
    
    // Check subsamples to update feasibility
    if (c_info_new->isFeasible && options.subsamples > 0)
    {
      arr parent = srrt0->getNodeSim(parent_id).frame_poses;
      const arr start = parent.reshape(-1, 7);
      c_info_new->isFeasible = checkConnection(*problemConfig, start, new_joint_state, options.subsamples, true);
    }
    problemConfig->collisionTolerance = org_collisionTolerance;
    
    // If the new node is feasible add it to the tree and perform a simulation step
    // Collision pairs are very important for SRRT
    if (c_info_new->isFeasible)
    {
      // Calculate simulation state
      SimState parent_sim_state = srrt0->getNodeSim(parent_id);
      sim->setState(
        parent_sim_state.frame_poses,
        parent_sim_state.joint_poses,
        parent_sim_state.frame_vels,
        parent_sim_state.joint_vels);
  
      double time_to_rollout = .01;
      double tau = .01;
      sim->setSplineRef(new_joint_state.reshape({1, new_joint_state.N}), {time_to_rollout});
      int timesteps = ceil(time_to_rollout/tau);
      for (int i = 0; i < timesteps; i++) {
        sim->step({}, tau, Simulation::ControlMode::_spline);
      }
  
      SimState new_sim_state;
      sim->getState(
        new_sim_state.frame_poses,
        new_sim_state.joint_poses,
        new_sim_state.frame_vels,
        new_sim_state.joint_vels);
  
      new_joint_state = new_sim_state.joint_poses;

      // Store new state in tree
      double newest_error = srrt0->add(new_joint_state, parent_id, c_info_new, new_sim_state);
      if (problemConfig->sphericalCoordinates.N)
      {
        CHECK_LE(problemConfig->sphericalCoordinates.d0, 1, "");
        arr q_org = new_joint_state;
        for (int i = 0; i < problemConfig->sphericalCoordinates.d0; i++)
        {
          flipSphericalCoordinates(new_joint_state, problemConfig->sphericalCoordinates[i]);
          srrt0->add(new_joint_state, parent_id, c_info_new, new_sim_state);
        }
        new_joint_state = q_org;
      }

      // Check if goal is reached
      if (options.subsamples>0)
      {
        // TODO: Figure out what subsamples are
        if (newest_error < options.error_thresh / options.subsamples)
        {
          return true;
        }
      }
      else
      {
        if (newest_error < options.error_thresh)
        {
          return true;
        }
      }
    }

    return false;
  }

  arr SRRT_PathFinder::sample_target_joint_state()
  {
    // Choose a target joint state: random or goal state
    arr joint_target;

    if(rnd.uni() < options.p_connect_joint_goal)
    {
      // Goal as target
      joint_target = goal_state.joint_poses;
    }
    else
    {
      // Random Target
      joint_target.resize(srrt0->getNodeSim(0).joint_poses.N);
      
      // Loop through each dimension and sample a random joint state
      for(int i = 0; i < joint_target.N; i++)
      {
        double min_ = problemConfig->limits(0, i);
        double max_ = problemConfig->limits(1, i);
        double range = max_ - min_;
        
        // Check if the joint can move
        CHECK_GE(range, 1e-3, "Limits are null interval: " << i << ' ' << problemConfig->C.getJointNames());
        
        // Linear interpolation
        joint_target.elem(i) = min_ + rnd.uni() * range;
      }

      // Overwrite the spherical joints with proper samples
      for(int i = 0; i < problemConfig->sphericalCoordinates.d0; i++)
      {
        randomSphericalCoordinates(joint_target, problemConfig->sphericalCoordinates[i]);
      }
    }
    return joint_target;
  }

  void SRRT_PathFinder::setStartGoal(Configuration& config_0, const Configuration& config_T)
  {
    // Setup config and sim
    problemConfig = make_shared<ConfigurationProblem>(config_0, options.useBroadCollisions, options.collisionTolerance, 1);
    problemConfig->verbose = 0;

    if (collision_pairs.N) {
      problemConfig->setExplicitCollisionPairs(collision_pairs);
    }

    sim = make_shared<Simulation>(config_0, Simulation::Engine::_physx, 0);

    arr start_joint_state = config_0.getJointState();
    arr end_joint_state = config_T.getJointState();

    // Check if the start and end states are feasible (not really correct, need to feed in the frame state)
    auto q0ret = problemConfig->query(start_joint_state);
    auto qTret = problemConfig->query(end_joint_state);
    
    if(!q0ret->isFeasible)
    {
      LOG(0) <<"initializing with infeasible q0:";
      q0ret->writeDetails(std::cout, *problemConfig);
    }
    
    if(!qTret->isFeasible)
    {
      LOG(0) <<"initializing with infeasible qT:";
      qTret->writeDetails(std::cout, *problemConfig);
    }

    // Store the start and end state and create a tree
    SimState sim_state0;

    sim->getState(
      sim_state0.frame_poses,
      sim_state0.joint_poses,
      sim_state0.frame_vels,
      sim_state0.joint_vels
    );

    // Just to fill the goal state with something before overwriting it
    sim->getState(
      goal_state.frame_poses,
      goal_state.joint_poses,
      goal_state.frame_vels,
      goal_state.joint_vels
    );
    
    goal_state.frame_poses = config_T.getFrameState();
    goal_state.joint_poses = end_joint_state;

    if (relevant_frame_names.N)
    {
      uintA relevant_frames = problemConfig->C.getFrameIDs(relevant_frame_names);
      srrt0 = make_shared<SRRT_SingleTree>(sim_state0, goal_state, q0ret, relevant_frames);
    }
    else
    {
      srrt0 = make_shared<SRRT_SingleTree>(sim_state0, goal_state, q0ret);
    }
    srrt0->use_rotations = options.use_rotations;
  }

  shared_ptr<SolverReturn> SRRT_PathFinder::solve()
  {
    // Allocate memory for the resulst of the solver
    if(!results) {
      results = make_shared<SolverReturn>();
    }
    
    // Mesure the time it takes the solver to find a solution
    auto start_time = rai::cpuTime();

    // Create new nodes in the tree until the goal is found or
    // the maximum amount of iterations is reached
    int solved = 0;
    while(!solved)
    {
      solved = stepConnect();
    }
    auto end_time = rai::cpuTime();

    // No solution was found, return the best one found
    if(solved == -1)
    {
      LOG(0) << "Solution not found! Best distance: " << srrt0->frame_state_nearest_score;
      path = srrt0->getPathFromNode(srrt0->frame_state_nearest_id);
      revertPath(path);
    }

    // Record results
    results->time = end_time - start_time;
    results->done = solved == 1;
    results->feasible = path.N;
    results->x = path;
    results->evals = iters;
    
    return results;
  }

  SRRT_PathFinder::SRRT_PathFinder()
  {
    collision_pairs = {};
    relevant_frame_names = {};
  }
  
  // SRRT_PathFinder::SRRT_PathFinder(const StringA& collision_pairs_, const StringA& relevant_frame_names_) :
  //   collision_pairs(collision_pairs_), relevant_frame_names(relevant_frame_names_) {}

  void SRRT_PathFinder::setInfo(const StringA& collision_pairs_, const StringA& relevant_frame_names_)
  {
    collision_pairs = collision_pairs_;
    relevant_frame_names = relevant_frame_names_;
  }
}
