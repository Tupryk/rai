#include <KOMO/komo.h>
#include <DataGen/rndStableConfigs.h>
#include <Kin/F_collisions.h>
#include "Kin/F_forces.h"
#include <Kin/viewer.h>
#include <Kin/F_pose.h>
#include <Optim/NLP_Solver.h>
#include "Optim/NLP_Sampler.h"
#include <PathAlgos/RRT_PathFinder.h>

#include <thread>
#include <memory>
#include <iostream>
#include <vector>
#include <random>
#include <algorithm>
#include <string>
#include <fstream>
#include <sstream>


void TEST(Easy)
{
  rai::Configuration C(rai::raiPath("../rai-robotModels/g1/g1_free.g"));
  // rai::Configuration C("./fingersBox.g");
  cout <<"configuration space dim=" <<C.getJointStateDimension() <<endl;
  C.getFrame("pelvis")->setPosition({0., 0., 2.0});
  
  auto table = C.addFrame("table");
  table->setPosition({0., 0., 0.1});
  table->setColor(arr{0.8, 0.8, 0.8});
  table->setShape(rai::ST_ssBox, arr{10., 10., 0.05, 0.001});
  table->setContact(1);

  auto frame = C.addFrame("obj", "left_rubber_hand_0");
  // auto frame = C.addFrame("obj", "1_fing");
  frame->setColor({1, 0.5, 0});
  frame->setShape(rai::ST_ssBox, {.2, .2, .2, 0.01});
  // frame->setShape(rai::ST_sphere, {.1});
  frame->setJoint(rai::JT_free);
  frame->setMass(7.5);
  frame->setContact(1);
  frame->setPosition({0., 0., 0.});
  
  C.view(true);
  
  ofstream dataFile_js;
  
  if(!dataFile_js.is_open()) {
    dataFile_js.open(STRING("joint_states.txt"));
  }
  
  StringA supports = {
    // G1
    "logo_link_0",
    // "head_link_0",
  //   "pelvis",
  //   "pelvis_contour_link_0",
  //   "waist_yaw_joint",
  //   "waist_roll_joint",
  //   "waist_pitch_joint",
  //   "waist_support_link_0",
  //   "left_hip_pitch_joint", "right_hip_pitch_joint",
  //   "left_hip_roll_joint", "right_hip_roll_joint",
  //   "left_hip_yaw_joint", "right_hip_yaw_joint",
  //   "left_shoulder_pitch_joint", "right_shoulder_pitch_joint",
  //   "left_knee_joint", "right_knee_joint",
  //   "left_ankle_pitch_joint", "right_ankle_pitch_joint",
  
  // "left_ankle_roll_joint", "right_ankle_roll_joint",
  
  // "left_shoulder_roll_joint", "right_shoulder_roll_joint",
  // "left_shoulder_yaw_joint", "right_shoulder_yaw_joint",
  //   "left_elbow_joint", "right_elbow_joint",
  //   "left_wrist_roll_joint", "right_wrist_roll_joint",
  //   "left_wrist_pitch_joint", "right_wrist_pitch_joint",
  //   "left_wrist_yaw_joint", "right_wrist_yaw_joint",
    
    // "left_rubber_hand_0", "right_rubber_hand_0"
    
    // GO2
    // "FL_foot_0", "FR_foot_0", "RL_foot_0", "RR_foot_0"
  };

  int verbose = 1;

  uint totalEvals=0, totalSucc=0;
  arr last_sampled_joint_state;

  std::ifstream file("joint_states_fingers_box.txt");
  std::string line;
  std::vector<float> values;

  auto qHome_original = C.getJointState();

  while (std::getline(file, line))
  {
    values.clear();

    std::istringstream iss(line);
    float num;
    while (iss >> num) {
      values.push_back(num);
    }

    arr l_fing = {values[0], values[1], values[2]};
    arr r_fing = {values[3], values[4], values[5]};
    arr box_pos = {values[6], values[7], values[8]};
    arr box_rot = {values[9], values[10], values[11], values[12]};
    // C.setJointState({
    //   values[0], values[1], values[2],
    //   values[3], values[4], values[5],
    //   values[6] - values[0], values[7] - values[1], values[8] - values[2],
    //   values[9], values[10], values[11], values[12]
    // });
    // C.view(true, "Jointed");
    // C.setJointState(qHome_original);

    auto qHome = randn(C.getJointStateDimension()) * 0.4;
    // auto qHome = randn(C.getJointStateDimension()) * 0.1;
    C.setJointState(qHome_original + qHome);

    KOMO komo;
    komo.setConfig(C, true);
    komo.setTiming(1,1,1,0);
    komo.addControlObjective({}, 0, 1e-2);

    komo.addObjective({}, FS_jointLimits, {}, OT_ineq);
    komo.addObjective({}, FS_accumulatedCollisions, {}, OT_eq, {1e1});

    komo.addObjective({}, FS_positionDiff, {"left_rubber_hand_0", "obj"}, OT_eq, {1e1}, l_fing - box_pos);
    komo.addObjective({}, FS_positionDiff, {"right_rubber_hand_0", "obj"}, OT_eq, {1e1}, r_fing - box_pos);
    // komo.addObjective({}, FS_positionDiff, {"1_fing", "obj"}, OT_eq, {1e1}, l_fing - box_pos);
    // komo.addObjective({}, FS_positionDiff, {"2_fing", "obj"}, OT_eq, {1e1}, r_fing - box_pos);
    komo.addObjective({}, FS_quaternion, {"obj"}, OT_eq, {1e1}, box_rot);
    komo.addObjective({}, make_shared<F_BodyTotalForce>(), {"pelvis"}, OT_eq, {1e1});

    std::random_device rd;
    std::mt19937 gen(rd());

    std::uniform_int_distribution<> dist(1, 2);
    int feet_idx = dist(gen); // One foot, Both feet
    
    if (feet_idx == 2)  // Both feet
    {
      int feet_sides = dist(gen); // Both feet full, One foot full
      if (feet_sides == 1)
      {
        komo.addContact_WithPoaFrame(1., "l_foot_marker_1", "table", .8, .05);
        komo.addContact_WithPoaFrame(1., "l_foot_marker_2", "table", .8, .05);
        komo.addContact_WithPoaFrame(1., "l_foot_marker_3", "table", .8, .05);
        komo.addContact_WithPoaFrame(1., "l_foot_marker_4", "table", .8, .05);
        
        komo.addContact_WithPoaFrame(1., "r_foot_marker_1", "table", .8, .05);
        komo.addContact_WithPoaFrame(1., "r_foot_marker_2", "table", .8, .05);
        komo.addContact_WithPoaFrame(1., "r_foot_marker_3", "table", .8, .05);
        komo.addContact_WithPoaFrame(1., "r_foot_marker_4", "table", .8, .05);
      }
      else if (feet_sides == 2)
      {
        int foot_half = dist(gen); // Left, Right
        int foot_side = dist(gen); // Front, Back
        if (foot_half == 1)
        {
          komo.addContact_WithPoaFrame(1., "r_foot_marker_1", "table", .8, .05);
          komo.addContact_WithPoaFrame(1., "r_foot_marker_2", "table", .8, .05);
          komo.addContact_WithPoaFrame(1., "r_foot_marker_3", "table", .8, .05);
          komo.addContact_WithPoaFrame(1., "r_foot_marker_4", "table", .8, .05);

          if (foot_side == 1)
          {
            komo.addContact_WithPoaFrame(1., "l_foot_marker_1", "table", .8, .05);
            komo.addContact_WithPoaFrame(1., "l_foot_marker_3", "table", .8, .05);
          }
          else if (foot_side == 2)
          {
            komo.addContact_WithPoaFrame(1., "l_foot_marker_2", "table", .8, .05);
            komo.addContact_WithPoaFrame(1., "l_foot_marker_4", "table", .8, .05);
          }
        }
        else if (foot_half == 2)
        {
          komo.addContact_WithPoaFrame(1., "l_foot_marker_1", "table", .8, .05);
          komo.addContact_WithPoaFrame(1., "l_foot_marker_2", "table", .8, .05);
          komo.addContact_WithPoaFrame(1., "l_foot_marker_3", "table", .8, .05);
          komo.addContact_WithPoaFrame(1., "l_foot_marker_4", "table", .8, .05);

          if (foot_side == 1)
          {
            komo.addContact_WithPoaFrame(1., "r_foot_marker_1", "table", .8, .05);
            komo.addContact_WithPoaFrame(1., "r_foot_marker_3", "table", .8, .05);
          }
          else if (foot_side == 2)
          {
            komo.addContact_WithPoaFrame(1., "r_foot_marker_2", "table", .8, .05);
            komo.addContact_WithPoaFrame(1., "r_foot_marker_4", "table", .8, .05);
          }
        }
      }
    }
    else
    {
      int foot_idx = dist(gen); // Left, Right
      if (foot_idx == 1)
      {
        komo.addContact_WithPoaFrame(1., "l_foot_marker_1", "table", .8, .05);
        komo.addContact_WithPoaFrame(1., "l_foot_marker_2", "table", .8, .05);
        komo.addContact_WithPoaFrame(1., "l_foot_marker_3", "table", .8, .05);
        komo.addContact_WithPoaFrame(1., "l_foot_marker_4", "table", .8, .05);
      }
      else if (foot_idx == 2)
      {
        komo.addContact_WithPoaFrame(1., "r_foot_marker_1", "table", .8, .05);
        komo.addContact_WithPoaFrame(1., "r_foot_marker_2", "table", .8, .05);
        komo.addContact_WithPoaFrame(1., "r_foot_marker_3", "table", .8, .05);
        komo.addContact_WithPoaFrame(1., "r_foot_marker_4", "table", .8, .05);
      }
    }

    if(verbose>0) komo.set_viewer(C.get_viewer());

    for(uint k=0;k<1;k++){
      // komo.initRandom();

      auto ret = rai::NLP_Solver(komo.nlp())
                .setOptions(rai::OptOptions().set_stopEvals(1000000).set_stopTolerance(.001))
                .solve();

      cout << "Forces Report: " <<komo.pathConfig.reportForces() <<endl;
      if(verbose>0) LOG(0) <<*ret;
      // cout <<komo.report(true) <<endl;
      if(verbose>2){
        cout <<komo.pathConfig.reportForces() <<endl;
      }
      if(!ret->feasible){
        if(verbose>2){
          komo.view(verbose>3, STRING("failed" <<*ret));
          // std::cout << "Collisions: " << komo.pathConfig.coll_getProxyPairs(0.1) << std::endl;
        }
      }else{
        totalSucc ++;
        totalEvals += ret->evals;
        last_sampled_joint_state = komo.getPath();
        last_sampled_joint_state.modRaw().write(dataFile_js);
        dataFile_js << endl;
        if(verbose>0) komo.view(verbose>1, STRING("\n" <<*ret));
      }
    }
  }
}

int MAIN(int argc,char** argv)
{
  rai::initCmdLine(argc,argv);

  rnd.clockSeed();

  testEasy();

  return 0;
}
