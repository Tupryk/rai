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


void setBoxContacts(KOMO* komo, double phase)
{
  std::random_device rd;
  std::mt19937 gen(rd());
  std::uniform_int_distribution<> dist(1, 2);

  komo->addObjective({}, make_shared<F_TotalForce>(), {"obj"}, OT_eq, {1e1});

  int one_two_hands = dist(gen); // One had, Both hands
  int hand_idx = dist(gen); // Left hand, Right hand

  std::vector<std::string> supports = {
    "logo_link_0",
    "head_link_0",
    "pelvis",
    "pelvis_contour_link_0",
    "waist_yaw_joint",
    "waist_roll_joint",
    "waist_pitch_joint",
    "waist_support_link_0",
    "left_hip_pitch_joint", "right_hip_pitch_joint",
    "left_hip_roll_joint", "right_hip_roll_joint",
    "left_hip_yaw_joint", "right_hip_yaw_joint",
    "left_shoulder_pitch_joint", "right_shoulder_pitch_joint",
    "left_knee_joint", "right_knee_joint",
    "left_ankle_pitch_joint", "right_ankle_pitch_joint",
  
    "left_ankle_roll_joint", "right_ankle_roll_joint",
  
    "left_shoulder_roll_joint", "right_shoulder_roll_joint",
    "left_shoulder_yaw_joint", "right_shoulder_yaw_joint",
    "left_elbow_joint", "right_elbow_joint",
    "left_wrist_roll_joint", "right_wrist_roll_joint",
    "left_wrist_pitch_joint", "right_wrist_pitch_joint",
    "left_wrist_yaw_joint", "right_wrist_yaw_joint",
    
    // "left_rubber_hand_0", "right_rubber_hand_0"
  };
  
  one_two_hands = 1;
  if (one_two_hands == 2 || hand_idx == 1)
  {
    komo->addContact_WithPoaFrame(phase, "obj", "left_rubber_hand_0", .8, .05);
    // komo->addContact_WithPoaFrame(phase, "obj", "left_knee_joint", .8, .05);
  }
  if (one_two_hands == 2 || hand_idx == 2)
  {
    komo->addContact_WithPoaFrame(phase, "obj", "right_rubber_hand_0", .8, .05);
    // komo->addContact_WithPoaFrame(phase, "obj", "logo_link_0", .8, .05);
  }
  if (one_two_hands != 2)
  {
    std::mt19937 rng(std::random_device{}());
    std::uniform_int_distribution<size_t> dist(0, supports.size() - 1);
    auto random_frame = supports[dist(rng)];
    komo->addContact_WithPoaFrame(phase, "obj", random_frame.c_str(), .8, .05);
  }
}

void setFeetContacts(KOMO* komo, double phase)
{
  std::random_device rd;
  std::mt19937 gen(rd());
  std::uniform_int_distribution<> dist(1, 2);

  int feet_idx = dist(gen); // One foot, Both feet    
  int feet_sides = dist(gen); // Both feet full, One foot full
  int foot_half = dist(gen); // Left, Right
  int foot_side = dist(gen); // Front, Back
  int foot_idx = dist(gen); // Left, Right

  komo->addObjective({}, make_shared<F_BodyTotalForce>(), {"pelvis"}, OT_eq, {1e1});

  if (
    (feet_idx == 2 && feet_sides == 1) ||
    (feet_idx == 2 && feet_sides == 2 && foot_half == 2) ||
    (feet_idx == 2 && feet_sides == 2 && foot_half == 1 && foot_side == 1) ||
    (feet_idx == 1 && foot_idx == 1)
  ) {
    komo->addContact_WithPoaFrame(phase, "l_foot_marker_1", "table", .8, .05);
    komo->addContact_WithPoaFrame(phase, "l_foot_marker_3", "table", .8, .05);
  }

  if (
    (feet_idx == 2 && feet_sides == 1) ||
    (feet_idx == 2 && feet_sides == 2 && foot_half == 2) ||
    (feet_idx == 2 && feet_sides == 2 && foot_half == 1 && foot_side == 2) ||
    (feet_idx == 1 && foot_idx == 1)
  ) {
    komo->addContact_WithPoaFrame(phase, "l_foot_marker_2", "table", .8, .05);
    komo->addContact_WithPoaFrame(phase, "l_foot_marker_4", "table", .8, .05);
  }

  if (
    (feet_idx == 2 && feet_sides == 2) ||
    (feet_idx == 2 && feet_sides == 1 && foot_half == 2) ||
    (feet_idx == 2 && feet_sides == 1 && foot_half == 1 && foot_side == 1) ||
    (feet_idx == 1 && foot_idx == 2)
  ) {
    komo->addContact_WithPoaFrame(phase, "r_foot_marker_1", "table", .8, .05);
    komo->addContact_WithPoaFrame(phase, "r_foot_marker_3", "table", .8, .05);
  }

  if (
    (feet_idx == 2 && feet_sides == 2) ||
    (feet_idx == 2 && feet_sides == 1 && foot_half == 2) ||
    (feet_idx == 2 && feet_sides == 1 && foot_half == 1 && foot_side == 2) ||
    (feet_idx == 1 && foot_idx == 2)
  ) {
    komo->addContact_WithPoaFrame(phase, "r_foot_marker_2", "table", .8, .05);
    komo->addContact_WithPoaFrame(phase, "r_foot_marker_4", "table", .8, .05);
  }
}

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
  // frame->setShape(rai::ST_ssBox, {.2, .2, .2, 0.01});
  frame->setShape(rai::ST_sphere, {.1});
  frame->setJoint(rai::JT_free);
  frame->setMass(7.5);
  frame->setContact(1);
  frame->setPosition({0., 0., 0.});
  
  C.view(true);
  
  ofstream dataFile_js;
  
  if(!dataFile_js.is_open()) {
    dataFile_js.open(STRING("joint_states.txt"));
  }

  int verbose = 1;

  uint totalEvals=0, totalSucc=0;
  arr last_sampled_joint_state;

  auto qHome_original = C.getJointState();
  
  while (1)
  {
    auto q1_offset = randn(C.getJointStateDimension()) * 1.;
    q1_offset.elem(q1_offset.N-4) += 10.;
    // auto q2_offset = randn(C.getJointStateDimension()) * 0.1;
    auto q1 = qHome_original + q1_offset;
    // auto q2 = qHome_original + q2_offset;

    KOMO komo;
    komo.setConfig(C, true);
    komo.setTiming(1,1,1,0);

    // komo.addControlObjective({}, 1, 1e-2);
    // komo.addControlObjective({}, 2, 1e-2);
    
    komo.addObjective({}, FS_jointLimits, {}, OT_ineq);
    komo.addObjective({}, FS_accumulatedCollisions, {}, OT_eq, {1e1});
    
    komo.addObjective({1.}, FS_qItself, {}, OT_sos, {1e-2}, q1);
    setBoxContacts(&komo, 1.);
    setFeetContacts(&komo, 1.);

    // komo.addObjective({2.}, FS_qItself, {}, OT_sos, {1e1}, q2);
    // // setBoxContacts(&komo, 2.);
    // // setFeetContacts(&komo, 2.);

    if(verbose>0) komo.set_viewer(C.get_viewer());

    auto ret = rai::NLP_Solver(komo.nlp())
              .setOptions(rai::OptOptions().set_stopEvals(1000).set_stopTolerance(.001))
              .solve();

    cout << "Forces Report: " <<komo.pathConfig.reportForces() <<endl;
    if(verbose>0) LOG(0) <<*ret;
    if(verbose>2){
      cout <<komo.pathConfig.reportForces() <<endl;
    }
    if(!ret->feasible){
      if(verbose>2){
        komo.view(verbose>3, STRING("failed" <<*ret));
      }
    }else{
      totalSucc ++;
      totalEvals += ret->evals;
      last_sampled_joint_state = komo.getPath();
      last_sampled_joint_state.modRaw().write(dataFile_js);
      dataFile_js << endl;
      if(verbose>0) komo.view(verbose>1, STRING("\n" <<*ret));
      // if(verbose>0) komo.view_play(verbose>1, STRING("\n" <<*ret));
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
