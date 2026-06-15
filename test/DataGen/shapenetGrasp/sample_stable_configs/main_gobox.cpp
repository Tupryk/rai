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
#include <iostream>
#include <format>


void TEST(Easy)
{
  rai::Configuration C(rai::raiPath("../rai-robotModels/go2/go2.yml"));
  // rai::Configuration C("./fingersBox.g");
  cout <<"configuration space dim=" <<C.getJointStateDimension() <<endl;
  C.getFrame("base_link")->setPosition({0., 0., 2.0});

  auto fl = C.addFrame("FL_marker", "FL_foot_0");
  fl->setColor({1, 0, 0});
  fl->setShape(rai::ST_sphere, {.03});
  fl->setMass(.001);
  fl->setContact(1);

  auto fr = C.addFrame("FR_marker", "FR_foot_0");
  fr->setColor({1, 0, 0});
  fr->setShape(rai::ST_sphere, {.03});
  fr->setMass(.001);
  fr->setContact(1);

  auto rl = C.addFrame("RL_marker", "RL_foot_0");
  rl->setColor({1, 0, 0});
  rl->setShape(rai::ST_sphere, {.03});
  rl->setMass(.001);
  rl->setContact(1);

  auto rr = C.addFrame("RR_marker", "RR_foot_0");
  rr->setColor({1, 0, 0});
  rr->setShape(rai::ST_sphere, {.03});
  rr->setMass(.001);
  rr->setContact(1);
  
  auto table = C.addFrame("table");
  table->setPosition({0., 0., 0.1});
  table->setColor(arr{0.8, 0.8, 0.8});
  table->setShape(rai::ST_ssBox, arr{10., 10., 0.05, 0.001});
  table->setContact(-2);

  // for (int i = 0; i < 4; i++)
  // {
  //   for (int j = 0; j < 4; j++)
  //   {
  //     for (int k = 0; k < 1; k++)
  //     {
  //       float x = ((float)i) / 4.f * 10.f - 3.75;
  //       float y = ((float)j) / 4.f * 10.f - 3.75f;
  //       float z = ((float)k) * 0.15f + 0.2f;

  //       float w = (1.f - ((float)k) / 3.f) * 0.5f + 0.5f;

  //       std::string s = std::format("table_step_{}_{}_{}", i, j, k);
  //       auto table_step = C.addFrame(s.c_str(), "table");
  //       table_step->setPosition({x, y, z});
  //       table_step->setColor(arr{0.8, 0.8, 0.8});
  //       table_step->setShape(rai::ST_ssBox, arr{w, w, 0.15, 0.001});
  //       table_step->setContact(-2);
  //     }
  //   }
  // }

  auto table_step1 = C.addFrame("table_step1", "table");
  table_step1->setPosition({1.5, 0.0, 0.2});
  table_step1->setColor(arr{0.8, 0.8, 0.8});
  table_step1->setShape(rai::ST_ssBox, arr{2.5, 2.5, 0.15, 0.001});
  table_step1->setContact(-2);

  auto table_step2 = C.addFrame("table_step2", "table");
  table_step2->setPosition({1.5, 0.0, 0.35});
  table_step2->setColor(arr{0.8, 0.8, 0.8});
  table_step2->setShape(rai::ST_ssBox, arr{1.5, 1.5, 0.15, 0.001});
  table_step2->setContact(-2);

  auto table_step3 = C.addFrame("table_step3", "table");
  table_step3->setPosition({1.5, 0.0, 0.5});
  table_step3->setColor(arr{0.8, 0.8, 0.8});
  table_step3->setShape(rai::ST_ssBox, arr{0.5, 0.5, 0.15, 0.001});
  table_step3->setContact(-2);

  auto frame = C.addFrame("obj", "FL_marker");
  frame->setColor({1, 0.5, 0});
  frame->setShape(rai::ST_ssBox, {.2, .2, .2, 0.01});
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

  std::vector<std::string> table_options = {"table", "table_step1", "table_step2", "table_step3"};

  std::random_device rd;
  std::mt19937 gen(rd());

  std::uniform_int_distribution<> distr(0, table_options.size() - 1);

  while (true)
  {
    auto qHome = randn(C.getJointStateDimension()) * 1.;
    qHome.elem(21) = 0.;
    C.setJointState(qHome_original + qHome);

    KOMO komo;
    komo.setConfig(C, true);
    komo.setTiming(1,1,1,0);
    // komo.addControlObjective({}, 0, 1e-1); // This might actually be wrong

    komo.addObjective({}, FS_jointLimits, {}, OT_ineq);
    komo.addObjective({}, FS_accumulatedCollisions, {}, OT_eq, {1e1});

    komo.addObjective({}, make_shared<F_BodyTotalForce>(), {"base_link"}, OT_eq, {1e1});
    komo.addObjective({}, make_shared<F_TotalForce>(), {"obj"}, OT_eq, {1e1});

    int feet_encoding = rand() % 16 + 1;
    if (feet_encoding & 0b0001) komo.addContact_WithPoaFrame(1., "FL_marker", table_options[distr(gen)].c_str(), .8, .05);
    if (feet_encoding & 0b0010) komo.addContact_WithPoaFrame(1., "FR_marker", table_options[distr(gen)].c_str(), .8, .05);
    if (feet_encoding & 0b0100) komo.addContact_WithPoaFrame(1., "RL_marker", table_options[distr(gen)].c_str(), .8, .05);
    if (feet_encoding & 0b1000) komo.addContact_WithPoaFrame(1., "RR_marker", table_options[distr(gen)].c_str(), .8, .05);

    int feet_grasp_encoding = rand() % 16 + 1;
    if (feet_grasp_encoding & 0b0001) komo.addContact_WithPoaFrame(1., "obj", "FL_marker", .8, .05);
    if (feet_grasp_encoding & 0b0010) komo.addContact_WithPoaFrame(1., "obj", "FR_marker", .8, .05);
    if (feet_grasp_encoding & 0b0100) komo.addContact_WithPoaFrame(1., "obj", "RL_marker", .8, .05);
    if (feet_grasp_encoding & 0b1000) komo.addContact_WithPoaFrame(1., "obj", "RR_marker", .8, .05);
    
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
