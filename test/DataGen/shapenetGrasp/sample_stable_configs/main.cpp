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
  rai::Configuration C("./oneFingerBall.yml");
  auto frame = C.addFrame("obj", "world");
  
  frame->setColor(arr{1, 0.5, 0.});
  frame->setShape(rai::ST_sphere, arr{.1});
  frame->setJoint(rai::JT_free);
  frame->setMass(.2);
  frame->setPosition(arr{0., 0., 0.8});
  frame->setContact(1);

  cout <<"configuration space dim=" <<C.getJointStateDimension() <<endl;
  C.view(true);
  
  ofstream dataFile_js;
  
  if(!dataFile_js.is_open()) {
    dataFile_js.open(STRING("joint_states.txt"));
  }
  
  int verbose = 1;

  uint totalEvals=0, totalSucc=0;

  auto qHome_original = C.getJointState();

  std::random_device rd;
  std::mt19937 gen(rd());

  while (true)
  {
    auto qHome = randn(C.getJointStateDimension()) * 1.;
    C.setJointState(qHome_original + qHome);

    KOMO komo;
    komo.setConfig(C, true);
    komo.setTiming(1,1,1,0);
    // komo.addControlObjective({}, 0, 1e0);

    komo.addObjective({}, FS_jointLimits, {}, OT_ineq);
    komo.addObjective({}, FS_accumulatedCollisions, {}, OT_eq, {1e1});

    komo.addObjective({}, make_shared<F_TotalForce>(), {"obj"}, OT_eq, {1e1});

    komo.addContact_WithPoaFrame(1., "obj", "fing", .8, .05);
    komo.addContact_WithPoaFrame(1., "obj", "table", .8, .05);
    komo.addContact_WithPoaFrame(1., "obj", "box", .8, .05);
    // komo.addContact_WithPoaFrame(1., "obj", "wall", .8, .05);
    
    if(verbose>0) komo.set_viewer(C.get_viewer());

    for(uint k=0;k<1;k++){
      // komo.initRandom();

      auto ret = rai::NLP_Solver(komo.nlp())
                .setOptions(rai::OptOptions().set_stopEvals(1000).set_stopTolerance(.001))
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
        arr sampled_joint_state = komo.getPath();
        sampled_joint_state.modRaw().write(dataFile_js);
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
