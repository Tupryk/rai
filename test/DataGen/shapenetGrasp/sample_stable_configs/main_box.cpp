#include <KOMO/komo.h>
#include <DataGen/rndStableConfigs.h>
#include <Kin/F_collisions.h>
#include <Kin/viewer.h>
#include <Kin/F_pose.h>
#include <Optim/NLP_Solver.h>
#include <PathAlgos/RRT_PathFinder.h>

#include <thread>
#include <memory>

//===========================================================================

void TEST(Easy){
  rai::Configuration C("./fingersBox.g");
  cout <<"configuration space dim=" <<C.getJointStateDimension() <<endl;
  auto frame = C.addFrame("obj", "world");
    
  frame->setColor(arr{1, 0.5, 0});
  frame->setShape(rai::ST_box, arr{.2, .2, .2});
  // frame->setShape(rai::ST_sphere, arr{.1});
  frame->setJoint(rai::JT_free);
  frame->setMass(.2);
  frame->setContact(1);
  C.view(true);
  
  RndStableConfigs rndC;
  std::cout << C.getJointStateDimension() << std::endl;

  StringA supports = {
     "1_fing", "2_fing", // "3_fing"
  };

  ofstream dataFile_js;
  ofstream dataFile_f;

  if(!dataFile_js.is_open()) {
    dataFile_js.open(STRING("joint_states.txt"));
  }

  if(!dataFile_f.is_open()) {
    dataFile_f.open(STRING("forces.txt"));
  }

  // arr contact_mode_counts = arr{0, 0, 0};
  arr contact_mode_counts = arr{0, 0};
  int sample_count = 10000;
  while (static_cast<int>(rndC.totalSucc) < sample_count)
  {
    if (rndC.getSample(C, supports))
    {
      // if (contact_mode_counts.elem(rndC.contact_mode_count-1) > (sample_count / contact_mode_counts.N) + 10) {
      // if (rndC.contact_mode_count < 2) {
      //   rndC.totalSucc--;
      //   continue;
      // }
      contact_mode_counts.elem(rndC.contact_mode_count-1)++;
      rndC.last_sampled_joint_state.modRaw().write(dataFile_js);
      dataFile_js << endl;
      dataFile_f << rndC.forces_report << endl << "-" << endl;
    }
  }
  std::cout << "contact_mode_counts: " << contact_mode_counts << std::endl;
}

int MAIN(int argc,char** argv)
{
  rai::initCmdLine(argc,argv);

  rnd.clockSeed();

  testEasy();

  return 0;
}

