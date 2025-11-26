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
  // rai::Configuration C(rai::raiPath("../rai-robotModels/scenarios/pandasTable.g"));
  rai::Configuration C(rai::raiPath("../rai-robotModels/g1/g1.g"));
  // rai::Configuration C("fingerRamp.g");
  cout <<"configuration space dim=" <<C.getJointStateDimension() <<endl;
  auto frame = C.addFrame("obj", "world");
  // frame->setPosition(arr{0., 0., .3});
  frame->setPosition(arr{0., 0.2, 0.9});
  frame->setPosition(arr{0.5, 0.0, 0.9});
  
  // frame->setColor(arr{0., .4, .8});
  // frame->setShape(rai::ST_sphere, arr{.03});
  // frame->setJoint(rai::JT_trans3, arr{
  //   -1.0, -1.0, 0.5,
  //    1.0,  1.0, 2.0
  // });
  // frame->setMass(.08);
    
  // frame->setColor(arr{1, 0.5, 0});
  // // frame->setShape(rai::ST_box, arr{.2, .2, .2});
  // // frame->setShape(rai::ST_sphere, arr{.1});
  // frame->setShape(rai::ST_sphere, arr{.03});
  // frame->setJoint(rai::JT_free);
  // frame->setMass(.2);

  frame->setColor(arr{1, 0.5, 0});
  frame->setShape(rai::ST_ssBox, arr{0.211, 0.26, 0.17, 0.001});
  frame->setJoint(rai::JT_free);
  frame->setMass(.22);
  
  frame->setContact(1);
  C.view(true);
  // C.animate();
  
  RndStableConfigs rndC;
  // rndC.savePngs=true;
  std::cout << C.getJointStateDimension() << std::endl;

  // StringA supports = {
  //   "l_panda_coll0", "l_panda_coll1", "l_panda_coll2", "l_panda_coll3", "l_panda_coll4", "l_panda_coll5", "l_panda_coll6", "l_panda_coll7",
  //   "r_panda_coll0", "r_panda_coll1", "r_panda_coll2", "r_panda_coll3", "r_panda_coll4", "r_panda_coll5", "r_panda_coll6", "r_panda_coll7",
  // };
  // StringA supports = {
  //   "l_panda_link0", "r_panda_link0",
  //   "l_panda_joint1", "l_panda_joint2", "l_panda_joint3", "l_panda_joint4", "l_panda_joint5", "l_panda_joint6", "l_panda_joint7",
  //   "r_panda_joint1", "r_panda_joint2", "r_panda_joint3", "r_panda_joint4", "r_panda_joint5", "r_panda_joint6", "r_panda_joint7",
  //   "l_panda_finger_joint1", "l_panda_rightfinger_0", "r_panda_finger_joint1", "r_panda_rightfinger_0",
  // };

  StringA supports = {
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
    "left_shoulder_roll_joint", "right_shoulder_roll_joint",
    "left_ankle_pitch_joint", "right_ankle_pitch_joint",
    "left_shoulder_yaw_joint", "right_shoulder_yaw_joint",
    "left_ankle_roll_joint", "right_ankle_roll_joint",
    "left_elbow_joint", "right_elbow_joint",
    "left_wrist_roll_joint", "right_wrist_roll_joint",
    "left_wrist_pitch_joint", "right_wrist_pitch_joint",
    "left_wrist_yaw_joint", "right_wrist_yaw_joint",
    "left_rubber_hand_0", "right_rubber_hand_0"
  };


  // StringA supports = {
  //    "box", "wall", "l_fing"
  // };
  // StringA supports = {
  //   "l_fing", "table"
  // };

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
  int sample_count = 2000000;
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

