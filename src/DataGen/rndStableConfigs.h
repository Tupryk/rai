#pragma once

#include "../Kin/kin.h"

struct RndStableConfigs_Options {
  RAI_PARAM("RndStableConfigs/", int, verbose, 1)
  RAI_PARAM("RndStableConfigs/", double, frictionCone_mu, .8)
  RAI_PARAM("RndStableConfigs/", double, prob_contact, .5)
};

struct RndStableConfigs {
  RndStableConfigs_Options opt;
  uint totalEvals=0, totalSucc=0;
  bool savePngs=false;
  arr last_sampled_joint_state;
  rai::Graph forces_report;
  int contact_mode_count;

  bool getSample(rai::Configuration& C, const StringA& supports);
  void report();
};
