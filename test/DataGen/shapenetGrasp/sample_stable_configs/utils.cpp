void run_in_sim()
{
    //-- set control reference
    sim.C.selectJointsByName({"jointX", "jointY", "jointZ"});
    sim.resetSplineRef();
    if(path2.N){
      sim.setSplineRef(path1, {1.});
      sim.setSplineRef(path2, {.5}, true);
    }else{
      sim.setSplineRef(path1, {1.});
    }

    //-- simulate
    while(sim.getTimeToSplineEnd()>0.) {
        sim.step({}, tau, rai::Simulation::_spline);
        C.view();
        rai::wait(.5*tau);
    }
    // plot tracking
    str pltcmd = "plot 'z.sim.dat' us 1:(0) t 'zero'";
    uint n=3;
    for(uint i=0;i<n;i++) pltcmd <<", '' us 1:" <<2+i <<" ls " <<i+1 <<" t 'real" <<i <<"'";
    for(uint i=0;i<n;i++) pltcmd <<", '' us 1:" <<2+n+i <<" ls " <<i+1 <<" w lp t 'ref" <<i <<"'";
    gnuplot(pltcmd, false);

    sim.C.selectJoints(allDofs);
}