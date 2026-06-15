#include <KOMO/komo.h>
#include <DataGen/rndStableConfigs.h>
#include <Kin/F_collisions.h>
#include <Kin/viewer.h>
#include <Kin/F_pose.h>
#include <Optim/NLP_Solver.h>
#include <PathAlgos/RRT_PathFinder.h>

#include <thread>
#include <memory>


void addBaseObjectives(KOMO& komo, str dof, const arr& qTarget, str obj, str poa)
{
  CHECK_EQ(komo.T, 2, "");

  //freeze joint before touch
  komo.addObjective({0.,1.}, FS_pose, {obj}, OT_eq, {1e1}, {}, 1);
  komo.addObjective({0.,1.}, FS_qItself, {dof}, OT_eq, {1e1}, {}, 1);

  //reach joint target after touch
  komo.addObjective({2.}, FS_qItself, {dof}, OT_eq, {1e1}, qTarget);

  //relative finger during push:
  //stay at POA
  komo.addObjective({1.,2.}, FS_positionRel, {"finger", poa}, OT_eq, {1e1});
}

void sample_POA()
{
    rai::Shape *s = obj->shape;
    arr size = s->size;
    if(s->type()==rai::ST_box || s->type()==rai::ST_ssBox) size.resizeCopy(3);
    else if(s->type()==rai::ST_capsule) size = arr{size(1), size(1), size(0)};
    else {
        return false;
    };
    CHECK_EQ(size.N, 3, "");

    for(uint k=0;;k++){ //try multiple points -> rejection sampling
        if(k>=100){ s=0; break; }
        //random point in box
        arr p = size%(rand(3)-.5);
        obj->ensure_X().applyOnPoint(p);

        //force direction via crossproduct with hinge
        arr forceDir;
        if(j->type>=rai::JT_hingeX && j->type<=rai::JT_hingeZ){
            forceDir = crossProduct(j_axis, p-j_pos);
            op_normalize(forceDir);
        }else if(j->type>=rai::JT_transX && j->type<=rai::JT_transZ){
            forceDir = j_axis;
            op_normalize(forceDir);
        }else NIY;

        //move point away, and project back on surface with PairCollision
        p -= .5*forceDir;
        rai::Transformation Tp=0;
        Tp.pos = p;
        rai::PairCollision coll(s->sscCore(), zeros(1,3), obj->ensure_X(), Tp, s->radius(), 0.);

        p = coll.p1 - coll.rad1 * coll.normal;

        //reject based on distance to axis (for hinge)
        double axisDist = length((eye(3) - j_axis*~j_axis) * (p-j_pos));
        // cout <<"axisDist: " <<axisDist <<endl;
        if(axisDist<.05 && j->type>=rai::JT_hingeX && j->type<=rai::JT_hingeZ){
            cout <<"rejected" <<endl;
            return false;
        }

        double fingerRad = C["finger"]->shape->radius();
        poa->setPosition(p - fingerRad*forceDir);
        poa->set_X()->rot.setDiff(Vector_x, forceDir);
        poa->setParent(obj, true);
        break; //accept point
    }
}

bool sample_primitive(arr state, arr forces)
{
    if(poa->parent) poa->unLink();

    //-- pick a random manip dof
    rai::Dof* dof = manipDofs.rndElem();
    rai::Joint* j = dynamic_cast<rai::Joint*>(dof);

    //-- sample a random target
    dof->limits.reshape(2,-1);
    arr x0 = dof->getDofState();
    arr x1 = rand(dof->limits[0], dof->limits[1]);

    //-- joint axis and point
    arr j_axis = (x1-x0).elem()*j->axis.getArr(); op_normalize(j_axis);
    cout << "====AXIS====" << j->axis.getArr() << endl;
    arr j_pos = j->frame->getPosition();

    //-- get a random shape
    rai::Frame *obj = dof->frame;
    FrameL F = {obj};
    obj->getSubtree(F);
    for(uint i=F.N;i--;) if(!F(i)->shape) F.remove(i);
    obj = F.rndElem();

    //-- sample a random POA on the obj
    sample_POA();

    if(!s) return false;

    C.view(true, "rnd POA");

    //-- solve sequence problem
    ManipulationHelper M(STRING("manip '" <<dof->frame->name <<"'"));
    M.setup_sequence(C, 2);
    addBaseObjectives(*M.komo, dof->frame->name, x1, obj->name, poa->name);

    M.komo->initOrg();
    M.solve();
    M.komo->set_viewer(C.get_viewer());
    M.komo->view(true, STRING(M.info <<'\n' <<*M.ret));
    if(!M.ret->feasible) {
        return false;
    }

    //-- approach motion
    std::shared_ptr<rai::RRT_PathFinder> R1 = M.sub_rrt(0, {}, {"jointX", "jointY", "jointZ"});
    R1->opt.set_stepsize(.02);
    R1->solve();
    R1->view(false, "rrt1");
    if(!R1->ret->feasible) return false;
    
    auto M1 = M.sub_motion(0, 50, true, 1e-2, 1e-2, false, true, true, {"jointX", "jointY", "jointZ"});
    M1->komo->initWithPath_qOrg(R1->get_resampledPath(M1->komo->T));
    M1->komo->addObjective({.0,.75}, FS_accumulatedCollisions, {}, OT_eq, {1e0});

    M1->komo->addObjective({.3,.5}, FS_distance, {"finger", obj->name}, OT_ineq, {1e1}, {-.05});
    M1->komo->addObjective({.2,.7}, FS_distance, {"finger", obj->name}, OT_ineq, {1e1}, {-.02});

    M1->komo->addObjective({.8}, FS_qItself, {}, OT_eq, {1e0}, M1->qTarget); //hit position!
    M1->komo->addObjective({.8}, FS_positionRel, {"finger", "POA"}, OT_eq, {1e1}, {2., 0., 0.}, 1); //hit velocity!

    M1->solve();
    if(!M1->ret->feasible) return false;

    path1 = M1->path;

    poa->unLink();
    return true;
}

int MAIN(int argc,char** argv)
{
    rai::initCmdLine(argc,argv);
    
    rai::Configuration C;
    C.addFile(STRING("scene.g"));
    C.getFrame("finger")->setPosition({0., 0., 1.});
    C.ensure_q();

    //get all collision shapes
    StringA colls;
    for(rai::Frame *f:C.frames)
    {
        if(!(f->name=="finger") && f->shape && f->shape->cont) {
            colls.append({"finger", f->name});
        }
    }
    LOG(0) <<"collision pairs: " <<colls;

    rai::Frame* poa = C.addFrame("POA");
    poa->setShape(rai::ST_sphere, {.01}) .setColor({1., 1., .5});
    C.addFrame("POAdir")->setParent(poa) .setShape(rai::ST_marker, {.1});

    DofL allDofs = C.activeDofs;
    DofL manipDofs = C.activeDofs.sub({3, -1});

    rai::Simulation sim(C, rai::Simulation::_physx, 0);
    sim.writeData = 1;

    arr q_ref = C.getJointState();
    double tau = .01;

    for(uint k=0;k<200;k++)
    {
        sample_primitive(arr state, arr forces)
    }

    return 0;
}

int main()
{
    // Load Config

    // Sample start and end states (stable or in motion)

    // Use LGP for primitive generation

    return 0;
}
