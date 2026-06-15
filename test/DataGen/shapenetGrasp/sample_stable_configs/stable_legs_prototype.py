import numpy as np
import robotic as ry

C = ry.Config()
C.addFrame("world")

frame = C.addFrame("table")
frame.setColor([0.8, 0.8, 0.8])
frame.setShape(ry.ST.ssBox, [5., 5., .2, 0.01])
frame.setContact(1)

def make_foot_ref(foot, ref, pos):
    frame = C.addFrame(ref, foot)
    frame.setColor([1, 0, 0])
    frame.setRelativePosition(pos)
    frame.setShape(ry.ST.sphere, [.01])

frame = C.addFrame("l_foot", "world")
frame.setColor([1, 0.5, 0])
frame.setPosition([0.3, 0, 1])
frame.setShape(ry.ST.ssBox, [.07, .15, .03, 0.01])
frame.setJoint(ry.JT.free)
frame.setContact(1)

make_foot_ref("l_foot", "l_ref1", [.035, .075, -0.015])
make_foot_ref("l_foot", "l_ref2", [.035, -.075, -0.015])
make_foot_ref("l_foot", "l_ref3", [-.035, .075, -0.015])
make_foot_ref("l_foot", "l_ref4", [-.035, -.075, -0.015])

frame = C.addFrame("r_foot", "world")
frame.setColor([1, 0.5, 0])
frame.setPosition([-0.3, 0, 1])
frame.setShape(ry.ST.ssBox, [.07, .15, .03, 0.01])
frame.setJoint(ry.JT.free)
frame.setContact(1)

make_foot_ref("r_foot", "r_ref1", [.035, .075, -0.015])
make_foot_ref("r_foot", "r_ref2", [.035, -.075, -0.015])
make_foot_ref("r_foot", "r_ref3", [-.035, .075, -0.015])
make_foot_ref("r_foot", "r_ref4", [-.035, -.075, -0.015])

frame = C.addFrame("center_of_mass", "world")
frame.setColor([1, 0.5, 0])
frame.setPosition([0, 0, 2])
frame.setShape(ry.ST.marker, [.1])
frame.setJoint(ry.JT.free)

frame = C.addFrame("center_of_mass_geom", "center_of_mass")
frame.setColor([.1, .3, .8])
frame.setShape(ry.ST.sphere, [.05])

C.view(True)

q_dim = len(C.getJointState())
for i in range(10_000):
    C.setJointState( np.random.randn(q_dim) * 0.2 )

    komo = ry.KOMO()
    komo.setConfig(C, True)
    komo.setTiming(1, 1, 1, 0)
    komo.addControlObjective([], 0, 1e-1)
    
    komo.addObjective([], ry.FS.jointLimits, [], ry.OT.ineq)
    komo.addObjective([], ry.FS.accumulatedCollisions, [], ry.OT.eq, [1e1])

    # Left foot
    s = np.random.randint(low=1, high=5)
    points = np.random.choice([1, 2, 3, 4], (s,), replace=False)

    for lp in points:
        komo.addObjective([], ry.FS.position, [f"l_ref{lp}"], ry.OT.eq, [0, 0, 1e1], [.0, .0, .1])

    # Right foot
    s = np.random.randint(low=1, high=5)
    points = np.random.choice([1, 2, 3, 4], (s,), replace=False)

    for lp in points:
        komo.addObjective([], ry.FS.position, [f"r_ref{lp}"], ry.OT.eq, [0, 0, 1e1], [.0, .0, .1])

    # TODO: Define an "in convex hull of points" feature in rai or "distance to in convex hull of points".

    ret = ry.NLP_Solver(komo.nlp(), verbose=0).solve()
    if ret.feasible:
        q = komo.getPath()
        C.setJointState(q[0])
        C.view(True)
    else:
        print(f"Pose {i} infeasible!")
