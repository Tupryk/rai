import numpy as np
import robotic as ry

file_path = "joint_states_fingers_box.txt"

poses = []

with open(file_path, "r") as f:
    for line in f:
        line = line.strip()
        if line:  # skip empty lines
            arr = np.fromstring(line, sep=' ')
            arr[2] += 1
            arr[5] += 1
            arr[8] += 1
            poses.append(arr)
poses = np.array(poses)

C = ry.Config()
C.addFile("/home/eckart/git/robotic/rai-robotModels/g1/g1_free.g")
# C.addFile("./fingersBox.g")

frame = C.addFrame("obj", "world")
frame.setColor([1, 0.5, 0])
frame.setShape(ry.ST.ssBox, [.2, .2, .2, 0.01])
frame.setJoint(ry.JT.free)
frame.setMass(.2)
frame.setContact(1)

C.view(True)
exit()

new_poses = []
for i, pose in enumerate(poses):
    # C.setJointState(pose)
    # C.view(True)

    C.setJointState(np.random.randn(len(C.getJointState()))*0.2)

    komo = ry.KOMO()
    komo.setConfig(C, True)
    komo.setTiming(1, 1, 1, 0)
    komo.addControlObjective([], 0, 1e-1)
    
    komo.addObjective([], ry.FS.jointLimits, [], ry.OT.ineq)
    komo.addObjective([], ry.FS.accumulatedCollisions, [], ry.OT.eq, [1e1])

    komo.addObjective([], ry.FS.position, ["left_rubber_hand_0"], ry.OT.eq, [1e1], pose[:3])
    komo.addObjective([], ry.FS.position, ["right_rubber_hand_0"], ry.OT.eq, [1e1], pose[3:6])
    # komo.addObjective([], ry.FS.position, ["1_fing"], ry.OT.eq, [1e1], pose[:3])
    # komo.addObjective([], ry.FS.position, ["2_fing"], ry.OT.eq, [1e1], pose[3:6])
    komo.addObjective([], ry.FS.pose, ["obj"], ry.OT.eq, [1e1], pose[6:])
    # komo.addObjective([], ry.FS.qItself, [], ry.OT.eq, [1e1], pose)

    ret = ry.NLP_Solver(komo.nlp(), verbose=0).solve()
    if ret.feasible:
        q = komo.getPath()
        C.setJointState(q[0])
        C.view(False)
        new_poses.append(q[0])
    else:
        print(f"Pose {i} infeasible!")

# TODO: When optimizing the legs dont fix the pelvis joint, just fix the box/torso pose.
np.savetxt("humanoid_torso_poses.txt", np.array(new_poses))
