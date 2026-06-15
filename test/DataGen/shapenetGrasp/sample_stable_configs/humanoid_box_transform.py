import numpy as np
import robotic as ry

file_path = "joint_states_humanoid_grasp_new.txt"

poses = []

with open(file_path, "r") as f:
    for line in f:
        line = line.strip()
        if line:  # skip empty lines
            arr = np.fromstring(line, sep=' ')
            poses.append(arr)
poses = np.array(poses)

C = ry.Config()
C.addFile("/home/eckart/git/robotic/rai-robotModels/g1/g1_free.g")
C.getFrame("pelvis").setPosition([0., 0., 2.0])

table = C.addFrame("table")
table.setPosition([0., 0., 0.1])
table.setColor([0.8, 0.8, 0.8])
table.setShape(ry.ST.ssBox, [10., 10., 0.05, 0.001])
table.setContact(1)


frame = C.addFrame("obj", "left_rubber_hand_0")
# frame = C.addFrame("obj", "world")
frame.setColor([1, 0.5, 0])
frame.setShape(ry.ST.ssBox, [.2, .2, .2, 0.01])
frame.setJoint(ry.JT.free)
frame.setMass(7.5)
frame.setContact(1)
frame.setPosition([0., 0., 0.])

# C.view(True)

new_poses = []
for i, pose in enumerate(poses):
    C.setJointState(pose)
    # C.view(True)
    pose[:7] = C.getFrame("pelvis").getPose()
    pose[-7:] = C.getFrame("obj").getPose()
    print(pose)
    print(pose.shape)
    new_poses.append(pose)

np.savetxt("fixed_humanoid_box_grasps.txt", np.array(new_poses))
