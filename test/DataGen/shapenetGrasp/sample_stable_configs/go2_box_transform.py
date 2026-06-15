import numpy as np
import robotic as ry

file_path = "gobox_table.txt"

poses = []

with open(file_path, "r") as f:
    for line in f:
        line = line.strip()
        if line:  # skip empty lines
            arr = np.fromstring(line, sep=' ')
            poses.append(arr)
poses = np.array(poses)

C = ry.Config()
C.addFile("/home/eckart/git/robotic/rai-robotModels/go2/go2.yml")
C.getFrame("base_link").setPosition([0., 0., 2.0])

fl = C.addFrame("FL_marker", "FL_foot_0")
fl.setColor([1, 0, 0])
fl.setShape(ry.ST.sphere, [.03])
fl.setMass(.001)
fl.setContact(1)

fr = C.addFrame("FR_marker", "FR_foot_0")
fr.setColor([1, 0, 0]);
fr.setShape(ry.ST.sphere, [.03])
fr.setMass(.001)
fr.setContact(1)

rl = C.addFrame("RL_marker", "RL_foot_0")
rl.setColor([1, 0, 0])
rl.setShape(ry.ST.sphere, [.03])
rl.setMass(.001)
rl.setContact(1)

rr = C.addFrame("RR_marker", "RR_foot_0")
rr.setColor([1, 0, 0])
rr.setShape(ry.ST.sphere, [.03])
rr.setMass(.001)
rr.setContact(1)

table = C.addFrame("table")
table.setPosition([0., 0., 0.1])
table.setColor([0.8, 0.8, 0.8])
table.setShape(ry.ST.ssBox, [10., 10., 0.05, 0.001])
table.setContact(1)


frame = C.addFrame("obj", "FL_marker")
# frame = C.addFrame("obj", "world")
frame.setColor([1, 0.5, 0])
frame.setShape(ry.ST.ssBox, [.2, .2, .2, 0.01])
frame.setJoint(ry.JT.free)
frame.setMass(7.5)
frame.setContact(1)
frame.setPosition([0., 0., 0.])

table_step1 = C.addFrame("table_step1", "table")
table_step1.setPosition([1.5, 0.0, 0.2])
table_step1.setColor([0.8, 0.8, 0.8])
table_step1.setShape(ry.ST.ssBox, [2.5, 2.5, 0.15, 0.001])
table_step1.setContact(-2)

table_step2 = C.addFrame("table_step2", "table")
table_step2.setPosition([1.5, 0.0, 0.35])
table_step2.setColor([0.8, 0.8, 0.8])
table_step2.setShape(ry.ST.ssBox, [1.5, 1.5, 0.15, 0.001])
table_step2.setContact(-2)

table_step3 = C.addFrame("table_step3", "table")
table_step3.setPosition([1.5, 0.0, 0.5])
table_step3.setColor([0.8, 0.8, 0.8])
table_step3.setShape(ry.ST.ssBox, [0.5, 0.5, 0.15, 0.001])
table_step3.setContact(-2)

# C.view(True)

state_vec = np.zeros(26)
state_vec[0] = 0
state_vec[1] = 0
state_vec[2] = 1
state_vec[3] = 1
state_vec[4] = 0
state_vec[5] = 0
state_vec[6] = 0
state_vec[7:-7] = 0

state_vec[15] = -1
state_vec[16] = -1
state_vec[17] = -1
state_vec[18] = -1

## + Box ##
# state_vec[21] += - 0.1 - 0.025
state_vec[19] = 1
state_vec[20] = 0
state_vec[21] = 0.3
state_vec[22] = 1
state_vec[23] = 0
state_vec[24] = 0
state_vec[25] = 0

new_poses = []
for i, pose in enumerate(poses):
    C.setJointState(pose)
    # C.view(True)
    pose[:7] = C.getFrame("base_link").getPose()
    pose[-7:] = C.getFrame("obj").getPose()
    state_vec = pose.copy()
    state_vec[7]  =  pose[7]   # FL_hip    (no flip)
    state_vec[8]  =  pose[11]  # FL_thigh  (no flip)
    state_vec[9]  =  pose[15]  # FL_calf   (no flip)

    state_vec[10] =  pose[8]   # FR_hip    (flip: 180° around X)
    state_vec[11] =  pose[12]  # FR_thigh  (no flip)
    state_vec[12] =  pose[16]  # FR_calf   (no flip)
    
    state_vec[13] =  pose[9]   # RL_hip    (no flip)
    state_vec[14] =  pose[13]  # RL_thigh  (flip: 180° around Y)
    state_vec[15] = pose[17]  # RL_calf   (flip: 180° around Y)
    
    state_vec[16] = pose[10]  # RR_hip    (flip: both)
    state_vec[17] = pose[14]  # RR_thigh  (flip: both)
    state_vec[18] = pose[18]  # RR_calf   (flip: both)
    print(state_vec[7:-7])
    print(pose.shape)
    new_poses.append(state_vec)

np.savetxt("fixed_gobox_table.txt", np.array(new_poses))
