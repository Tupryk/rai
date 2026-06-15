1_base: { pose: [0, 0, 0], multibody: true, multibody_gravity: false }
1_fake(1_base): {  }
1_joint1(1_fake): { joint: transX, limits: [-0.5, 0.5], mass: 0.01, inertia: [0, 0, 0], sampleUniform: 1 }
1_joint2(1_joint1): { joint: transY, limits: [-0.5, 0.5], mass: 0.01, inertia: [0, 0, 0], sampleUniform: 1 }
1_fing(1_joint2): { joint: transZ, limits: [-0.5, 0.5], shape: sphere, size: [0.04], color: [0.5, 1, 1], contact: 1, mass: 0.1, inertia: [0.00025, 0.00025, 0.00025], sampleUniform: 1 }

2_base: { pose: [0, 0, 0], multibody: true, multibody_gravity: false }
2_fake(2_base): {  }
2_joint1(2_fake): { joint: transX, limits: [-0.5, 0.5], mass: 0.01, inertia: [0, 0, 0], sampleUniform: 1 }
2_joint2(2_joint1): { joint: transY, limits: [-0.5, 0.5], mass: 0.01, inertia: [0, 0, 0], sampleUniform: 1 }
2_fing(2_joint2): { joint: transZ, limits: [-0.5, 0.5], shape: sphere, size: [0.04], color: [0.5, 1, 1], contact: 1, mass: 0.1, inertia: [0.00025, 0.00025, 0.00025], sampleUniform: 1 }

#3_base: { pose: [0, 0, 0], multibody: true, multibody_gravity: false }
#3_fake(3_base): {  }
#3_joint1(3_fake): { joint: transX, limits: [-0.5, 0.5], mass: 0.01, inertia: [0, 0, 0], sampleUniform: 1 }
#3_joint2(3_joint1): { joint: transY, limits: [-0.5, 0.5], mass: 0.01, inertia: [0, 0, 0], sampleUniform: 1 }
#3_fing(3_joint2): { joint: transZ, limits: [-0.5, 0.5], shape: sphere, size: [0.04], color: [0.5, 1, 1], contact: 1, mass: 0.1, inertia: [0.00025, 0.00025, 0.00025], sampleUniform: 1 }

world: {  }
#table(world): { pose: [0, 0, 0.1], shape: box, size: [1, 1, 0.1], color: [0.6], contact: 1 }
##obj: { pose: [0, 0, 0.3], limits: [-0.5, -0.5, 0, 0.5, 0.5, 1], shape: box, size: [0.1, 0.1, 0.1], color: [1, 0.5, 0], contact: 1, mass: 0.2, inertia: [0.002, 0.002, 0.002], sampleUniform: 1 }
#wall(table): { pose: [0.45, 0, 0.3], shape: box, size: [0.1, 1, 0.5], color: [0.6], contact: 1 }
#box(table): { pose: [-0.05, -0.35, 0.25], shape: box, size: [0.9, 0.3, 0.4], color: [0.6], contact: 1 }
