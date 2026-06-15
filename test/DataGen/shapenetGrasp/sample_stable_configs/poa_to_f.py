import re
import json
import pickle
import numpy as np
import robotic as ry


def load_stable_configs(joints_path: str, forces_path: str) -> list[dict]:
    
    configs = []
    with open(joints_path) as f:
        for line in f:
            joint_state = np.array([float(x) for x in line.split()])
            config = {"joint_state": joint_state}
            configs.append(config)

    forces = []
    index = 0
    with open(forces_path) as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            if line == "-":
                if forces:
                    configs[index]["forces"] = forces
                    index += 1
                    forces = []
                continue

            json_line = re.sub(r'(\w+):', r'"\1":', line)
            obj = json.loads(json_line)
            forces.append({
                "poa": obj["poa"],
                "vec": obj["force"],
                "parent": obj["from"].rsplit("_", 1)[0],
            })

    if forces:
        configs[index]["forces"] = forces
        index += 1

    assert index == len(configs)

    return configs


def poa_to_joints(poa: np.ndarray, force: np.ndarray,
                  parent: str, C_copy: ry.Config) -> np.ndarray:
    
    C_copy.addFrame("poa", parent) \
        .setPosition(poa)
    y, J = C_copy.eval(ry.FS.position, ["poa"])
    tau = J.T @ force

    return tau, J


if __name__ == "__main__":

    C = ry.Config()
    C.addFile(ry.raiPath("../rai-robotModels/scenarios/pandasTable.g"))
    
    frame = C.addFrame("obj", "world") \
        .setPosition([0., 0., .7]) \
        .setColor([0., .4, .8]) \
        .setShape(ry.ST.sphere, [.03]) \
        .setMass(.08) \
        .setContact(1) \
        .setJoint(ry.JT.trans3, [
            -1.0, -1.0, 0.5,
             1.0,  1.0, 2.0
        ])

    stable_configs = load_stable_configs("joint_states.txt", "forces.txt")
    Js = []
    for config in stable_configs:
        
        J_ = []
        tau = np.zeros_like(C.getJointState())
        for i, force in enumerate(config["forces"]):
            
            C_copy = ry.Config()
            C_copy.addConfigurationCopy(C)
            C_copy.setJointState(config["joint_state"])
            
            tau_i, J = poa_to_joints(force["poa"], force["vec"], force["parent"], C_copy)
            tau += tau_i
            J_.append(J)

            del C_copy
        Js.append(J_)

    with open("Js.pkl", "wb") as f:
        pickle.dump(Js, f)
