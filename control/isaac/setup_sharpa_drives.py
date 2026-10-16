import omni.usd
from isaacsim.core.experimental.prims import Articulation

ROBOT_PATH = "/biped_s40_with_sharpa_wave_hands"

# kp взяты из Sharpa actuators в текущем MJCF.
# damping пока ставим консервативно 0.1 для первого теста.
DAMPING = 0.1

KP_BY_JOINT = {
    "thumb_CMC_FE": 6.95,
    "thumb_CMC_AA": 13.2,
    "thumb_MCP_FE": 4.76,
    "thumb_MCP_AA": 6.62,
    "thumb_IP": 0.9,

    "index_MCP_FE": 4.76,
    "index_MCP_AA": 6.62,
    "index_PIP": 0.9,
    "index_DIP": 0.9,

    "middle_MCP_FE": 4.76,
    "middle_MCP_AA": 6.62,
    "middle_PIP": 0.9,
    "middle_DIP": 0.9,

    "ring_MCP_FE": 4.76,
    "ring_MCP_AA": 6.62,
    "ring_PIP": 0.9,
    "ring_DIP": 0.9,

    "pinky_CMC": 1.38,
    "pinky_MCP_FE": 4.76,
    "pinky_MCP_AA": 6.62,
    "pinky_PIP": 0.9,
    "pinky_DIP": 0.9,
}

robot = Articulation(ROBOT_PATH)

applied = 0
missing = []

for side in ("left", "right"):
    for joint_suffix, stiffness in KP_BY_JOINT.items():
        name = f"{side}_{joint_suffix}"

        if name not in robot.dof_names:
            missing.append(name)
            continue

        idx = robot.dof_names.index(name)

        robot.set_dof_gains(
            stiffnesses=[[stiffness]],
            dampings=[[DAMPING]],
            dof_indices=[idx],
        )

        print(
            f"{name}: stiffness={stiffness}, "
            f"damping={DAMPING}"
        )

        applied += 1

stage = omni.usd.get_context().get_stage()
stage.GetRootLayer().Save()

print()
print(f"Applied Sharpa drives: {applied}")

if missing:
    print("Missing joints:")
    for name in missing:
        print("  ", name)

print("USD saved.")