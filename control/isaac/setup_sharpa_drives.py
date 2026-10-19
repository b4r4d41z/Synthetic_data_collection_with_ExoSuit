from isaacsim.core.experimental.prims import Articulation

ROBOT_PATH = "/biped_s40_with_sharpa_wave_hands"

# stiffness из Sharpa MJCF
GAINS = {
    "thumb_CMC_FE": (6.95, 0.1),
    "thumb_CMC_AA": (13.2, 0.1),
    "thumb_MCP_FE": (4.76, 0.1),
    "thumb_MCP_AA": (6.62, 0.1),
    "thumb_IP":     (0.9, 0.1),

    "index_MCP_FE": (4.76, 0.1),
    "index_MCP_AA": (6.62, 0.1),
    "index_PIP":    (0.9, 0.1),
    "index_DIP":    (0.9, 0.1),

    "middle_MCP_FE": (4.76, 0.1),
    "middle_MCP_AA": (6.62, 0.1),
    "middle_PIP":    (0.9, 0.1),
    "middle_DIP":    (0.9, 0.1),

    "ring_MCP_FE": (4.76, 0.1),
    "ring_MCP_AA": (6.62, 0.1),
    "ring_PIP":    (0.9, 0.1),
    "ring_DIP":    (0.9, 0.1),

    "pinky_CMC":    (1.38, 0.1),
    "pinky_MCP_FE": (4.76, 0.1),
    "pinky_MCP_AA": (6.62, 0.1),
    "pinky_PIP":    (0.9, 0.1),
    "pinky_DIP":    (0.9, 0.1),
}


robot = Articulation(ROBOT_PATH)

print("\n=== Setting Sharpa drive gains ===")

applied = 0

for side in ("left", "right"):

    print(f"\n--- {side.upper()} HAND ---")

    for suffix, (stiffness, damping) in GAINS.items():

        name = f"{side}_{suffix}"

        if name not in robot.dof_names:
            print(f"NOT FOUND: {name}")
            continue

        idx = robot.dof_names.index(name)

        robot.set_dof_gains(
            stiffnesses=[[stiffness]],
            dampings=[[damping]],
            dof_indices=[idx],
        )

        applied += 1

        print(
            f"{name:<25} "
            f"Kp={stiffness:<6} "
            f"Kd={damping}"
        )


print("\n=== Verification ===")

for name in [
    "left_index_MCP_FE",
    "left_thumb_CMC_FE",
    "right_index_MCP_FE",
    "right_thumb_CMC_FE",
]:

    if name not in robot.dof_names:
        continue

    idx = robot.dof_names.index(name)

    stiffness, damping = robot.get_dof_gains(
        dof_indices=[idx]
    )

    print(
        name,
        "stiffness =", stiffness.numpy(),
        "damping =", damping.numpy()
    )


print(f"\nApplied gains to {applied} Sharpa joints.")
print("DONE")