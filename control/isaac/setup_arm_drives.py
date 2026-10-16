import omni.usd
from isaacsim.core.experimental.prims import Articulation

ROBOT_PATH = "/biped_s40_with_sharpa_wave_hands"

# stiffness берём из MJCF.
# damping=20 пока используем как рабочее тестовое значение:
# на zarm_r1_joint оно уже подтвердило нормальную работу.
GAINS = {
    "zarm_l1_joint": (500.0, 20.0),
    "zarm_l2_joint": (500.0, 20.0),
    "zarm_l3_joint": (300.0, 20.0),
    "zarm_l4_joint": (300.0, 20.0),
    "zarm_l5_joint": (150.0, 20.0),
    "zarm_l6_joint": (150.0, 20.0),
    "zarm_l7_joint": (150.0, 20.0),

    "zarm_r1_joint": (500.0, 20.0),
    "zarm_r2_joint": (500.0, 20.0),
    "zarm_r3_joint": (300.0, 20.0),
    "zarm_r4_joint": (300.0, 20.0),
    "zarm_r5_joint": (150.0, 20.0),
    "zarm_r6_joint": (150.0, 20.0),
    "zarm_r7_joint": (150.0, 20.0),
}

robot = Articulation(ROBOT_PATH)

print("Applying arm drive gains...")

for name, (stiffness, damping) in GAINS.items():
    if name not in robot.dof_names:
        print(f"NOT FOUND: {name}")
        continue

    idx = robot.dof_names.index(name)

    robot.set_dof_gains(
        stiffnesses=[[stiffness]],
        dampings=[[damping]],
        dof_indices=[idx],
    )

    print(
        f"{name}: index={idx}, "
        f"stiffness={stiffness}, damping={damping}"
    )

# Сохраняем изменения в текущий USD
stage = omni.usd.get_context().get_stage()
stage.GetRootLayer().Save()

print("Done. USD saved.")