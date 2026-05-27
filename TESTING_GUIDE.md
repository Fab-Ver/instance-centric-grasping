# Testing Guide — ICGNet ROS2 Gazebo

Stato del branch: `improved_version_to_test`
Ultimo aggiornamento: 2026-05-27

---

## 0. Setup obbligatorio (ogni nuova shell)

```bash
cd ~/Robotics_Project/instance-centric-grasping
source /opt/ros/humble/setup.bash && source install/setup.bash
```

---

## 1. Build

```bash
colcon build --packages-select panda_description icgnet_msgs icgnet_main
source install/setup.bash
```

---

## Pipeline A — Con GPU (inferenza + salvataggio dati)

### T1 — Simulazione

```bash
ros2 launch icgnet_main world.launch.py
```

Attendi che Gazebo e RViz si aprano e i controller siano attivi:

```bash
ros2 control list_controllers
# panda_arm_controller[active]  panda_hand_controller[active]
```

### T2 — Spawn oggetto

```bash
PKG=$(ros2 pkg prefix icgnet_main)/share/icgnet_main
ros2 run icgnet_main spawn_object --ros-args -p target_type:=beer_can
```

### T3 — Inferenza ICGNet (GPU)

```bash
export PYTHONPATH=~/Robotics_Project/instance-centric-grasping/.venv/lib/python3.10/site-packages:$PYTHONPATH
ros2 launch icgnet_main icgnet_inference.launch.py
# Atteso: "ICGNet loaded successfully." + "ICGNetGraspNode ready"
```

### T4 — Salva dati inferenza (run una volta, poi CTRL+C)

```bash
PKG=$(ros2 pkg prefix icgnet_main)/share/icgnet_main
ros2 run icgnet_main save_inference --ros-args \
  -p object_sdf_path:=$PKG/models/can/beer_can/model.sdf \
  -p object_name:=target_obj \
  -p object_x:=0.65 -p object_y:=0.0 -p object_z:=0.05
```

### T5 — Trigger + grasp

```bash
# Trigger inferenza — i dati vengono auto-salvati dopo 2s
ros2 service call /icgnet/compute_grasps std_srvs/srv/Trigger
# Attendi [SAVED] nei log di T4, poi CTRL+C T4
```

**Opzionale: esegui grasp nella stessa sessione**

```bash
# T5-bis: avvia executor
ros2 launch icgnet_main grasp_execution.launch.py

# T5-ter: trigger grasp
ros2 service call /icgnet/execute_grasp icgnet_msgs/srv/ExecuteGrasp "{target: 'can'}"
```

---

## Pipeline B — Senza GPU (replay dati salvati)

> Prerequisito: aver eseguito la Pipeline A almeno una volta per generare `~/icgnet_inference_data/`.

### T1 — Simulazione

```bash
ros2 launch icgnet_main world.launch.py
```

### T2 — Replay inferenza (sostituisce T3 della Pipeline A)

```bash
ros2 launch icgnet_main icgnet_replay.launch.py \
  inference_dir:=$HOME/icgnet_inference_data
# Atteso: "ReplayInferenceNode ready — N grasps, M collision objects"
```

### T3 — Grasp executor

```bash
ros2 launch icgnet_main grasp_execution.launch.py
```

### T4 — Trigger + grasp

```bash
# Spawna beer_can in Gazebo + pubblica grasps e collision objects salvati
ros2 service call /icgnet/compute_grasps std_srvs/srv/Trigger

# Esegui grasp
ros2 service call /icgnet/execute_grasp icgnet_msgs/srv/ExecuteGrasp "{target: 'can'}"
```

Ogni nuovo trigger rispawna l'oggetto nella stessa posizione — la stessa inferenza viene ripubblicata, permettendo N attempt partendo dallo stesso stato.

---

## Sequenza grasp attesa (log)

```
[FILTER]  N grasps pass score>=0.40  [0.41–0.87]
[PLAN]    score=0.752  inst=0  cls=2  width=0.0712m
[STEP 1/5] PRE-GRASP      → [x, y, z]
[STEP 2/5] APPROACH       → [x, y, z]   fraction=1.00
[STEP 2b]  MICRO-ADVANCE  → [x, y, z]
[STEP 3/5] CLOSING GRIPPER  gap=21.3mm  ✓
[STEP 4/5] LIFTING         → [x, y, z+0.25]
[STEP 5/5] HOME
[SUCCESS]  Grasp completed on attempt 1/5
```

---

## Spawn oggetti (riferimento rapido)

```bash
PKG=$(ros2 pkg prefix icgnet_main)/share/icgnet_main

# Per classe semantica (random dal catalog)
ros2 run icgnet_main spawn_object --ros-args -p target_class:=can
ros2 run icgnet_main spawn_object --ros-args -p target_class:=bottle
ros2 run icgnet_main spawn_object --ros-args -p target_class:=mug

# Modello esatto
ros2 run icgnet_main spawn_object --ros-args -p target_type:=beer_can
ros2 run icgnet_main spawn_object --ros-args -p target_type:=coke_can
ros2 run icgnet_main spawn_object --ros-args -p target_type:=soup_can

# Spawn diretto
ros2 run gazebo_ros spawn_entity.py -entity target_obj \
  -file $PKG/models/can/beer_can/model.sdf -x 0.65 -y 0.0 -z 0.05

# Elimina oggetto
ros2 service call /delete_entity gazebo_msgs/srv/DeleteEntity "{name: 'target_obj'}"
```

---

## Diagnostici

```bash
# Controller
ros2 control list_controllers

# Topic ICGNet
ros2 topic list | grep icgnet
ros2 topic echo /icgnet/grasps_rich --once

# Collision objects in planning scene
ros2 service call /get_planning_scene moveit_msgs/srv/GetPlanningScene \
  '{components: {components: 1}}' 2>/dev/null | grep '"id"'

# Camera pointcloud
ros2 topic hz /camera/rgbd_camera/points   # ~10 Hz atteso

# Joint states
ros2 topic echo /joint_states --once
```

---

## Problemi noti

| Sintomo | Causa | Fix |
|---|---|---|
| Grasp abortisce a STEP 2 (fraction < 0.9) | Collision object blocca path cartesiano | Normale se oggetto in posizione critica — executor tenta il candidato successivo |
| Finger gap < 5mm dopo close | Grasp pose errata o oggetto fuori workspace | ICGNet seleziona automaticamente il candidato successivo |
| `[SAVED]` non compare in T4 | Inferenza non ancora triggerata | Chiama `/icgnet/compute_grasps` prima |
| SpawnEntity fallisce in replay | Entity `target_obj` già presente | Elimina prima: `ros2 service call /delete_entity ...` |
| `collision_object` silenzioso | `return_meshes: false` in icgnet_params.yaml | Imposta `return_meshes: true` e `publish_collision_objects: true` |
| Mesh ICGNet non visibili in RViz | Display non configurato | Il display `ICGNet Collision Meshes` è già nel config RViz — verifica che sia abilitato |
