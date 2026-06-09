# ICGNet Reconstruction Diagnostics Guide

> Creato: 2026-06-09. Si riferisce ai log `[RECON_DIAG]` aggiunti in
> `grasp_service_node._publish_collision_objects_from_reconstructions`.

## Contesto

Il collega ha riportato che in scena multi-oggetto le mesh ICGNet sembrano un "blob" unico.
**L'analisi del codice ha escluso bug nel codice**: le ricostruzioni sono già separate
per-istanza (loop su latent individuali, `postprocess=False`). La causa è runtime, non
algoritmica. Questo file spiega come interpretare i log diagnostici e cosa modificare.

---

## Come leggere i log [RECON_DIAG]

Dopo ogni `ros2 service call /icgnet/compute_grasps std_srvs/srv/Trigger` in scena
multi-oggetto, cercare questi log in T3 (`icgnet_inference.launch.py`):

```
[RECON_DIAG] N reconstruction(s) received from ICGNet.
[RECON_DIAG] inst=0: 1842v 3680f AABB=[-0.12,-0.08,0.00]->[0.15,0.10,0.22] centroid=(0.01,0.01,0.11)
[RECON_DIAG] inst=1: 2103v 4202f AABB=[-0.05,-0.20,0.00]->[0.25,0.02,0.18] centroid=(0.10,-0.09,0.09)
[RECON_DIAG] AABB OVERLAP: inst 0 ↔ inst 1. Possible Mask3D under-segmentation or hull inflation.
```

---

## Interpretazione e azioni correttive

### Caso A — `[RECON_DIAG] 1 reconstruction(s)` con 2 oggetti in scena

**Causa**: Mask3D ha assegnato i due oggetti a **un solo latent istanza** (under-segmentation).
Non è un bug ICGNet/nostro: è un limite della percezione con oggetti vicini o pointcloud
parziale.

**Conseguenza**: 1 solo CollisionObject (ad es. `icgnet_inst_0`) che copre entrambi gli oggetti.
Il grasp planner sceglie tra i grasp disponibili su quel blob → risultato imprevedibile.

**Azioni**:
1. **Aumentare la distanza tra oggetti** al spawn: `spawn_min_dist` in `spawn_object.py` o
   `scene_manager_params.yaml` → portare da 0.18m a 0.25m.
2. **Verificare qualità pointcloud**: `ros2 topic echo /icgnet/preprocessed_cloud` — se la
   nuvola di punti delle due istanze è fusa (nessuna separazione visiva), aumentare
   `voxel_size` in `icgnet_params.yaml` può aiutare; oppure accettare il limite.
3. **Nessuna modifica al codice ICGNet/patch** — la separazione è impossibile a livello di
   marching cubes se il backbone non distingue le istanze.

---

### Caso B — `[RECON_DIAG] 2 reconstruction(s)` + `[RECON_DIAG] AABB OVERLAP`

**Causa**: Mask3D ha correttamente separato le istanze (2 latent → 2 mesh separate), ma:
- I bounding box per-istanza con margine `±0.05m` si sovrappongono (oggetti a ~10cm), oppure
- Il convex hull di ciascuna mesh "gonfia" lo shape oltre il confine fisico dell'oggetto.

**Conseguenza**: i due `CollisionObject` pubblicati sono distinti (`icgnet_inst_0`, `icgnet_inst_1`)
ma le loro geometrie si intersecano in RViz e in MoveIt. Il planner può rifiutare traiettorie
che avvicinano il gripper alla "zona di sovrapposizione".

**Azioni** (dal meno al più invasivo):

#### B1 — Disabilitare convex hull (riduce inflation)
In `config/icgnet_params.yaml`:
```yaml
collision_use_convex_hull: false   # default: true
```
I CO useranno la mesh marching cubes completa (più fedele ma più lenta per FCL).
Verificare latenza planning — su 2-3 oggetti è accettabile.

#### B2 — Ridurre il margine bounds per-istanza
In `scripts/patches/icg_net.py`, riga 512:
```python
_margin = 0.05   # default: 0.05m (5cm padding per-istanza)
```
Portare a `0.02` o `0.01`. Rischio: se i punti ICGNet dell'istanza sono rumorosi o
mancanti sui bordi, la mesh viene troncata. Testare visivamente.

#### B3 — Clip AABB per-istanza (previene overflow nel CO)
In `grasp_service_node._publish_collision_objects_from_reconstructions`, prima di
`mesh.convex_hull`, aggiungere un clip all'AABB dell'istanza:
```python
if use_hull:
    # Clip mesh to instance bbox before hulling, to prevent overflow between instances.
    inst_pts = embeddings.pointwise_labels == inst_id  # requires passing embeddings
    # ... clip mesh.vertices to inst_pts AABB ± small margin
    mesh = mesh.convex_hull
```
Richiede passare `embeddings` alla funzione — più invasivo.

---

### Caso C — `[RECON_DIAG] 2 reconstruction(s)`, nessun OVERLAP, blob visivo

**Causa**: il blob visivo in RViz era il display `PlanningScene` (ICGNet Collision Meshes)
che renderizza convex-hull contigui — sembrano fusi ma sono CO distinti.

**Verifica**:
```bash
ros2 service call /get_planning_scene moveit_msgs/srv/GetPlanningScene '{components: {components: 1}}'
# Controlla n. di collision_objects e i loro id
```
Se `icgnet_inst_0` e `icgnet_inst_1` sono entrambi presenti → è solo rendering MoveIt,
nessun problema reale. Il `scene_visualizer` mostra le mesh ground-truth separate.

---

## Checklist rapida su GPU

1. Spawnare 2 can con `scene_manager` (o `spawn_object --ros-args -p target_class:=can -p num_objects:=2`).
2. Trigger inference: `ros2 service call /icgnet/compute_grasps std_srvs/srv/Trigger`
3. Leggere log T3:
   - `[RECON_DIAG] N reconstruction(s)` → N==2? Se sì, Mask3D ha separato le istanze. ✅
   - `[RECON_DIAG] AABB OVERLAP`? Se sì → Caso B.
   - N==1 → Caso A.
4. In RViz, controllare il display "Scene Meshes (gz twin)" (`/icgnet/scene_meshes`):
   oggetti separati e alle pose corrette?
5. Controllare "ICGNet Collision Meshes" (`/monitored_planning_scene`):
   2 CO distinti visibili separatamente?

---

## Parametri chiave (summary)

| Param | File | Default | Effetto |
|---|---|---|---|
| `collision_use_convex_hull` | `icgnet_params.yaml` | `true` | `false` → mesh completa, meno inflation |
| `_margin` (bounds per-istanza) | `scripts/patches/icg_net.py:512` | `0.05` | ridurre a 0.02 per meno overlap |
| `spawn_min_dist` | `scene_manager_params.yaml` | `0.18` | aumentare a 0.25 per ridurre under-seg |
| `mesh_resolution` | `scripts/patches/icg_net.py:35` | `64` | aumentare → mesh più fine, +tempo |
