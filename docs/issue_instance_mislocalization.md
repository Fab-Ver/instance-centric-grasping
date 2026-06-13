# Issue — Grasp etichettati per la classe target ma fisicamente sull'oggetto sbagliato

> Rilevato: 2026-06-13 — branch `main`
> Stato: 🔴 aperto — diagnostica in corso (aggiunto logging `[GRASP_POS]`)

## Sintomo

`execute_grasp {target: 'box'}` con la box a **sinistra** (y=-0.199): il robot tenta di
afferrare la zona centrale/destra della scena (in mezzo a palline + can), **non** la box.

## Scena di riproduzione

`scene_manager` con `target_class:=box target_count:=1`:

| Entity | Modello | Posa (x, y, z) |
|---|---|---|
| `target_obj_0` | cardboard_box | (0.592, **-0.199**, 0.047) |
| `distractor_0` | baseball | (0.483, +0.074, 0.041) |
| `distractor_1` | tennis_ball | (0.669, +0.041, 0.040) |
| `distractor_2` | soup_can | (0.601, +0.222, 0.052) |

## Dati ICGNet (inference)

```
Raw grasp tensors: rot=[296,3,3], centers=[296,3], scores=[296], class_preds=[3]
Reconstructed 3 instance mesh(es).
[RECON] inst_0 → class=box (id=1)
[RECON] inst_1 → class=can (id=2)
[RECON] inst_2 → class=ball (id=5)
[INSTANCES] inst_0=box(id=1, 18g), inst_1=can(id=2, 2g), inst_2=ball(id=5, 276g) | total=296
[SCORES] max=0.4452 min=0.3000 mean=0.3317 | >0.3: 296  >0.5: 0  >0.7: 0
[RECON_DIAG] inst=0: AABB=[0.547,-0.253,-0.041]->[0.638,0.257,0.099] centroid=(0.600,0.010,0.047)
[RECON_DIAG] inst=1: AABB=[0.548,-0.251,-0.020]->[0.637,0.257,0.099] centroid=(0.598,0.043,0.060)
Instance 2: empty mesh, skipping collision object.
[RECON_DIAG] AABB OVERLAP: inst 0 ↔ inst 1.
```

## Indizi e interpretazione

| Indizio | Valore | Lettura |
|---|---|---|
| Centroide istanza box | (0.600, **+0.010**, 0.047) | Box reale a y=-0.199 → istanza mislocalizzata al centro scena |
| AABB inst_0 e inst_1 in Y | -0.25 → +0.26 (**51 cm**) | Ogni istanza copre l'intera scena → impossibile per oggetti separati |
| `AABB OVERLAP inst 0 ↔ inst 1` | warning | Istanze geometricamente sovrapposte |
| Distribuzione grasp | ball=276, box=18, can=2 | 93% dei grasp sulla palla; inst_2 ha **mesh vuota** |
| Scores | max 0.44, mean 0.33, **nessuno >0.5** | Predizione ICGNet di bassa qualità su tutta la scena |

**Causa NON è il centroide né l'executor.** L'executor filtra i grasp per
`semantic_class == box` (`_matches_target`) e va sulla **posizione del singolo grasp**, non
sul centroide. Il centroide è solo un sintomo diagnostico. Il problema è a monte: la
**segmentazione di istanza (Mask3D) di ICGNet è degenere** — assegna ai grasp "box" posizioni
fisicamente sbagliate (centro scena invece che y=-0.199), e ricostruisce istanze che si
estendono su tutta la scena.

## Causa radice CONFERMATA (2026-06-13, run #2)

**Under-segmentation di Mask3D** dentro ICGNet. Il modello collassa più oggetti distinti in
poche istanze che coprono l'intera scena.

Run di conferma (scena: box target + 3 cilindri/lattine, tutti ben separati):
```
class_preds=[2]   ← solo 2 istanze per 4 oggetti
[INSTANCES] inst_0=box(31g), inst_1=can(1141g)
[GRASP_POS] inst_0 box (31g): mean=(0.525,0.085,0.122) y=[-0.193,0.217]   ← span TUTTA la scena
[GRASP_POS] inst_1 can (1141g): mean=(0.603,-0.055,0.118) y=[-0.224,0.234]
[RECON_DIAG] inst=0 centroid=(0.550,-0.028) AABB y=[-0.215,0.218]
[RECON_DIAG] inst=1 centroid=(0.553,-0.021) AABB y=[-0.214,0.218]   ← quasi identica a inst=0
[SCORES] max=0.7511 mean=0.4678 >0.5: 450
```
ICGNet mette ~tutto in una mega-istanza "can" (1141 grasp) + una "box" sparsa di 31 grasp-rumore
distribuiti su tutta la scena. L'executor sceglie il grasp "box" con score più alto → punto
casuale in mezzo ai cilindri → afferra l'oggetto sbagliato.

**Vincoli del modello** (`icgnet_weights/config.yaml`):
- `voxel_size: 0.003` (3mm, risoluzione fine — non è il limite)
- `num_queries: 32` (capacità fino a 32 istanze — non è il limite)
- `add_normals/add_colors/add_z: false` → segmentazione **puramente geometrica** (feature costante)
- Pesi pretrained PyBullet **congelati** → no retrain praticabile.

**Cause scartate:**
- Offset world-frame: `scene_bounds=extract_scene_bounds(coords)` normalizza la pos-encoding sul
  bbox degli oggetti → l'offset assoluto è irrilevante.
- Piano di terra: `workspace_z_min=0.01` lo rimuove già; bin escluso a parte.
- Convex-hull inflation: secondaria, non spiega istanze/grasp scene-wide.

**Causa residua plausibile:** domain gap di viewpoint (training PyBullet multi-view randomizzato
vs nostra singola vista obliqua fissa ~55° da `[0.97,0,0.616]`) e/o assenza del piano di
supporto su cui il modello è stato addestrato.

## Opzioni di fix (modello congelato → solo input o post-processing)

**A) Clustering geometrico post-hoc** (pragmatico, indipendente da ICGNet):
gli oggetti sono fisicamente separati (gap >10cm). Si fa Euclidean/DBSCAN clustering sulla
`preprocessed_cloud` per ottenere le istanze VERE, poi si assegna ogni grasp ICGNet al cluster
più vicino; la classe semantica per-cluster = voto di maggioranza delle label per-grasp ICGNet.
`target=box` → cluster la cui maggioranza è box. Disaccoppia l'esecuzione dalla segmentazione
rotta. Costo: medio. Rischio: la label semantica ICGNet potrebbe restare rumorosa.

**B) Migliorare l'input per far segmentare ICGNet correttamente:**
- B1: viewpoint più top-down (esperimento più economico — solo TF camera + SDF).
- B2: fusione multi-vista (più clouds da angoli diversi) → cloud più completa, vicina al training.
Costo: B1 basso, B2 alto. Rischio: potrebbe non bastare (domain gap).

## Azione presa

Aggiunto logging `[GRASP_POS]` in `grasp_service_node.py` (dopo `[INSTANCES]`): media + range
XYZ delle posizioni grasp per-istanza in world frame. Ha confermato lo span scene-wide.

## Prossimi passi

- [ ] Decidere strategia di fix (A vs B).
- [ ] Se B1: testare camera più top-down e ri-osservare n_istanze.
- [ ] Se A: implementare clustering + mapping grasp→cluster nell'executor/service node.
