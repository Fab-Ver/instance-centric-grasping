# Issue — ICGNet under-segmenta le scene multi-oggetto (instance/semantic)

> Rilevato: 2026-06-13 — branch `main`
> Stato: 🔴 **confermato, non risolvibile via input** — decisione di scope presa (vedi sotto).
> Questo file è la **fonte unica** sulla diagnostica multi-oggetto ICGNet (assorbe il vecchio
> `recon_diagnostics_guide.md`, eliminato perché partiva da un'assunzione poi smentita).

## Sintomo

`execute_grasp {target: 'box'}` in scena multi-oggetto: il robot afferra l'oggetto **sbagliato**
(es. una lattina) credendolo la box. ICGNet non riesce a separare/etichettare gli oggetti, quindi
non possiamo indicargli quale oggetto prendere.

## Causa radice CONFERMATA

**Under-segmentation di Mask3D** dentro ICGNet: il modello collassa più oggetti distinti in
poche istanze che coprono l'intera scena, e la testa semantica mescola le classi *dentro* lo
stesso oggetto. È un **domain gap** (training PyBullet/TSDF → nostre nuvole Gazebo single-view),
non un bug del nostro codice né della patch.

### Evidenza — run #1 (oggetti sparsi)

Scena: box target + 3 distrattori (cans/ball) ben separati.
```
class_preds=[3]
[INSTANCES] inst_0=box(18g), inst_1=can(2g), inst_2=ball(276g) | total=296
[RECON_DIAG] inst=0 box  AABB y=[-0.253,0.257] centroid=(0.600,0.010)   ← box reale a y=-0.199
[RECON_DIAG] inst=1 can  AABB y=[-0.251,0.257] centroid=(0.598,0.043)   ← AABB ~identica a inst_0
[RECON_DIAG] AABB OVERLAP: inst 0 ↔ inst 1
[SCORES] max=0.4452 mean=0.3317 >0.5: 0
```
Ogni istanza ha un AABB che copre l'**intera** scena (51 cm in Y); il centroide "box" è al centro
scena (y=+0.01), non sulla box (y=-0.199). 93% dei grasp finisce su una sola mega-istanza.

### Evidenza — run #2 (oggetti vicini, cluster ~10 cm)

Scena: box (0.617,0.014) + beer_can (0.524,0.053) + tennis_ball (0.581,-0.062).
```
[FILTER] total=898 → kept=1 (box)  | rejected: target=825 (non-box) low_prepos=72
grasp "box" scelto: icgnet_tcp=[0.519,0.009,0.105]   ← è sulla beer_can (0.524,0.053), NON sulla box (0.617)
```
Su 898 grasp, ICGNet etichetta "box" un solo punto, per giunta posizionato sulla lattina. Avvicinare
gli oggetti **non** ha cambiato nulla.

### Evidenza — run #3 (oggetti GSO, geometria in-distribution)

Test dell'**ultima variabile non isolata**: la geometria degli oggetti. Spawnati 3 **Google Scanned
Objects** reali (gli stessi del training ICGNet, da Gazebo Fuel): canister(can) + choc_box(box) + mug,
a (0.59, y) spaziati 13 cm.
```
class_preds=[6]   ← 6 istanze per 3 oggetti (OVER-segmentation)
[INSTANCES] inst_0=box, inst_1=can, inst_2=box, inst_3=can, inst_4=ball, inst_5=ball
            ← 2 "ball" ALLUCINATE (nessuna palla in scena), nessun "mug", can/box duplicati
14 coppie AABB OVERLAP su 6 istanze
```
Con geometria **in-distribution** il failure mode *cambia* (da under- a **over**-segmentation, con
classi inventate) ma resta inutilizzabile: 6≠3 istanze, classi sbagliate. La geometria reale NON
risolve. (Nota: con `grasp_score_threshold=0.0` l'output ha ~17k grasp quasi tutti score~0 → il
`[GRASP_POS]` è rumoroso, ma `class_preds=6` viene da Mask3D nell'encoder, **indipendente** dal
threshold → le 6 istanze sbagliate sono segmentazione reale.)

## Cosa abbiamo testato ed ESCLUSO

| Ipotesi | Verdetto | Motivo |
|---|---|---|
| **Camera (intrinsics/FOV)** | ❌ non è la causa | 640×480 e ~60° HFOV **identici** al training (`CameraIntrinsic(640,480,540,540,...)`, HFOV 61°). |
| **Viewpoint** | ❌ non è la causa | Elevazione ~53° e distanza ~0.70 m ≈ modo "top" del training (60°, 0.60 m). Top-down testato, nessun cambiamento. |
| **Spaziatura / scala scena** | ❌ non è la causa | Cluster stretto 14×14 cm (= `generate_packed_scene` del training, `x,y∼U(0.08,0.22)`) → ancora merge. |
| **Offset world-frame** | ❌ irrilevante | `scene_bounds=extract_scene_bounds(coords)` ricalcolato ogni forward (`model/icgnet.py:673`) → pos-encoding normalizzata sul bbox. |
| **Piano di terra** | ❌ escluso | `workspace_z_min=0.01` lo rimuove; bin escluso a parte. |
| **Convex-hull inflation** | ⚠️ secondaria | Gonfia l'AABB ma non spiega istanze/grasp scene-wide. |
| **Geometria oggetti (GSO)** | ❌ non è la causa | Oggetti GSO reali in-distribution (run #3) → over-segmentation con classi allucinate. La geometria reale non risolve. |

**Vincoli del modello** (`icgnet_weights/config.yaml`): `voxel_size=0.003` (fine), `num_queries=32`
(capacità OK), `add_normals/add_colors/add_z=false` (segmentazione puramente geometrica), pesi
pretrained **congelati** → no retrain.

## Verdetto

La mis-segmentazione (under- o over- a seconda della scena) è un **domain gap del modello
congelato** e **non è risolvibile agendo sull'input**. Ablazione di 4 variabili — camera,
viewpoint, spaziatura, **geometria oggetti (GSO)** — tutte negative. Esclusa anche la geometria
in-distribution, l'unica variabile rimasta è la **pipeline del sensore**:

> Noi diamo a Mask3D una **depth raw single-view di Gazebo**; il training usa **TSDF-fusion**
> (`data_collection/.../simulation.py::acquire_tsdf` integra la depth in un volume TSDF da cui
> estrae la nuvola — superficie liscia, completa, densità uniforme). Mask3D è verosimilmente
> sensibile a quella qualità di superficie. Single-object funziona perché con 1 oggetto
> `scene_bounds` = bbox dell'oggetto → riempie lo spazio normalizzato; il multi-oggetto collassa.

L'unica leva non testata — **implementare TSDF-fusion multi-vista** (replicare `acquire_tsdf`) — è
la pipeline esatta del training ma è lavoro consistente → **fuori scope** per la deadline.

## Decisione di scope (2026-06-13)

Vincolati a usare ICGNet:
1. **Valutazione su single-object** per classe (`ball`, `box`, `can`): 1 oggetto, **0 distrattori**.
2. **Multi-oggetto = limite documentato** nel report (risultato negativo *misurato* + figura).
3. **Workaround clustering geometrico** (separare per geometria, classificare per forma) considerato
   ma **fuori scope**: il mandato è valutare ICGNet, non sostituirne la segmentazione. Citato come
   future work nel report (preempt alla domanda "perché non avete clusterizzato?").

## Riferimento — come leggere i log diagnostici

Dopo `ros2 service call /icgnet/compute_grasps std_srvs/srv/Trigger`, in T2/T3:

- `[INSTANCES] N instance(s): inst_i=class(id, Ng)` — quante istanze e quanti grasp ciascuna.
- `[GRASP_POS] inst_i class (Ng): mean=(x,y,z) x=[..] y=[..] z=[..]` — spread XYZ dei grasp per
  istanza in world frame. **Range Y largo (~tutta la scena) = under-segmentation.**
- `[RECON_DIAG] inst=i: Nv Nf AABB=[..]->[..] centroid=(..)` — geometria mesh per istanza.
- `[RECON_DIAG] AABB OVERLAP: inst A ↔ inst B` — AABB sovrapposti (sintomo di merge o hull inflation).
- `[SCORES] ... >0.5: K` — qualità grasp; K basso/0 = predizione poco confidente (input OOD).

Codice: `grasp_service_node._publish_collision_objects_from_reconstructions` (RECON_DIAG) e il blocco
`[INSTANCES]`/`[GRASP_POS]` subito dopo l'inferenza.
