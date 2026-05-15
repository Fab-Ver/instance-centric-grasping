# Local ICGNet Inference — Setup Guide

Guida completa per far girare il nodo `grasp_service_node` in locale su una macchina con GPU.
Stack: Ubuntu 22.04, Python 3.10, CUDA 12.1, PyTorch 2.2.0.

---

## 0. Prerequisiti: verifica GPU e driver NVIDIA

```bash
nvidia-smi
```

Se il comando fallisce, installa i driver prima di continuare:
```bash
sudo apt-get install -y ubuntu-drivers-common
sudo ubuntu-drivers autoinstall
sudo reboot
```

---

## 1. CUDA Toolkit 12.1

Se `nvcc --version` mostra già CUDA 12.x, salta questa sezione.

```bash
# Aggiungi la repo NVIDIA
wget https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2204/x86_64/cuda-keyring_1.1-1_all.deb
sudo dpkg -i cuda-keyring_1.1-1_all.deb
sudo apt-get update
sudo apt-get install -y cuda-toolkit-12-1

# Aggiungi al PATH (metti anche in ~/.bashrc per renderlo permanente)
export PATH=/usr/local/cuda-12.1/bin:$PATH
export CUDA_HOME=/usr/local/cuda-12.1

# Verifica
nvcc --version
```

---

## 2. Dipendenze di sistema

```bash
sudo apt-get install -y \
    libopenblas-dev \
    build-essential \
    python3-dev \
    python3-venv \
    git
```

---

## 3. Virtual environment Python 3.10

Usiamo un venv Python 3.10 (il Python di sistema su Ubuntu 22.04).
**Non usare `uv sync`** su questa macchina: quello è per il path Kaggle (Python 3.12).

```bash
cd ~/instance-centric-grasping

# Crea il venv nella root del progetto
python3 -m venv .venv

# Attivalo (da fare ogni volta in un nuovo terminale)
source .venv/bin/activate

# Aggiorna pip
pip install --upgrade pip wheel setuptools
```

---

## 4. PyTorch 2.2.0 + CUDA 12.1

```bash
pip install \
    torch==2.2.0+cu121 \
    torchvision==0.17.0+cu121 \
    torchaudio==2.2.0+cu121 \
    --index-url https://download.pytorch.org/whl/cu121

# Verifica (deve stampare True e la versione CUDA)
python3 -c "import torch; print(torch.__version__, torch.cuda.is_available())"
```

---

## 5. PyTorch Geometric

```bash
pip install torch_geometric

pip install \
    pyg_lib \
    torch_scatter \
    torch_sparse \
    torch_cluster \
    torch_spline_conv \
    -f https://data.pyg.org/whl/torch-2.2.0+cu121.html
```

---

## 6. Altre dipendenze ML

```bash
pip install \
    "numpy==1.26.4" \
    scipy \
    hydra-core \
    trimesh \
    einops \
    networkx \
    loguru \
    open3d \
    scikit-learn
```

---

## 7. MinkowskiEngine (fork patchato per PyTorch >= 2.0)

La versione ufficiale NVIDIA non supporta PyTorch 2.x. Usa il fork di renezurbruegg.

**IMPORTANTE — trova prima la compute capability della tua GPU:**
```bash
nvidia-smi --query-gpu=name --format=csv,noheader
```

| GPU | `TORCH_CUDA_ARCH_LIST` |
|-----|------------------------|
| GTX 1080 / 1070 / 1060 | `6.1` |
| RTX 2080 / 2070 / 2060 | `7.5` |
| RTX 3090 / 3080 / 3070 / 3060 | `8.6` |
| RTX 4090 / 4080 / 4070 | `8.9` |
| A100 | `8.0` |
| V100 | `7.0` |

Compilare per tutte le architetture insieme (`6.0 6.1 6.2 7.0 ...`) causa OOM e il processo
viene killato. Usa **solo** l'architettura della tua GPU.

```bash
# Sostituisci 8.6 con il valore della tua GPU dalla tabella sopra
export TORCH_CUDA_ARCH_LIST="8.6"
export CUDA_HOME=/usr/local/cuda-12.1
export MAX_JOBS=2   # limita i job paralleli per evitare OOM

# Clona il fork patchato
git clone https://github.com/renezurbruegg/MinkowskiEngine.git ~/MinkowskiEngine
cd ~/MinkowskiEngine

# Compila e installa (~15-30 min)
python setup.py install --force_cuda --blas=openblas --cuda_home=$CUDA_HOME

cd ~/instance-centric-grasping

# Verifica
python3 -c "import MinkowskiEngine; print('ME OK:', MinkowskiEngine.__version__)"
```

---

## 8. icg\_net — repo, pointnet2 e checkpoint

```bash
# Clone del repo icg_net
git clone https://github.com/renezurbruegg/icg_net.git ~/icg_net

# Compila l'estensione C++ pointnet2 (richiesta da icg_net)
cd ~/icg_net/third_party/pointnet2
python setup.py install
cd ~/instance-centric-grasping

# Installa icg_net. pip install -e fallisce con setuptools >= 67.
# Opzione 1 — setup.py develop:
cd ~/icg_net && python setup.py develop && cd ~/instance-centric-grasping
# Opzione 2 — se fallisce, .pth file (garantito):
#   echo "$HOME/icg_net" > ~/instance-centric-grasping/.venv/lib/python3.10/site-packages/icg_net.pth
#   (trova il path corretto con: python3 -c "import site; print(site.getsitepackages()[0])")

python3 -c "import icg_net; print('icg_net OK')"

# Applica il patch a icg_net (fix hydra.experimental + absolute config path)
cp ~/instance-centric-grasping/scripts/patches/icg_net.py ~/icg_net/icg_net/icg_net.py

# Clone di icg_benchmark e download del checkpoint
git clone https://github.com/renezurbruegg/icg_benchmark.git ~/icg_benchmark
cd ~/icg_benchmark
python scripts/download_data.py
# → checkpoint salvato in: ~/icg_benchmark/data/icgnet/51--0.656/checkpoint.ckpt
cd ~/instance-centric-grasping
```

---

## 9. Build del workspace colcon

Gli eseguibili installati da colcon usano `#!/usr/bin/python3` (Python di sistema) come
shebang — il venv non cambia questo. La soluzione corretta è **PYTHONPATH** (step 11).

NON usare `--symlink-install`: causa `error: option --editable not recognized` con
setuptools >= 64. Usa il build standard:

```bash
source /opt/ros/humble/setup.bash
cd ~/instance-centric-grasping
colcon build --packages-select icgnet_main panda_ros2_gazebo
source install/setup.bash
```

---

## 10. Configura i path in icgnet\_params.yaml

```bash
nano src/icgnet_main/config/icgnet_params.yaml
```

Imposta i due path:
```yaml
icgnet_grasp_node:
  ros__parameters:
    config_path:      "~/icg_benchmark/data/icgnet/51--0.656/config.yaml"
    icgnet_repo_path: "~/icg_net"
```

---

## 11. Come runnare (ogni sessione)

Gli eseguibili colcon usano `#!/usr/bin/python3` hardcoded — `source .venv/bin/activate`
non ha effetto su di loro. Usa **PYTHONPATH** per esporre le ML deps al Python di sistema.

**Terminale 1 — Simulazione:**
```bash
source /opt/ros/humble/setup.bash
source ~/instance-centric-grasping/install/setup.bash
ros2 launch icgnet_main world.launch.py
```

**Terminale 2 — Nodo ICGNet (attendi "ICGNet caricato correttamente.", ~10-20s):**
```bash
source /opt/ros/humble/setup.bash
source ~/instance-centric-grasping/install/setup.bash
export PYTHONPATH=~/instance-centric-grasping/.venv/lib/python3.10/site-packages:$PYTHONPATH
ros2 launch icgnet_main icgnet_inference.launch.py
```

**Terminale 3 — Trigger predizione:**
```bash
source /opt/ros/humble/setup.bash
source ~/instance-centric-grasping/install/setup.bash
ros2 service call /icgnet/compute_grasps std_srvs/srv/Trigger
```

---

## Troubleshooting

| Errore | Causa | Fix |
|--------|-------|-----|
| `ModuleNotFoundError: torch` al lancio | PYTHONPATH non settato | Aggiungi `export PYTHONPATH=.../.venv/lib/python3.10/site-packages:$PYTHONPATH` prima del launch |
| `ModuleNotFoundError: rclpy` | ROS2 non sourcato | `source /opt/ros/humble/setup.bash` prima del launch |
| `Cannot find primary config '...'` | hydra.experimental bug | Copia `scripts/patches/icg_net.py` → `~/icg_net/icg_net/icg_net.py` |
| ME compile **Killed** (OOM) | Troppo architetture CUDA | Usa solo quella della tua GPU (es. `TORCH_CUDA_ARCH_LIST="6.1"`) e `MAX_JOBS=2` |
| `option --editable not recognized` nel colcon build | setuptools >= 64 | Usa `colcon build` senza `--symlink-install` |
| ME compile error: `nvtx3/nvToolsExt.h` | Bug noto in CUDA 12 | Usa il fork patchato di renezurbruegg, non NVIDIA/MinkowskiEngine |
| `cuda_home` non trovato durante ME compile | `CUDA_HOME` non settato | `export CUDA_HOME=/usr/local/cuda-12.1` prima del `python setup.py install` |
| Grasp non pubblicati | `score_threshold` troppo alto | Metti `score_threshold: 0.0` in `icgnet_params.yaml` |
