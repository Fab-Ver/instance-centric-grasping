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

Se `nvcc --version` mostra già CUDA 12.x, salta l'installazione ma esegui comunque il blocco `.bashrc`.

```bash
# Aggiungi la repo NVIDIA
wget https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2204/x86_64/cuda-keyring_1.1-1_all.deb
sudo dpkg -i cuda-keyring_1.1-1_all.deb
sudo apt-get update
sudo apt-get install -y cuda-toolkit-12-1
```

**PATH permanente** — senza questo, `nvcc` sparisce ad ogni nuova sessione:

```bash
echo 'export PATH=/usr/local/cuda-12.1/bin:$PATH' >> ~/.bashrc
echo 'export CUDA_HOME=/usr/local/cuda-12.1' >> ~/.bashrc
source ~/.bashrc

# Verifica
nvcc --version   # deve mostrare release 12.1
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
**Non usare `uv sync`**: questo progetto usa pip + Python 3.10 per l'inferenza locale (non uv/Python 3.12).

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

> **Se esiste già `wheels/MinkowskiEngine*.whl` nella root del repo, salta al paragrafo
> "Reinstall da wheel" in fondo a questa sezione — non devi ricompilare.**

### 7A. Prima compilazione (~20-30 min, richiede GPU + nvcc)

**Trova la compute capability della tua GPU:**
```bash
nvidia-smi --query-gpu=name --format=csv,noheader
```

| GPU | `TORCH_CUDA_ARCH_LIST` |
|-----|------------------------|
| GTX 1050 Ti / 1060 / 1070 / 1080 | `6.1` |
| RTX 2060 / 2070 / 2080 | `7.5` |
| RTX 3060 / 3070 / 3080 / 3090 | `8.6` |
| RTX 4070 / 4080 / 4090 | `8.9` |
| A100 | `8.0` |
| V100 | `7.0` |

Compilare per tutte le architetture insieme causa OOM. Usa **solo** l'architettura della tua GPU.

```bash
# Sostituisci 6.1 con il valore della tua GPU dalla tabella sopra
export TORCH_CUDA_ARCH_LIST="6.1"
export CUDA_HOME=/usr/local/cuda-12.1
export MAX_JOBS=2   # limita i job paralleli per evitare OOM

# Clona il fork patchato (se non già presente)
[ -d ~/MinkowskiEngine ] || git clone https://github.com/renezurbruegg/MinkowskiEngine.git ~/MinkowskiEngine
cd ~/MinkowskiEngine

# Compila e crea il wheel (~20-30 min)
python3 setup.py bdist_wheel --force_cuda --blas=openblas --cuda_home=$CUDA_HOME
```

### 7B. Salva il wheel nel repo (esegui subito dopo la compilazione)

```bash
cd ~/MinkowskiEngine
WHL=$(ls dist/minkowskiengine*.whl | head -1)
echo "Wheel creato: $WHL"

# Salva nella cartella wheels/ del repo — da qui in poi non si ricompila mai più
mkdir -p ~/Robotics_Project/instance-centric-grasping/wheels
cp "$WHL" ~/Robotics_Project/instance-centric-grasping/wheels/

cd ~/Robotics_Project/instance-centric-grasping
```

### 7C. Installa nel venv (sia prima volta che reinstall)

```bash
# venv già attivo
pip install wheels/minkowskiengine*.whl

# Verifica
python3 -c "import MinkowskiEngine as ME; print('ME OK')"
```

### Reinstall da wheel (venv ricreato, nuova macchina, nuovo membro del team)

> Questo è il percorso normale dopo la prima compilazione — secondi, senza GPU.

```bash
source ~/Robotics_Project/instance-centric-grasping/.venv/bin/activate
cd ~/Robotics_Project/instance-centric-grasping
pip install wheels/minkowskiengine*.whl
python3 -c "import MinkowskiEngine as ME; print('ME OK')"
```

---

## 8. icg\_net — repo, pointnet2 e checkpoint

```bash
# Clone del repo icg_net (se non già presente)
[ -d ~/icg_net ] || git clone https://github.com/renezurbruegg/icg_net.git ~/icg_net

# Compila l'estensione C++ pointnet2 (richiesta da icg_net)
cd ~/icg_net/icg_net/third_party/pointnet2
python setup.py install
cd ~/Robotics_Project/instance-centric-grasping

# Installa icg_net tramite .pth file (metodo unico funzionante:
# icg_net usa pyproject.toml senza setup.py, e pip install -e fallisce con setuptools >= 67)
echo "$HOME/icg_net" > .venv/lib/python3.10/site-packages/icg_net.pth

# Verifica — IMPORTANTE: esegui SEMPRE da ~/Robotics_Project/instance-centric-grasping,
# MAI da dentro ~/icg_net/* (la sottocartella typing/ locale ombreggia la stdlib
# e causa un circular import che fa crashare torch)
python3 -c "import icg_net; print('icg_net OK')"

# Applica il patch a icg_net (fix hydra.experimental + absolute config path)
cp scripts/patches/icg_net.py ~/icg_net/icg_net/icg_net.py

# Clone di icg_benchmark e download del checkpoint
[ -d ~/icg_benchmark ] || git clone https://github.com/renezurbruegg/icg_benchmark.git ~/icg_benchmark
cd ~/icg_benchmark
python scripts/download_data.py
# → checkpoint salvato in: ~/icg_benchmark/data/icgnet/51--0.656/checkpoint.ckpt
cd ~/Robotics_Project/instance-centric-grasping

# Already saved locally in icgnet_weights
```

---

## 9. Build del workspace colcon

Gli eseguibili installati da colcon usano `#!/usr/bin/python3` (Python di sistema) come
shebang — il venv non cambia questo. La soluzione corretta è **PYTHONPATH** (step 11).

NON usare `--symlink-install`: causa `error: option --editable not recognized` con
setuptools >= 64. Usa il build standard:

```bash
source /opt/ros/humble/setup.bash
cd ~/Robotics_Project/instance-centric-grasping
colcon build --packages-select icgnet_main panda_ros2_gazebo icgnet_msgs
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
    config_path:      "~/icg_benchmark/data/icgnet/51--0.656/config.yaml"  # TODO: fix with local path
    icgnet_repo_path: "~/icg_net"
```

---

## 11. Come runnare (ogni sessione)

Gli eseguibili colcon usano `#!/usr/bin/python3` hardcoded — `source .venv/bin/activate`
non ha effetto su di loro. Usa **PYTHONPATH** per esporre le ML deps al Python di sistema.

**PYTHONPATH permanente** — aggiungilo a `~/.bashrc` una volta sola per non doverlo
riesportare ogni sessione:

```bash
echo 'export PYTHONPATH=~/Robotics_Project/instance-centric-grasping/.venv/lib/python3.10/site-packages:$PYTHONPATH' >> ~/.bashrc
source ~/.bashrc
```

**Terminale 1 — Simulazione:**
```bash
source /opt/ros/humble/setup.bash
source ~/Robotics_Project/instance-centric-grasping/install/setup.bash
ros2 launch icgnet_main world.launch.py
```

**Terminale 2 — Nodo ICGNet (attendi "ICGNet caricato correttamente.", ~10-20s):**
```bash
source /opt/ros/humble/setup.bash
source ~/Robotics_Project/instance-centric-grasping/install/setup.bash
ros2 launch icgnet_main icgnet_inference.launch.py
```

**Terminale 3 — Trigger predizione:**
```bash
source /opt/ros/humble/setup.bash
source ~/Robotics_Project/instance-centric-grasping/install/setup.bash
ros2 service call /icgnet/compute_grasps std_srvs/srv/Trigger
```