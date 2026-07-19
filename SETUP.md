# SETUP — Rebuilding This Environment From Nothing

*If you lose this machine: `git clone` gets you the code, but the code isn't the whole
project — the toolchain around it (ROS2, PX4, Gazebo, MAVROS, Python/LaTeX deps) lives
outside git and has to be reinstalled. This is that list. Versions below are what this
project was actually developed and validated against (captured 2026-07-19) — matching
them exactly isn't mandatory, but if results stop reproducing, a version drift here is
the first thing to suspect.*

**What's in git vs. what isn't:**
- **In git** (`github.com/IshanJakkulwar/mppi_controller`): all of `mppi_controller` —
  code, scripts, paper, plan docs, committed trial data.
- **NOT in git — separate installs, listed below:** the OS, ROS2 itself, MAVROS, Gazebo,
  and critically **PX4-Autopilot, which is its own separate git clone at
  `~/PX4-Autopilot`, entirely outside this repo.** Losing the laptop without a note like
  this one means losing track of which PX4 version everything was validated against.

---

## 1. Base OS

**Ubuntu 22.04 LTS (Jammy)** — this is a hard requirement of ROS2 Humble, not a
preference. Nothing here has been tried on 24.04.

## 2. ROS2 Humble

Installed via apt (not built from source). Official instructions:
[docs.ros.org — Humble installation (Ubuntu)](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html).

```bash
sudo apt update && sudo apt install ros-humble-desktop
sudo apt install python3-colcon-common-extensions
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
```

Installed package version on this machine: `ros-humble-desktop 0.10.0-1jammy`.

## 3. MAVROS

Via apt, as part of the ROS2 ecosystem (not built from source):

```bash
sudo apt install ros-humble-mavros ros-humble-mavros-extras ros-humble-mavros-msgs
```

**Required extra step** — MAVROS needs GeographicLib's datum/geoid data files, which
apt does *not* pull in automatically:

```bash
sudo /opt/ros/humble/lib/mavros/install_geographiclib_datasets.sh
```

Installed version on this machine: `ros-humble-mavros 2.14.0-1jammy`.

## 4. Gazebo (two different simulators — you need both, for different purposes)

- **Gazebo Harmonic** (`gz sim`, "Gazebo Sim" version 8.x) — used for all **SITL** work
  (everything in Phase 0–2, the entire main dataset). Install per [Gazebo Harmonic on
  Ubuntu](https://gazebosim.org/docs/harmonic/install_ubuntu):
  ```bash
  sudo apt install lsb-release curl gnupg
  curl https://packages.osrfoundation.org/gazebo.gpg -o /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
  echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null
  sudo apt update && sudo apt install gz-harmonic
  ```
  Installed version on this machine: **Gazebo Sim 8.14.0**.

- **Gazebo Classic 11** (`gazebo`, a different, older simulator binary) — needed **only
  for HITL** (`HITL_PLAN.md`); PX4's hardware-in-the-loop mode does not support Harmonic.
  **Not yet installed on this machine as of 2026-07-19** — install when HITL work
  actually starts:
  ```bash
  curl -sSL http://get.gazebosim.org | sh   # installs classic gazebo11
  ```
  Both simulators coexist fine on one machine (different binaries, `gazebo` vs `gz sim`).

## 5. PX4-Autopilot (separate clone, NOT part of this git repo)

```bash
cd ~
git clone https://github.com/PX4/PX4-Autopilot.git --recursive
cd PX4-Autopilot
git checkout v1.15.4          # the exact tag this project was validated against
bash ./Tools/setup/ubuntu.sh  # installs PX4's own build dependencies
```

Then build once to confirm it works (also does a first-time toolchain setup):
```bash
make px4_sitl gz_x500
```
(Ctrl-C once you see it running — this is just a smoke test, `run_trial.py` drives it
for real.)

**If reproducing exact historical results matters**, stay on `v1.15.4`. If starting HITL
work fresh, a newer PX4 release is probably fine — but re-verify the calibration ladder
in `PROJECT_PLAN.md` (stress-condition timings, deadline margins) since PX4 release
changes can shift baseline timing behavior.

## 6. Eigen3 (C++ linear algebra — required by `mppi_controller`)

```bash
sudo apt install libeigen3-dev
```
Installed version on this machine: `3.4.0`.

## 7. Build tools

```bash
sudo apt install build-essential cmake git
```
(`python3-colcon-common-extensions` already covered in step 2.)

## 8. Python packages (analysis scripts)

```bash
sudo apt install python3-scipy python3-numpy python3-matplotlib
```
Versions this project was validated against: scipy 1.8.0, numpy 1.26.4, matplotlib
3.5.1. (The scipy/numpy version-mismatch warning you may see on import is cosmetic —
everything still runs correctly; it hasn't been worth chasing down.)

## 9. tmux

```bash
sudo apt install tmux
```
Version used: 3.2a. Nothing project-specific here — used purely as your terminal
multiplexer for running PX4/Gazebo/MAVROS/the node across panes.

## 10. LaTeX / paper compilation

**No LaTeX toolchain was ever installed on the primary dev machine** — every compile of
`paper/main.tex` was done on **Overleaf** (upload `main.tex` + `references.bib` +
`figures/`, it has `IEEEtran` built in). This remains the recommended path — it's zero
setup and matches what's already been used. If you want a local alternative instead:
```bash
sudo apt install texlive-full   # large; texlive-latex-extra + texlive-fonts-recommended
                                 # + texlive-bibtex-extra is a lighter alternative if
                                 # texlive-full is more than you want
```

## 11. Git / GitHub access

```bash
git clone git@github.com:IshanJakkulwar/mppi_controller.git ~/mppi_research_ws/src/mppi_controller
```
Needs your SSH key added to the new machine and to your GitHub account (or switch the
remote to HTTPS with a personal access token). This repo **is** the `src/` package
directory of the workspace — `~/mppi_research_ws` itself (the `build/`/`install/`/`log/`
siblings) is not tracked and regenerates automatically from `colcon build`.

## 12. First build, end to end

```bash
mkdir -p ~/mppi_research_ws/src
cd ~/mppi_research_ws/src
git clone git@github.com:IshanJakkulwar/mppi_controller.git
cd ~/mppi_research_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select mppi_controller     # run from the WORKSPACE root, not
                                                    # from inside src/mppi_controller —
                                                    # doing it from the wrong directory
                                                    # creates stray build/install/log
                                                    # dirs inside the git repo
source install/setup.bash
```

## 13. Sanity check that everything is wired up

In one terminal:
```bash
cd ~/PX4-Autopilot && make px4_sitl gz_x500
```
In a second terminal (after the sim is flying and stable):
```bash
source /opt/ros/humble/setup.bash && source ~/mppi_research_ws/install/setup.bash
ros2 launch mavros px4.launch fcu_url:=udp://:14540@127.0.0.1:14557
```
In a third terminal:
```bash
cd ~/mppi_research_ws/src/mppi_controller
python3 scripts/run_trial.py --condition adaptive --trials 1
```
If that produces a `results/adaptive/trial_01.csv` with `VALID trial saved` in the
output, the whole toolchain is correctly reproduced. (In practice `run_trial.py` starts
its own PX4/Gazebo/MAVROS lifecycle per trial via `--restart-sim`, so you don't normally
need the first two terminals for real batches — this manual sequence is purely to
smoke-test a from-scratch install.)

---

**See also:** `HANDOFF.md` for the project's current state, decisions, and remaining
work queue; `HITL_PLAN.md` for the additional hardware-specific setup (Pixhawk 6C, FTDI
wiring) needed only once HITL work begins.
