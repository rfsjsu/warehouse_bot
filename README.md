# warehouse_bot — Voice-Controlled Robot in Gazebo (ROS 2 Jazzy)

This repo runs a warehouse robot in **Gazebo Sim (gz sim)** with visualization in **RViz2**.  
It also includes a **voice control pipeline**:

- **faster-whisper (local STT)**: microphone audio → text
- **Claude (LLM)**: text → structured robot command (JSON)
- **ROS 2**: publish `geometry_msgs/Twist` to `/cmd_vel`
- **ros_gz_bridge**: `/cmd_vel` bridged into Gazebo → robot moves

> Tested on Ubuntu 24.04 / Linux Mint 22.x base, ROS 2 Jazzy.

---

## Repo Structure (important)

This single git repo contains multiple ROS packages:

warehouse_bot/ (git repo root)
├── launch/
│ └── complete.launch.py (starts Gazebo + RViz + bridges + voice)
├── warehouse_voice/ (ROS package: voice pipeline)
│ ├── package.xml
│ ├── setup.py
│ └── warehouse_voice/
│ └── voice_whisper_claude_cmdvel.py
└── ... (other warehouse_bot files)

yaml
Copy code

> NOTE: `warehouse_voice` is inside this repo, but in a typical ROS workspace we also expose it under `~/ros2_ws/src/warehouse_voice` via a symlink for easier `colcon` discovery.

---

## Prerequisites

### System packages
```bash
sudo apt update
sudo apt install -y \
  python3-venv python3-full python3-pip \
  portaudio19-dev \
  wmctrl
ROS 2 Jazzy installed
Assumes ROS 2 Jazzy is installed at:

/opt/ros/jazzy/setup.bash

1) Create a ROS 2 workspace and clone this repo
bash
Copy code
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src

# clone your branch (example)
git clone <YOUR_REPO_URL> warehouse_bot
cd warehouse_bot
git checkout <YOUR_BRANCH_NAME>
2) Create Python virtualenv (required)
We use a venv because Ubuntu 24.04 often blocks system-wide pip installs (PEP 668).

bash
Copy code
cd ~/ros2_ws
python3 -m venv .venv
source ~/ros2_ws/.venv/bin/activate
python -m pip install -U pip
Install dependencies:

bash
Copy code
pip install sounddevice numpy faster-whisper anthropic
(Optional: if you see warnings about jinja2/typeguard)

bash
Copy code
pip install jinja2 typeguard
Quick sanity check:

bash
Copy code
python -c "import sounddevice, numpy; print('OK:', sounddevice.__version__)"
3) Expose warehouse_voice to colcon (one-time symlink)
Because warehouse_voice is nested inside the repo, some setups won’t discover it reliably unless it appears under ~/ros2_ws/src/.

Create a symlink once:

bash
Copy code
rm -rf ~/ros2_ws/src/warehouse_voice
ln -s ~/ros2_ws/src/warehouse_bot/warehouse_voice ~/ros2_ws/src/warehouse_voice
Verify:

bash
Copy code
ls -la ~/ros2_ws/src | grep warehouse_voice
Expected output includes something like:

bash
Copy code
warehouse_voice -> /home/<user>/ros2_ws/src/warehouse_bot/warehouse_voice
4) Build the workspace
bash
Copy code
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash

rm -rf build install log
colcon build --symlink-install
5) Set your Claude API key (required for voice parsing)
In the terminal you will launch from:

bash
Copy code
export ANTHROPIC_API_KEY="YOUR_KEY_HERE"
Tip (optional): persist it in ~/.bashrc:

bash
Copy code
echo 'export ANTHROPIC_API_KEY="YOUR_KEY_HERE"' >> ~/.bashrc
source ~/.bashrc
6) Run (single command)
Open a fresh terminal:

bash
Copy code
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash

# key must be in env
export ANTHROPIC_API_KEY="YOUR_KEY_HERE"

ros2 launch warehouse_bot complete.launch.py
Expected:

Gazebo server + GUI opens

RViz opens

Robot is visible in RViz

7) Voice commands (demonstration)
Speak clearly into the mic:

robot go forward

robot turn left

robot turn right

stop

Why the “robot” wake word?
To avoid accidental commands from background audio (videos, conversations, noise).
The voice node ignores most speech unless it includes the wake word.

8) Quick checks
A) Is /cmd_vel being published?
Open a second terminal:

bash
Copy code
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 topic echo /cmd_vel
When you say robot go forward, you should see linear.x become positive briefly.

B) Ensure the voice node is running only once
bash
Copy code
ros2 node list | grep voice
Expected: only one line:

bash
Copy code
/voice_whisper_claude_cmdvel
If you see duplicates, stop and clean up:

bash
Copy code
pkill -f voice_whisper_claude_cmdvel
Then run the launch again.

9) Troubleshooting
Problem: Robot moves then immediately stops (without saying “stop”)
Common causes:

Voice node running twice (duplicate publishers).

Whisper picks up noise and triggers a “none/stop” command.

Fixes:

Confirm only one voice node:

bash
Copy code
ros2 node list | grep voice
Reduce noise and speak closer to the mic.

Problem: No Gazebo GUI window
Sometimes it opens behind other windows:

bash
Copy code
wmctrl -l | grep -i gz
wmctrl -a Gazebo
Notes / Design Decisions
We intentionally use local STT (faster-whisper) to keep STT cost-free and offline-capable.

Claude is used only for intent parsing (text → command JSON).

/cmd_vel is the standard ROS velocity interface, bridged into Gazebo via ros_gz_bridge.

Contact
If anything breaks, paste:

last ~30 lines of the launch terminal log

output of:

bash
Copy code
ros2 node list
ros2 topic info /cmd_vel -v
