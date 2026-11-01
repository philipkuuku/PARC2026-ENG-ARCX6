# Voice + Task Manager (Sub-teams 2 and 3)

How a spoken or typed command becomes a Nav2 goal, and what each new file does.
Everything is a small, separate ROS 2 node. They talk only through topics, so any
one of them can be replaced without touching the others.

```
 mic ──► stt_node ──► /speech/transcript ──► terminal_nav ──► Nav2 (NavigateToPose)
                                                  │  ▲
                                          llm_backend (Groq, optional)
                                                  │
 speaker ◄── tts_node ◄── /robot/say ◄────────────┘
                ╰──► /tts/speaking  (mutes the mic while the robot talks)
```

## Files

| File | Owner | What it does |
|---|---|---|
| `terminal_nav.py` | Sub-team 3 | Task manager: text command → validated goal → Nav2. Also a typed terminal. |
| `llm_backend.py` | Sub-team 3 | Optional language model that turns free text into an action (Groq, tool calling). |
| `config/locations.yaml` | Sub-team 3 | Named spots and which tag maps to which pose. |
| `stt_node.py` | Sub-team 2 (reference) | Microphone → Groq Whisper → `/speech/transcript`. |
| `tts_node.py` | Sub-team 2 (reference) | `/robot/say` → Groq text-to-speech → speaker. |

## Topic contract

| Topic | Type | Direction |
|---|---|---|
| `/speech/transcript` | `std_msgs/String` | STT → task manager (one message per utterance) |
| `/robot/say` | `std_msgs/String` | task manager → TTS (text to speak) |
| `/tts/speaking` | `std_msgs/Bool` | TTS → STT (true while speaking) |
| `/task/status` | `std_msgs/String` (JSON) | task manager → anyone (state, target, remaining stops) |

## Sub-team 3: task manager (`terminal_nav`)

**Valid tags:** 1, 4, 7, 9 (organizers: there is no tag 0). Anything else is refused.

**Typed commands** (also work for testing without voice):

| Command | Meaning |
|---|---|
| `/pose X Y [YAW_DEG]` | go to a map position |
| `/spot NAME` | go to a named spot |
| `/tag N` | go to tag N's location |
| `/seq N N N` | visit several tags in order |
| `/stop` | cancel the trip |
| `/where` | position, and which tag it is at or nearest to |
| `/status` | what it is doing now |
| `/tags` | list tags and whether their pose is known |

Plain text such as "please take me to tag seven" goes to the language model, or to
the rule-based parser if the model is unavailable.

**What it does**
- Every command becomes one structured action; only that reaches Nav2.
- The model never outputs coordinates. It picks a tag ID, and the code looks up
  the pose. Unknown or invalid tags are refused, never guessed.
- **"Stop" never waits on the model** (keyword match first).
- Model calls run in a separate thread so a slow reply can't stall Nav2 feedback.
- **Arrival announcements:** "I have arrived at tag 1. Heading to tag 9 next."
  Failures are spoken too.
- **Close enough counts:** if Nav2 fails but the robot is within `arrival_distance`
  (0.5 m) of the goal, the stop counts as reached and the trip continues.
- **Retries:** if it fails while still far away, the same stop is retried up to
  `max_retries` (2) times. If someone else cancels the goal, it does not retry.
- Nav2's own failure reason is printed when this Nav2 version provides one.
- **Offline fallback:** the rule-based parser handles navigation, "stop", and the
  questions "where are you", "which tag", "what are you doing", "where can you go".

**Parameters** (`ros2 run ... --ros-args -p name:=value`)

| Parameter | Default | Meaning |
|---|---|---|
| `locations_file` | package `config/locations.yaml` | spots and tag poses |
| `arrival_distance` | 0.5 | m; a failed goal this close counts as reached |
| `max_retries` | 2 | resends of a stop after a far failure |
| `use_llm` | true | false = rule-based parser only |
| `llm_base_url` | Groq | any OpenAI-compatible server (e.g. Ollama) |
| `llm_model` | `openai/gpt-oss-20b` | check `console.groq.com` if it says "model not found" |
| `llm_timeout` | 8.0 | seconds |
| `llm_reasoning_effort` | (off) | try `low` for speed |

## Sub-team 3: language model (`llm_backend`)

- Calls an OpenAI-compatible `/chat/completions` endpoint with tool calling.
- **Tools:** `navigate_to`, `navigate_sequence`, `cancel_navigation`,
  `list_locations`, `where_am_i`, `report_status`, `ask_clarification`.
  Tag IDs are limited to the registry's tags.
- **Memory:** the last 4 exchanges are sent, so "yes" and "go ahead" make sense.
  If the API rejects the history (HTTP 400), it clears it and retries once.
- One quick retry on a network hiccup (DNS, connection), none on timeouts or HTTP errors.
- The key is read from `GROQ_API_KEY` (or `LLM_API_KEY`). It is never in the code.

## Sub-team 2: reference voice nodes

**`stt_node`** (microphone → text)
- Detects speech by loudness, learns the room's noise level for about 1 s, and
  removes the mic's DC offset (this was why the first test needed shouting).
- Sends each utterance to Groq `whisper-large-v3-turbo`, with a short vocabulary
  hint ("tag one, tag four, …").
- Drops Whisper's phantom text ("thank you", "you") and any echo of its own prompt.
- Optional wake word, several phrases allowed: `-p wake_word:="hey robot,hi robot"`.
  After it, `wake_window` seconds of follow-ups need no wake word.
  **"Stop" never needs the wake word.** A bare wake word makes the robot say "Yes?".
- Mutes the mic while `/tts/speaking` is true and for `mute_tail_s` after.

| Parameter | Default | Meaning |
|---|---|---|
| `wake_word` | (off) | comma-separated phrases |
| `wake_window` | 8.0 | seconds of follow-ups without the wake word |
| `ack_text` | `Yes?` | spoken after a bare wake word ("" = silent) |
| `min_rms` / `noise_factor` | 0.01 / 3.0 | loudness gate: max(`min_rms`, noise × `noise_factor`) |
| `silence_s` | 0.8 | quiet time that ends an utterance (lower = faster) |
| `device` | system default | microphone index |
| `debug_levels` | false | print `peak`, `noise`, `threshold`, `dc` each second |

**`tts_node`** (text → speech)
- Speaks `/robot/say` messages in order (keeps the newest 5), via Groq
  `canopylabs/orpheus-v1-english`, voice `autumn` (`-p voice:=troy` to change).
- Publishes `/tts/speaking` before any sound so the mic is muted in time.
- If the online call fails and `espeak-ng` is installed, it speaks with that.

## Setup

```bash
# 1. audio libraries (use apt, not pip, to avoid a NumPy 2 clash with ROS)
sudo apt install python3-sounddevice python3-numpy libportaudio2 espeak-ng

# 2. copy terminal_nav.py, llm_backend.py, stt_node.py, tts_node.py into the
#    inner package folder; put locations.yaml in config/. In setup.py add:
#      'terminal_nav = arcx6_solution.terminal_nav:main',
#      'stt_node = arcx6_solution.stt_node:main',
#      'tts_node = arcx6_solution.tts_node:main',
cd ~/parc_main/PARC2026-ENG-ARCX6/ros2_ws && colcon build --symlink-install

# 3. in every terminal (no Python venv activated for ROS nodes)
export GROQ_API_KEY=gsk_...
```

Run, one terminal each (localization and Nav2 first):

```bash
ros2 run arcx6_solution tts_node
ros2 run arcx6_solution stt_node --ros-args -p wake_word:="hey robot,hi robot" -p wake_window:=15.0
ros2 run arcx6_solution terminal_nav --ros-args -p use_sim_time:=true
```

Test without a microphone:

```bash
ros2 topic pub --once /speech/transcript std_msgs/msg/String "{data: 'go to tag seven'}"
ros2 topic pub --once /robot/say std_msgs/msg/String "{data: 'Hello, I am ready.'}"
```

## Status

| Piece | Status |
|---|---|
| `terminal_nav` + Groq model, driving in the sim | Working (tested live) |
| Close-enough, retries, announcements, `/status` | Tested with stubs; not yet seen failing live |
| `stt_node` | Logic tested; microphone level and wake word being tuned live |
| `tts_node` | Logic tested with a mock server; not yet confirmed on real speakers |

## Known limits and next steps

- **Tag poses are placeholders** (four spots picked in RViz on the stadium map).
  The arena is a 6 m × 6 m box, so it needs a new map and new test spots.
- **Live tag handling not built yet:** read `tag36h11:<id>` from TF, average it,
  drive to a standoff pose (about 0.8 m) in front of the tag, search when unknown,
  verify on arrival. `Registry.set_tag_pose()` is the hook. Needs Sub-team 1's
  `apriltag_ros` config to use `ids: [1, 4, 7, 9]` and `size: 0.165`.
- **Real robot:** the command-velocity and scan topics, frames and `use_sim_time`
  are sim-specific and must become parameters. The robot's topic list shows
  different names (`/cmd_vel`, several `/scan*`, RealSense cameras).
- **Wake word is text-based** (every utterance goes online first). A real on-device
  wake-word model is better for the Jetson.
- **Needs internet** for the model, STT and TTS. Without it, typed commands and the
  rule-based parser still work, and TTS falls back to `espeak-ng`.
- **Shared robot:** don't leave keys, prompts or tuned parameters on it. Use a
  separate API key, keep files in a temporary folder, and delete them after each
  session.
