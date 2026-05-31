==============================================================================
  SWARM ROBOT MOVEMENT SIMULATOR
  Python + C ctypes Implementation
==============================================================================

PROJECT KEY POINTS COVERED
─────────────────────────────────────────────────────────────────────────────
  ✓  Boids algorithm (flocking) in Python — 30+ robots
  ✓  Pygame real-time visualisation — cohesion, alignment, separation
  ✓  Obstacle avoidance layer
  ✓  Core vector math in C, called via Python ctypes
  ✓  1-page report: swarm algorithms in drone coordination  (report.html)


PROJECT FILES
─────────────────────────────────────────────────────────────────────────────
  swarm_math.c        ← C library: vector math + boids physics
  swarm_simulator.py  ← Python: Pygame window + ctypes bridge
  build.sh            ← Build script (Linux / Mac)
  build.bat           ← Build script (Windows with MinGW)
  report.html         ← 1-page written report (open in any browser)
  README.txt          ← This file


HOW TO RUN  (3 steps)
─────────────────────────────────────────────────────────────────────────────

  STEP 1 — Install Python & Pygame
    Download Python 3.10+ from https://www.python.org/downloads/
    Then open a terminal / command prompt and run:
      pip install pygame

  STEP 2 — Build the C library

    Linux or Mac (Terminal):
      bash build.sh

    Windows (Command Prompt — needs MinGW gcc):
      build.bat
      [Download MinGW from https://www.mingw-w64.org/ if you don't have gcc]

  STEP 3 — Run the simulator
      python swarm_simulator.py


CONTROLS
─────────────────────────────────────────────────────────────────────────────
  P           Play / Pause simulation
  R           Reset (new random robots & obstacles)
  T           Toggle trails on/off
  +  or  =    Increase speed
  -           Decrease speed
  Mouse click Add an obstacle at the clicked position


HOW IT WORKS
─────────────────────────────────────────────────────────────────────────────
  BOIDS ALGORITHM (implemented in C, called from Python):
  ┌─────────────────┬────────────────────────────────────────────────────┐
  │ Separation      │ Each robot steers away from too-close neighbours   │
  │ Alignment       │ Each robot matches the heading of nearby robots    │
  │ Cohesion        │ Each robot steers toward the centre of its group   │
  │ Obstacle avoid  │ Soft + hard repulsive forces from obstacles        │
  └─────────────────┴────────────────────────────────────────────────────┘

  CTYPES BRIDGE:
    Python defines CRobot, CObs, CParams structs that match swarm_math.c.
    Every frame, Python builds ctypes arrays and passes them to:
      boids_step()     — full O(n²) physics loop (runs in C)
      compute_stats()  — avg speed + cluster count (runs in C)
    Python only handles rendering (Pygame) and user input.

  VECTOR MATH IN C (swarm_math.c Section 1):
    vec2_len        — Euclidean length
    vec2_dist       — distance between two points
    vec2_normalize  — normalise vector in-place
    vec2_limit      — clamp magnitude
    vec2_set_len    — set magnitude to exact value
    vec2_dot        — dot product
    vec2_angle      — atan2 wrapper


REPORT
─────────────────────────────────────────────────────────────────────────────
  Open report.html in any web browser to read the 1-page report.
  To save as PDF: File → Print → Save as PDF.

  Topics covered in the report:
    • What is swarm intelligence
    • The three Boids rules explained
    • How swarm algorithms apply to real drone fleets
    • Six real-world use cases (search & rescue, agriculture, military, etc.)
    • What this project implements
    • Challenges and limitations
    • References


REQUIREMENTS SUMMARY
─────────────────────────────────────────────────────────────────────────────
  Python 3.10+
  pygame         (pip install pygame)
  gcc            (built-in on Linux/Mac; MinGW on Windows)

==============================================================================
