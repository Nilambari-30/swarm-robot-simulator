#!/usr/bin/env bash
# =========================================================
# build.sh  —  Compile swarm_math.c into swarm_math.so
# Usage:  bash build.sh
# =========================================================
set -e
echo "=================================================="
echo "  Building swarm_math.c  →  swarm_math.so"
echo "=================================================="
gcc -O2 -shared -fPIC -o swarm_math.so swarm_math.c -lm
echo ""
echo "✓  Build successful!  swarm_math.so is ready."
echo ""
echo "Next steps:"
echo "  pip install pygame"
echo "  python swarm_simulator.py"
echo "=================================================="
