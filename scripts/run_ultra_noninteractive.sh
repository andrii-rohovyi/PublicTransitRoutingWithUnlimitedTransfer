#!/usr/bin/env bash
# Usage: ./run_ultra_noninteractive.sh "<command>" <logfile> [timeout_seconds]
# Example: ./run_ultra_noninteractive.sh "runTDDijkstraQueries Networks/London/Walking/Full/intermediate.binary 5" logs/td_london.log 300

set -euo pipefail

COMMAND="$1"
LOGFILE="$2"
TIMEOUT_SECONDS=${3:-0}

mkdir -p "$(dirname "$LOGFILE")"

# Build the heredoc input for ULTRA non-interactive mode
# We feed the command followed by 'exit' to ensure ULTRA quits after executing.

if [ "$TIMEOUT_SECONDS" -gt 0 ]; then
  TIMEOUT_CMD=(timeout "$TIMEOUT_SECONDS")
else
  TIMEOUT_CMD=()
fi

# Run ULTRA with the command as stdin. Filter out noisy terminal control messages and unknown-key outputs.
# We keep buffering unbuffered with stdbuf -oL to stream output to log.

{
  echo "$COMMAND"
  echo "exit"
} | "${TIMEOUT_CMD[@]}" ./cmake-build-release/ULTRA 2>&1 | stdbuf -oL sed -u '/tcsetattr()/d; /tcsetattr ICANON/d; /~ICANON/d; /Unknown key/d' > "$LOGFILE" &

PID=$!

echo "Started ULTRA (PID=$PID). Logging to $LOGFILE"

echo "PID=$PID" > "$LOGFILE.pid"

# Optionally tail the logfile
# tail -f "$LOGFILE" &
