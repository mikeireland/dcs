#!/usr/bin/env bash
set -euo pipefail

# ---- Usage ----
if [[ $# -lt 2 ]]; then
  echo "Usage: $0 <mode> <mask> [beam_list]"
  echo "  <mode>      : required"
  echo "  <mask>      : required"
  echo "  [beam_list] : optional comma-separated (e.g. 1,2,4). Default: 1,2,3,4"
  exit 1
fi
MODE="$1"
MASK="$2"
BEAM_INPUT="${3:-1,2,3,4}"

# ---- Parse beam list ----
IFS=',' read -ra BEAMS <<< "$BEAM_INPUT"

# ---- Beam→Port mapping ----
declare -A PORTS=(
  [1]=6662
  [2]=6663
  [3]=6664
  [4]=6665
)

# ---- Pick a terminal emulator ----
open_term() {
  local title="$1"; shift
  local cmd="$*"
  if command -v gnome-terminal >/dev/null 2>&1; then
    gnome-terminal --title="$title" -- bash -lc "$cmd; echo; echo '[$title] finished. Press Enter to close.'; read"
  elif command -v xterm >/dev/null 2>&1; then
    xterm -T "$title" -e bash -lc "$cmd; echo; echo '[$title] finished. Press Enter to close.'; read"
  else
    echo "Error: need gnome-terminal or xterm installed." >&2
    exit 1
  fi
}

# ---- Launch beams ----
for beam in "${BEAMS[@]}"; do
  if [[ -z "${PORTS[$beam]:-}" ]]; then
    echo "Warning: beam $beam not valid (must be 1–4). Skipping."
    continue
  fi
  port="${PORTS[$beam]}"
  socket="tcp://*:${port}"
  title="Baldr Beam ${beam}"
  open_term "$title" "./baldr --beam ${beam} --mode ${MODE} --mask ${MASK} --socket ${socket}" &
  sleep 0.15
done

# ---- Arrange windows in 2×2 grid (optional) ----
if command -v wmctrl >/dev/null 2>&1 && command -v xdotool >/dev/null 2>&1; then
  if command -v xdpyinfo >/dev/null 2>&1; then
    read -r SW SH < <(xdpyinfo | awk '/dimensions:/{split($2,a,"x"); print a[1], a[2]}')
  elif command -v xrandr >/dev/null 2>&1; then
    read -r SW SH < <(xrandr | awk '/\*/{print $1; exit}' | awk -Fx '{print $1, $2}')
  else
    SW=1920; SH=1080
  fi

  M=8
  (( W = SW/2 - 2*M ))
  (( H = SH/2 - 2*M ))
  Xs=($M $((W+3*M)) $M $((W+3*M)) )
  Ys=($M $M $((H+3*M)) $((H+3*M)) )

  idx=0
  for beam in "${BEAMS[@]}"; do
    title="Baldr Beam ${beam}"
    win_id="$(xdotool search --name "$title" | tail -n1 || true)"
    [[ -n "$win_id" ]] || continue
    wmctrl -ir "$win_id" -e "0,${Xs[$idx]},${Ys[$idx]},$W,$H" || true
    ((idx++))
  done
else
  echo "Tip: install 'wmctrl' and 'xdotool' to auto-tile the terminals."
fi