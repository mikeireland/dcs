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

# ---- Quick environment sanity checks ----
if [[ -z "${DISPLAY:-}" ]]; then
  echo "Error: No DISPLAY found. You must run this inside a GUI session (not a headless SSH shell)."
  echo "Tip: If you are on SSH, try:  ssh -X <host>   (and ensure X11 apps work) "
  exit 1
fi

# Wayland note: opening terminals is fine; tiling tools may not work on Wayland (you commented them out already)

# ---- Parse beam list ----
IFS=',' read -ra BEAMS <<< "$BEAM_INPUT"

# ---- Beam→Port mapping ----
declare -A PORTS=(
  [1]=6662
  [2]=6663
  [3]=6664
  [4]=6665
)

# ---- Terminal launcher abstraction ----
# Each launcher must run a bash -lc "<cmd>; read" so the window stays open when the command ends.
launch_with() {
  local term="$1"; shift
  local title="$1"; shift
  local cmd="$*"

  case "$term" in
    gnome-terminal)
      gnome-terminal --title="$title" -- bash -lc "$cmd; echo; echo '[$title] finished. Press Enter to close.'; read"
      ;;
    kgx) # GNOME Console
      kgx --title="$title" -- bash -lc "$cmd; echo; echo '[$title] finished. Press Enter to close.'; read"
      ;;
    xfce4-terminal)
      xfce4-terminal --title "$title" -e "bash -lc '$cmd; echo; echo \"[$title] finished. Press Enter to close.\"; read'"
      ;;
    mate-terminal)
      mate-terminal --title="$title" -- bash -lc "$cmd; echo; echo '[$title] finished. Press Enter to close.'; read"
      ;;
    konsole)
      konsole --new-tab --hold -p tabtitle="$title" -e bash -lc "$cmd; echo; echo '[$title] finished. Press Enter to close.'; read"
      ;;
    tilix)
      tilix -t "$title" -e bash -lc "$cmd; echo; echo '[$title] finished. Press Enter to close.'; read"
      ;;
    xterm)
      xterm -T "$title" -e bash -lc "$cmd; echo; echo '[$title] finished. Press Enter to close.'; read"
      ;;
    alacritty)
      alacritty --title "$title" -e bash -lc "$cmd; echo; echo '[$title] finished. Press Enter to close.'; read"
      ;;
    kitty)
      kitty --title "$title" bash -lc "$cmd; echo; echo '[$title] finished. Press Enter to close.'; read"
      ;;
    terminator)
      terminator -T "$title" -x bash -lc "$cmd; echo; echo '[$title] finished. Press Enter to close.'; read"
      ;;
    *)
      return 1
      ;;
  esac
}

detect_terminal() {
  for t in gnome-terminal kgx xfce4-terminal mate-terminal konsole tilix xterm alacritty kitty terminator; do
    if command -v "$t" >/dev/null 2>&1; then
      echo "$t"
      return 0
    fi
  done
  return 1
}

TERM_CMD="$(detect_terminal || true)"
if [[ -z "${TERM_CMD:-}" ]]; then
  echo "Error: no supported terminal emulator found."
  echo "Install one, e.g.: sudo apt install gnome-terminal  (or xterm, xfce4-terminal, etc.)"
  exit 1
fi

echo "Using terminal: $TERM_CMD"
echo "Mode: $MODE | Mask: $MASK | Beams: ${BEAMS[*]}"

# ---- Launch beams ----
for beam in "${BEAMS[@]}"; do
  if [[ -z "${PORTS[$beam]:-}" ]]; then
    echo "Warning: beam $beam not valid (must be 1–4). Skipping."
    continue
  fi
  port="${PORTS[$beam]}"
  socket="tcp://*:${port}"
  title="Baldr Beam ${beam}"

  cmd="./baldr --beam ${beam} --mode ${MODE} --mask ${MASK} --socket ${socket}"
  echo "Launching: $title  ->  $cmd"

  # Launch in background so we don’t block
  launch_with "$TERM_CMD" "$title" "$cmd" &
  # tiny delay helps the WM create/focus windows with proper titles
  sleep 0.15
done

wait || true