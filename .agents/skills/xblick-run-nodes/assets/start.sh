#!/usr/bin/env bash
# Start the programs of an experiment plan in one tmux session, one window per program, each with its log.
#
#   bash assets/start.sh plan.tsv [--session NAME] [--logs DIR] [--dry-run]
#
# plan.tsv: one program per line, tab-separated: name, working directory, environment (VAR=value ... or -), command.
#   sam3	~/code/xblick-sam3	CUDA_VISIBLE_DEVICES=0 CUDA_MPS_PIPE_DIRECTORY=/tmp/xblick-mps-gpu0	.venv/bin/xblick-sam3 --camera demo/cam --words cup
# Lines starting with # are skipped. The script prints one JSON line per started window on stdout and the PIDs file.
# Exit codes: 0 started, 2 bad arguments or plan, 3 the tmux session exists already.
set -euo pipefail

usage() { sed -n '2,10p' "$0" | sed 's/^# \{0,1\}//'; }
[ $# -ge 1 ] || { usage >&2; exit 2; }
case "$1" in -h|--help) usage; exit 0 ;; esac
plan=$1; shift
session=exp; logs=""; dry=0
while [ $# -gt 0 ]; do
  case "$1" in
    --session) session=$2; shift 2 ;;
    --logs) logs=$2; shift 2 ;;
    --dry-run) dry=1; shift ;;
    -h|--help) usage; exit 0 ;;
    *) echo "Error: unknown argument '$1'." >&2; exit 2 ;;
  esac
done
[ -f "$plan" ] || { echo "Error: no plan file '$plan'." >&2; exit 2; }
command -v tmux > /dev/null || { echo "Error: tmux is not installed." >&2; exit 2; }
logs=${logs:-$(dirname "$plan")/logs}
if [ $dry = 0 ] && tmux has-session -t "$session" 2>/dev/null; then
  echo "Error: tmux session '$session' exists. Stop it or pass --session." >&2; exit 3
fi
mkdir -p "$logs"
pids="$logs/pids.tsv"
n=0
while IFS=$'\t' read -r name dir env cmd || [ -n "$name" ]; do
  [ -z "$name" ] || [[ "$name" == \#* ]] && continue
  [ -n "$cmd" ] || { echo "Error: plan line '$name' has fewer than four tab-separated fields." >&2; exit 2; }
  dir=${dir/#\~/$HOME}
  [ -d "$dir" ] || { echo "Error: '$name': no directory '$dir'." >&2; exit 2; }
  [ "$env" = "-" ] && env=""
  line="cd '$dir' && echo \$\$ > '$logs/$name.pid' && exec env $env $cmd > '$logs/$name.log' 2>&1"
  printf '{"window": "%s", "dir": "%s", "env": "%s", "command": "%s", "log": "%s"}\n' \
    "$name" "$dir" "$env" "$cmd" "$logs/$name.log"
  [ $dry = 1 ] && continue
  if [ $n = 0 ]; then tmux new-session -d -s "$session" -n "$name" "bash -c \"$line\""
  else tmux new-window -t "$session" -n "$name" "bash -c \"$line\""; fi
  n=$((n + 1))
  sleep 1
done < "$plan"
[ $dry = 1 ] && exit 0
sleep 1
: > "$pids"
for f in "$logs"/*.pid; do [ -f "$f" ] && printf '%s\t%s\n' "$(basename "$f" .pid)" "$(cat "$f")" >> "$pids"; done
echo "started $n windows in tmux session '$session'; PIDs in $pids; stop each with: kill -TERM <pid>" >&2
