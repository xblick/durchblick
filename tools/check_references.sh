#!/usr/bin/env bash
# Fails when a tracked text file names an internal system, host or path: a robot framework, the lab's robots, its
# servers or its file systems. The allowlist (tools/reference_allowlist.txt) names the hits that may stay. This script
# and the allowlist contain the pattern and are not searched. Usage: tools/check_references.sh   (from any directory)
# Exit 0: no hit outside the allowlist; 1: the hits are printed as path:line:text.
set -euo pipefail
cd "$(dirname "$0")/.."
pattern='armarx|armar[67]|axii|h2t|kit\.edu|/cvhci|/lsdf|/var/tmp|armar-user|visionx|armem'
allowlist=tools/reference_allowlist.txt
[[ -f "$allowlist" ]] || { echo "missing $allowlist" >&2; exit 1; }

# allowlist: "<path>" allows every hit in that file, "<path>:<text>" the hits on lines that contain <text>
declare -A whole=()
declare -a partial_path=() partial_text=()
while IFS= read -r entry || [[ -n "$entry" ]]; do
  [[ -z "$entry" || "$entry" == \#* ]] && continue
  if [[ "$entry" == *:* ]]; then
    partial_path+=("${entry%%:*}")
    partial_text+=("${entry#*:}")
  else
    whole["$entry"]=1
  fi
done < "$allowlist"

status=0
while IFS= read -r hit; do
  path="${hit%%:*}"
  [[ -n "${whole[$path]:-}" ]] && continue
  rest="${hit#*:}"
  text="${rest#*:}"
  allowed=0
  for i in "${!partial_path[@]}"; do
    if [[ "${partial_path[$i]}" == "$path" && "$text" == *"${partial_text[$i]}"* ]]; then
      allowed=1
      break
    fi
  done
  if [[ $allowed == 0 ]]; then
    echo "$hit"
    status=1
  fi
done < <(git ls-files -z -- . ':!:tools/check_references.sh' ":!:$allowlist" | xargs -0 grep -d skip -nIiE "$pattern" -- || true)

if [[ $status != 0 ]]; then
  echo "internal references found (allowed ones go into $allowlist)" >&2
fi
exit $status
