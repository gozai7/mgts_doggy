#!/usr/bin/env bash
set -euo pipefail

root=${1:-data/official}
mkdir -p "$root"
tmp=$(mktemp -d "$root/.go2-XXXXXX")
trap 'rm -rf "$tmp"' EXIT
revision=ecbeeb681f7fe1e274402623da27444f63a4c61a
for phase in mapping localization; do
  if [[ -s "$root/$phase/metadata.yaml" ]] && compgen -G "$root/$phase/*.mcap" >/dev/null; then
    continue
  fi
  [[ ! -e "$root/$phase" || -z "$(ls -A "$root/$phase")" ]] || {
    echo "Incomplete bag in $root/$phase (expected metadata.yaml and MCAP)" >&2; exit 1;
  }
  name=bag_for_$phase
  zip="$root/$name.zip"
  if [[ ! -f "$zip" ]]; then
    zip="$tmp/$name.zip"
    # The raw GitHub URL contains only a Git LFS pointer.
    curl -fL --retry 2 -o "$zip" "https://media.githubusercontent.com/media/ivr0ma/hackathon/$revision/hackathon/ros2_ws/$name.zip"
  fi
  unzip -q "$zip" "$name/metadata.yaml" "$name/*.mcap" -d "$tmp"
  mv -T "$tmp/$name" "$root/$phase"
done
