#!/bin/bash
# Compare two mappers on the archived baseline keyframe times.
set -eu
scene=$1
case "$scene" in
  office) profile="" ;;
  corridor|colonnade|eight|atrium|atrium-opaque) profile="--profile /mgts/configs/planar.yaml" ;;
  *) echo "Unknown scene: $scene" >&2; exit 2 ;;
esac
mkdir -p runs
out=$(mktemp -d "runs/compare-$scene.XXXXXX")
out=${out#runs/}
cp "benchmarks/directional/keyframes/$scene.csv.gz" "runs/$out/keyframes.csv.gz"
for variant in baseline directional; do
    flag=""
    if [ "$variant" = directional ]; then flag=--directional-odom; fi
    make map A="$scene-A" MAP="$out/$variant" \
        ARGS="$profile --keyframes /runs/$out/keyframes.csv.gz --rate 12 $flag"
    make test B="$scene-B" MAP="$out/$variant" ROUTE="$out/$variant-B" ARGS="$profile --rate 3 --refine"
done
printf 'Reports: runs/%s/{baseline,directional}-B/report.json\n' "$out"
