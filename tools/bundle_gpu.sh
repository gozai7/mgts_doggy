#!/usr/bin/env bash
set -euo pipefail
out=${RESULTS_DIR:-results}
mkdir -p "$out"
staging=$(mktemp -d "$out/.glim-bundle-XXXXXX")
trap 'rm -rf -- "$staging"' EXIT
kit="$staging/glim-portable"
mkdir "$kit"
install -m 755 docker/competition/portable-run.sh "$kit/run.sh"
cp docs/GPU_PORTABLE.md "$kit/README.md"
sed 's/^  mode: cpu$/  mode: gpu/; s/^  profile: default_indoor$/  profile: gpu/' \
  configs/competition/dataset.yaml > "$kit/dataset.yaml"
image=mgts-competition-glim:gpu
docker image inspect "$image" --format '{{.Id}}' > "$kit/image.id"
docker image save "$image" | zstd -q -T1 -3 > "$kit/image.tar.zst"
archive=MGTS-GLIM-GPU-linux-amd64.tar
tar -cf "$staging/$archive" -C "$staging" glim-portable
mv "$staging/$archive" "$out/$archive"
sha256sum "$out/$archive"
