#!/usr/bin/env bash
set -euo pipefail
cd -- "$(dirname -- "$0")"
image=$(cat image.id)
if ! docker image inspect "$image" >/dev/null 2>&1; then
  docker image load --input image.tar.zst
fi
gpu=(--gpus all)
if docker info --format '{{json .DiscoveredDevices}}' 2>/dev/null | grep -q 'nvidia.com/gpu=all'; then
  gpu=(--device nvidia.com/gpu=all)
fi
mkdir -p data results
case "${1:-run-all}" in
  gpu-check) cmd=(python3 -c 'import ctypes as c; d=c.CDLL("libcuda.so.1"); assert d.cuInit(0)==0; n=c.c_int(); assert d.cuDeviceGetCount(c.byref(n))==0 and n.value; print("CUDA devices:", n.value)');;
  smoke-test) cmd=(bash tools/ros_smoke.sh);;
  go2-live)
    cmd=(bash -ec '
      cfg=/mgts/configs/competition/official-go2.yaml
      python3 -m competition.runner mapping --config "$cfg"
      python3 -m competition.runner route-odometry --config "$cfg"
      python3 -m tools.go2_track "/results/$1" --glim --config "$cfg"
    ' bash "${2:-go2-live-$(date +%Y%m%d-%H%M%S)}");;
  *) cmd=(python3 -m competition.runner "${1:-run-all}" --config /mgts/configs/competition/dataset.yaml "${@:2}");;
esac
exec docker run --rm --init --pull=never --network none --shm-size 1g "${gpu[@]}" \
  -e OPENBLAS_NUM_THREADS=1 -e ROS_LOCALHOST_ONLY=1 -e GLIM_SMOKE_MODE=gpu -e NVIDIA_DRIVER_CAPABILITIES=compute,utility \
  -v "$PWD/data:/data:ro" -v "$PWD/results:/results" \
  -v "$PWD/dataset.yaml:/mgts/configs/competition/dataset.yaml:ro" "$image" "${cmd[@]}"
