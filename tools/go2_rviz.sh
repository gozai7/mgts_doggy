#!/usr/bin/env bash
set -eu
phase=${1:-mapping}
case "$phase" in
  mapping|localization) viewer=mgts-go2-rviz ;;
  result) viewer=mgts-go2-result; test -f "$PWD/results/${2:?Specify a run name}/result_bag/metadata.yaml" ;;
  glim)
    run=${2:?Specify a run name}
    viewer=${MGTS_GO2_VIEWER:-mgts-glim-$(basename "$run")}
    if [[ $(docker inspect --format '{{.State.Running}}' "$viewer" 2>/dev/null || true) == true ]]; then
      echo "RViz is already showing $run"
      exit 0
    fi
    for file in map.pcd mapping.tum; do
      [[ -s "$PWD/results/$run/$file" ]] || { echo "Missing results/$run/$file" >&2; exit 2; }
    done
    [[ -s "$PWD/results/$run/full_route.tum" || -s "$PWD/results/$run/glim_b_in_map.tum" || -s "$PWD/results/$run/trajectory.tum" ]] || { echo "Missing route B" >&2; exit 2; } ;;
  *) echo "Use mapping, localization, result or glim" >&2; exit 2 ;;
esac
x11=(-e DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix:ro)
if [[ -n ${XAUTHORITY:-} && -r $XAUTHORITY ]]; then
  x11+=(-e XAUTHORITY=/tmp/rviz.auth -v "$XAUTHORITY:/tmp/rviz.auth:ro")
fi
: "${DISPLAY:?Set DISPLAY to the desktop X11 or XWayland display}"
options=()
if [[ $phase == glim ]]; then
  options+=(--detach)
  if [[ -d /dev/dri ]]; then options+=(--device /dev/dri); fi
fi
docker run "${options[@]}" --rm --name "$viewer" --network none "${x11[@]}" -e QT_X11_NO_MITSHM=1 \
  -v "$PWD/results:/results:ro" -v "$PWD/data/official:/data:ro" -v "$PWD/configs:/mgts/configs:ro" \
  -v "$PWD/tools/view_results.py:/mgts/tools/view_results.py:ro" \
  mgts-competition-glim:gpu bash -c '
    if [[ $1 == glim ]]; then
      python3 /mgts/tools/view_results.py --flat "/results/$2" &
      layout=glim-routes
      if [[ -s "/results/$2/full_route.tum" ]]; then layout=glim-full-route; fi
      cp "/mgts/configs/competition/$layout.rviz" "/tmp/$(basename "$2").rviz"
      rviz2 -d "/tmp/$(basename "$2").rviz" &
      wait -n
      exit $?
    fi
    if [[ $1 == result ]]; then
      ros2 bag play "/results/$2/result_bag" --qos-profile-overrides-path /mgts/configs/competition/result-play-qos.yaml --loop >/tmp/go2-play.log 2>&1 &
      layout=results
      if [[ -s "/results/$2/full_route.tum" ]]; then layout=glim-full-route; fi
      exec rviz2 -d "/mgts/configs/competition/$layout.rviz"
    fi
    ros2 bag play "/data/$1" --loop --topics /utlidar/cloud_deskewed /utlidar/robot_odom >/tmp/go2-play.log 2>&1 &
    exec rviz2 -d /mgts/configs/competition/official-go2.rviz
  ' bash "$phase" "${2:-}"
