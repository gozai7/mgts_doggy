#!/usr/bin/env bash
set -eu
phase=${1:-mapping}
case "$phase" in
  mapping|localization) viewer=mgts-go2-rviz ;;
  result) viewer=mgts-go2-result; test -f "$PWD/results/${2:?Specify a run name}/result_bag/metadata.yaml" ;;
  *) echo "Use mapping, localization or result" >&2; exit 2 ;;
esac
x11=(-e DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix:ro)
if [[ -n ${XAUTHORITY:-} && -r $XAUTHORITY ]]; then
  x11+=(-e XAUTHORITY=/tmp/rviz.auth -v "$XAUTHORITY:/tmp/rviz.auth:ro")
fi
: "${DISPLAY:?Set DISPLAY to the desktop X11 or XWayland display}"
docker run --rm --name "$viewer" --network none "${x11[@]}" -e QT_X11_NO_MITSHM=1 \
  -v "$PWD/results:/results:ro" -v "$PWD/data/official:/data:ro" -v "$PWD/configs:/mgts/configs:ro" \
  mgts-competition-glim:gpu bash -c '
    if [[ $1 == result ]]; then
      ros2 bag play "/results/$2/result_bag" --qos-profile-overrides-path /mgts/configs/competition/result-play-qos.yaml --loop >/tmp/go2-play.log 2>&1 &
      exec rviz2 -d /mgts/configs/competition/results.rviz
    fi
    ros2 bag play "/data/$1" --loop --topics /utlidar/cloud_deskewed /utlidar/robot_odom >/tmp/go2-play.log 2>&1 &
    exec rviz2 -d /mgts/configs/competition/official-go2.rviz
  ' bash "$phase" "${2:-}"
