#!/bin/bash
# RViz2 は AMENT 上の全 rviz プラグイン .so を列挙して読み込む。
# underlay（例: ~/autoware/install）の tier4_dummy_object_rviz_plugin が
# overlay の tier4_simulation_msgs と ABI 不一致だと symbol lookup error で即終了し、
# respawn 有効時はスプラッシュ画像の点滅になる。当該 prefix だけ除外してから起動する。
set -euo pipefail
_filtered=""
IFS=':' read -ra _parts <<< "${AMENT_PREFIX_PATH:-}"
for _p in "${_parts[@]}"; do
  [[ -z "$_p" ]] && continue
  case "$_p" in
    */tier4_dummy_object_rviz_plugin) continue ;;
    */tier4_state_rviz_plugin) continue ;;
  esac
  _filtered="${_filtered:+${_filtered}:}${_p}"
done
export AMENT_PREFIX_PATH="$_filtered"
exec "$(command -v rviz2)" "$@"
