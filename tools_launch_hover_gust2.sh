#!/usr/bin/env bash
# Mk-7 hover gust runner v2 — STRICT PID discipline: no pattern kills; every child PID recorded to the
# ledger file $OUT/$LABEL.pids; cleanup kills ONLY those PIDs. Stage-by-stage aliveness checks.
# usage: launch_hover_gust2.sh LABEL [DURATION_S]  (env: FWDSC/FWDEN/TILT/VELMAX/SLEW/PITCHMIN/XY*/RAMP/WCSV)
set -uo pipefail
ROOT=/home/tasukunakai/PX4-mk7-tune
OUT=/home/tasukunakai/windforce
CFG=/home/tasukunakai/.gz/sim/server.config
LABEL="${1:-hg2}"; DUR="${2:-90}"
PIDS="$OUT/$LABEL.pids"; : > "$PIDS"
mkdir -p "$OUT"; cd "$ROOT"
note(){ echo "[runner] $*"; }
# [0] refuse to start if the environment is not clean (NO pattern kill — report instead)
if pgrep -x px4 >/dev/null 2>&1 || pgrep -f "gz sim" >/dev/null 2>&1; then
  note "ENV NOT CLEAN (px4/gz already running) — aborting, no kills attempted"; exit 2
fi
rm -f /tmp/mk7_wind_go /tmp/mk7_wind_stop /tmp/mk7_wind_now
rm -f build/px4_sitl_default/rootfs/parameters*.bson
# [1] SITL (gz server + px4) — record the pipeline PID
FDRAG_DIR=$ROOT/Tools/simulation/gz/plugins/flaperon_drag/build
sleep 100000 | GZ_SIM_SYSTEM_PLUGIN_PATH="$FDRAG_DIR:${GZ_SIM_SYSTEM_PLUGIN_PATH:-}" GZ_SIM_SERVER_CONFIG_PATH="$CFG" PX4_GZ_WORLD=teTra_wind_choshi PX4_GZ_MODEL_POSE="0,0,0,0,0,-0.506" HEADLESS=1 make px4_sitl gz_teTra_mk-7_EM2 > "$OUT/$LABEL.sitl.log" 2>&1 &
SITL_PID=$!; echo "SITL_PID=$SITL_PID" >> "$PIDS"; note "stage1 SITL launched pid=$SITL_PID"
# [2] wait for the gz server to answer (topic list), up to 90 s
GZOK=0
for i in $(seq 1 30); do
  sleep 3
  if gz topic -l 2>/dev/null | grep -q "/world/teTra_wind_choshi/wind"; then GZOK=1; break; fi
  kill -0 "$SITL_PID" 2>/dev/null || { note "stage2 FAIL: SITL pipeline died"; tail -5 "$OUT/$LABEL.sitl.log"; exit 3; }
done
[ "$GZOK" = 1 ] || { note "stage2 FAIL: gz wind topic never appeared"; tail -5 "$OUT/$LABEL.sitl.log"; exit 3; }
note "stage2 gz server UP (wind topic present, t=$((i*3))s)"
# [3] hover script
timeout $((DUR+260)) python3 /home/tasukunakai/PX4-Autopilot/tools_hover_gust_test.py --connect udpin:0.0.0.0:14550 --duration "$DUR" > "$OUT/$LABEL.test.log" 2>&1 &
TP=$!; echo "HOVER_PID=$TP" >> "$PIDS"; note "stage3 hover script pid=$TP"
# [4] wind publisher
timeout $((DUR+280)) python3 /home/tasukunakai/PX4-Autopilot/tools_wind_pub.py --world teTra_wind_choshi --ramp "${RAMP:-20}" --csv "${WCSV:-/home/tasukunakai/mk7_choshi_wind.csv}" --umean "${UMEAN:-30}" > "$OUT/$LABEL.wind.log" 2>&1 &
WP=$!; echo "WIND_PID=$WP" >> "$PIDS"; note "stage4 wind publisher pid=$WP"
# [5] wait for the hover script; then PID-exact cleanup of OUR children only
wait "$TP" 2>/dev/null || true
note "stage5 hover done; killing recorded PIDs only"
for p in "$WP" "$SITL_PID"; do kill -TERM "$p" 2>/dev/null; done
sleep 3
for p in "$WP" "$SITL_PID"; do kill -KILL "$p" 2>/dev/null; done
# the make pipeline's px4/gz children share OUR process group (runner runs under setsid via mk7_launch);
# the outer ledger kill (-PGID) finishes anything left — no pattern kill anywhere.
ulg=$(find "$ROOT/build/px4_sitl_default/rootfs/log" -name "*.ulg" -printf "%T@ %p\n" 2>/dev/null | sort -n | tail -1 | cut -d" " -f2-)
echo "ULG=$ulg"
grep -E "RESULT|DONE|FAILED|ARMED|reached|HOLD" "$OUT/$LABEL.test.log" | tail -12
echo "--- wind log tail ---"; tail -4 "$OUT/$LABEL.wind.log"
echo "DONE_HOVERGUST2"
