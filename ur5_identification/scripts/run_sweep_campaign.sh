#!/usr/bin/env bash
# ============================================================================
#  Campaña de barridos de excitación: una corrida por junta (FASE 2).
#
#  Uso:
#    run_sweep_campaign.sh [test_base] [joint_damping] [joint_friction]
#
#  Ejemplos:
#    # Control negativo: planta SIN fricción, verifica coherencia modelo-planta
#    run_sweep_campaign.sh 300 0 0
#    # Con fricción conocida inyectada, para validar el identificador
#    run_sweep_campaign.sh 400 1.5 2.5
#
#  Los CSV salen como ~/.ros/ur5_dyn_control/fl_<test_base+joint>.csv
# ============================================================================
set -u

TEST_BASE="${1:-300}"
DAMPING="${2:-0}"
FRICTION="${3:-0}"
WORLD_NAME="${WORLD:-empty_test_world.sdf}"

PKG="$(ros2 pkg prefix ur5_dyn_control)/share/ur5_dyn_control"
WORLD="$PKG/worlds/$WORLD_NAME"
LOGDIR="${LOGDIR:-/tmp/sweep_campaign}"
mkdir -p "$LOGDIR"

cleanup() {
  pkill -9 -f "ign gazebo"          2>/dev/null
  pkill -9 -f "/bin/sh -c ruby"     2>/dev/null
  pkill -9 -f parameter_bridge      2>/dev/null
  pkill -9 -f robot_state_publisher 2>/dev/null
  pkill -9 -f gz_fl_control_node    2>/dev/null
  sleep 3
}

echo "Campaña: test_base=$TEST_BASE damping=$DAMPING friction=$FRICTION mundo=$WORLD_NAME"
cleanup

for J in ${JOINTS:-0 1 2 3 4 5}; do
  TN=$((TEST_BASE + J))
  LOG="$LOGDIR/sweep_j${J}.log"
  CSV="$HOME/.ros/ur5_dyn_control/fl_${TN}.csv"

  # Reanudable: una corrida ya completa no se repite.
  if [ -s "$CSV" ] && grep -q "Estado -> DONE" "$LOG" 2>/dev/null; then
    echo "[$(date +%H:%M:%S)] junta $J ya completa, se salta"
    continue
  fi

  echo "[$(date +%H:%M:%S)] junta $J -> fl_${TN}.csv"
  rm -f "$CSV"

  # sweep.joint y t_sim se pasan por línea de comandos. OJO con t_sim: el
  # launch lo declara con default "0.0" y ese default SOBRESCRIBE lo que diga el
  # params_file, así que sin pasarlo aquí el nodo se queda en HOLD_END
  # indefinidamente, nunca llega a DONE y cada junta consume el timeout entero.
  timeout 900 ros2 launch ur5_dyn_control fl_control.launch.py \
      gazebo_gui:=false test_num:="$TN" \
      params_file:="$PKG/config/sweep_params.yaml" \
      world:="$WORLD" \
      joint_damping:="$DAMPING" joint_friction:="$FRICTION" \
      sweep_joint:="$J" t_sim:=2.0 > "$LOG" 2>&1 &
  LAUNCH_PID=$!

  # El nodo llega a DONE pero `ros2 launch` no termina solo, así que sin esto
  # cada junta consumiría el timeout entero (15 min en vez de ~7).
  for _ in $(seq 1 900); do
    grep -q "Estado -> DONE" "$LOG" 2>/dev/null && break
    kill -0 "$LAUNCH_PID" 2>/dev/null || break
    sleep 1
  done
  sleep 2                       # que el CSV se cierre y vacíe
  kill -INT "$LAUNCH_PID" 2>/dev/null
  wait "$LAUNCH_PID" 2>/dev/null

  cleanup
  if grep -q "Estado -> DONE" "$LOG"; then
    SZ=$(stat -c%s "$HOME/.ros/ur5_dyn_control/fl_${TN}.csv" 2>/dev/null || echo 0)
    echo "    OK  ($((SZ / 1024 / 1024)) MB)"
  else
    echo "    FALLO — ver $LOG"
  fi
done

echo "[$(date +%H:%M:%S)] campaña terminada. CSVs en ~/.ros/ur5_dyn_control/"
