#!/bin/bash
# Robot Monitor v2 — comprehensive navigation analysis

WS="/home/natdanai/ROS_Source_Code/yahboomcar_ws/function_package_without_WiFi_module/yahboomcar_ws"
LOG_DIR="/home/natdanai/ROS_Source_Code/robot_logs"
mkdir -p "$LOG_DIR"

source /opt/ros/humble/setup.bash
source "$WS/install/setup.bash"
export ROS_DOMAIN_ID=18

TIMESTAMP=$(date +"%Y%m%d_%H%M%S")
LOG="$LOG_DIR/run_$TIMESTAMP"
mkdir -p "$LOG"

echo "=== Robot Monitor v2 Started: $TIMESTAMP ===" | tee "$LOG/summary.txt"
echo "Logs saved to: $LOG"
echo ""

# ─── Startup: ตรวจ odom topics และ EKF ──────────────────────────────────────
echo "--- ODOM TOPIC CHECK ---"
ODOM_TOPICS=$(ros2 topic list 2>/dev/null | grep -E "odom|filtered")
if [ -z "$ODOM_TOPICS" ]; then
    echo "⚠️  ไม่พบ odom topic เลย — ROS2 อาจยังไม่พร้อม"
else
    echo "$ODOM_TOPICS"
    if echo "$ODOM_TOPICS" | grep -q "/odometry/filtered"; then
        echo "✅  /odometry/filtered พบแล้ว — EKF กำลังทำงาน"
    else
        echo "⚠️  ไม่พบ /odometry/filtered — EKF ไม่ได้รัน!"
        echo "    topic odom ที่มีจริง: $ODOM_TOPICS"
        echo "    แก้ไข: ตรวจสอบ robot_localization node และ bt_navigator odom_topic"
    fi
fi
echo "$ODOM_TOPICS" > "$LOG/odom_topics.txt"
echo "---"
echo ""

# ─── Cleanup on exit ────────────────────────────────────────────────────────
trap 'echo ""; echo "=== Monitor stopped ==="; echo ""; \
      echo "SESSION SUMMARY:" | tee -a "$LOG/summary.txt"; \
      echo "  Oscillation sign-flips : $OSC_COUNT" | tee -a "$LOG/summary.txt"; \
      echo "  Recovery events        : $RECOVERY_COUNT" | tee -a "$LOG/summary.txt"; \
      echo "  Duration               : $LOOP_COUNT loops x 3s" | tee -a "$LOG/summary.txt"; \
      kill $(jobs -p) 2>/dev/null; exit 0' INT TERM

# ─── Background: topic loggers ───────────────────────────────────────────────
(source /opt/ros/humble/setup.bash; source "$WS/install/setup.bash"; export ROS_DOMAIN_ID=18
 ros2 topic echo /cmd_vel --no-arr 2>/dev/null) >> "$LOG/cmd_vel.txt" &

(source /opt/ros/humble/setup.bash; source "$WS/install/setup.bash"; export ROS_DOMAIN_ID=18
 ros2 topic echo /cmd_vel_nav --no-arr 2>/dev/null) >> "$LOG/cmd_vel_nav.txt" &

(source /opt/ros/humble/setup.bash; source "$WS/install/setup.bash"; export ROS_DOMAIN_ID=18
 ros2 topic echo /odometry/filtered --no-arr 2>/dev/null) >> "$LOG/odom_filtered.txt" &

(source /opt/ros/humble/setup.bash; source "$WS/install/setup.bash"; export ROS_DOMAIN_ID=18
 ros2 topic echo /amcl_pose --no-arr 2>/dev/null) >> "$LOG/amcl_pose.txt" &

(source /opt/ros/humble/setup.bash; source "$WS/install/setup.bash"; export ROS_DOMAIN_ID=18
 ros2 topic echo /odom_raw --no-arr 2>/dev/null) >> "$LOG/odom_raw.txt" &

# Capture Nav2 warnings/errors from rosout
(source /opt/ros/humble/setup.bash; source "$WS/install/setup.bash"; export ROS_DOMAIN_ID=18
 ros2 topic echo /rosout 2>/dev/null | grep -E "\[(WARN|ERROR)\]") >> "$LOG/rosout_warnings.txt" &

# ─── Background: periodic Hz checks ─────────────────────────────────────────
HZ_SCAN_FILE="$LOG/hz_scan.tmp"
HZ_IMU_FILE="$LOG/hz_imu.tmp"

(while true; do
    source /opt/ros/humble/setup.bash; source "$WS/install/setup.bash"; export ROS_DOMAIN_ID=18
    timeout 6 ros2 topic hz /scan --window 20 2>/dev/null \
        | grep "average rate" | awk '{print $3}' > "$HZ_SCAN_FILE"
    sleep 24
done) &

(while true; do
    source /opt/ros/humble/setup.bash; source "$WS/install/setup.bash"; export ROS_DOMAIN_ID=18
    timeout 6 ros2 topic hz /imu/data --window 20 2>/dev/null \
        | grep "average rate" | awk '{print $3}' > "$HZ_IMU_FILE"
    sleep 24
done) &

# ─── Background: Nav2 goal feedback ─────────────────────────────────────────
NAV_STATUS_FILE="$LOG/nav_status.tmp"
(while true; do
    source /opt/ros/humble/setup.bash; source "$WS/install/setup.bash"; export ROS_DOMAIN_ID=18
    STATUS=$(timeout 2 ros2 topic echo /navigate_to_pose/_action/status \
        --once --no-arr 2>/dev/null | grep "status:" | head -1 | awk '{print $2}')
    case "$STATUS" in
        1) echo "ACCEPTED" ;;
        2) echo "EXECUTING" ;;
        4) echo "SUCCEEDED" ;;
        5) echo "CANCELED" ;;
        6) echo "ABORTED" ;;
        *) echo "UNKNOWN($STATUS)" ;;
    esac > "$NAV_STATUS_FILE"
    sleep 2
done) &

# ─── Background: TF delay check (every 60s) ──────────────────────────────────
TF_FILE="$LOG/tf_check.tmp"
(while true; do
    source /opt/ros/humble/setup.bash; source "$WS/install/setup.bash"; export ROS_DOMAIN_ID=18
    timeout 3 ros2 topic echo /tf --once --no-arr 2>/dev/null \
        | grep -m1 "sec:" | awk '{print $2}' > "$TF_FILE"
    sleep 60
done) &

# ─── Live status loop ────────────────────────────────────────────────────────
PREV_ANG=""
OSC_COUNT=0
RECOVERY_COUNT=0
LOOP_COUNT=0
PREV_STATE=""

echo "--- LIVE STATUS (Ctrl+C to stop) ---"

while true; do
    LOOP_COUNT=$((LOOP_COUNT + 1))
    echo ""
    echo "=== $(date +%H:%M:%S) ==="
    NOW=$(date +%s)

    # ── Process health ────────────────────────────────────────────────────
    NODE_COUNT=$(ros2 node list 2>/dev/null | wc -l)
    WP_ALIVE=$(pgrep -f "run_waypoints" | wc -l)
    echo "[SYS]  nodes=$NODE_COUNT  run_waypoints=$([ "$WP_ALIVE" -gt 0 ] && echo 'RUNNING' || echo 'STOPPED')"

    # ── CPU load ─────────────────────────────────────────────────────────
    CPU_LOAD=$(awk '{print $1}' /proc/loadavg)
    CPU_CORES=$(nproc)
    CPU_WARN=$(echo "$CPU_LOAD $CPU_CORES" | awk '{if($1/$2 > 0.85) print " ⚠️ HIGH" ; else print ""}')
    echo "[CPU]  load=${CPU_LOAD}/${CPU_CORES}${CPU_WARN}"

    # ── cmd_vel ───────────────────────────────────────────────────────────
    CMD_RAW=$(timeout 2 ros2 topic echo /cmd_vel --once --no-arr 2>/dev/null)
    LIN_X=$(echo "$CMD_RAW" | grep -m1 "^  x:" | awk '{printf "%.4f", $2}')
    ANG_Z=$(echo "$CMD_RAW" | grep -m1 "^  z:" | awk '{printf "%.4f", $2}')
    echo "[VEL]  cmd_vel: lin_x=${LIN_X:-N/A}  ang_z=${ANG_Z:-N/A}"

    NAV_RAW=$(timeout 2 ros2 topic echo /cmd_vel_nav --once --no-arr 2>/dev/null)
    NAV_X=$(echo "$NAV_RAW" | grep -m1 "^  x:" | awk '{printf "%.4f", $2}')
    NAV_Z=$(echo "$NAV_RAW" | grep -m1 "^  z:" | awk '{printf "%.4f", $2}')
    echo "[VEL]  cmd_vel_nav: lin_x=${NAV_X:-N/A}  ang_z=${NAV_Z:-N/A}"

    # ── Robot behavior state classifier ───────────────────────────────────
    if [ -n "$LIN_X" ] && [ "$LIN_X" != "N/A" ]; then
        STATE=$(echo "$LIN_X ${ANG_Z:-0}" | awk '{
            lx=$1; az=$2
            az_abs=(az<0)?-az:az
            if     (lx < -0.05)              print "RECOVERY_BACKUP"
            else if(lx < 0.05 && az_abs > 0.8)  print "RECOVERY_SPIN"
            else if(lx < 0.05 && az_abs > 0.15) print "ROTATE_TO_HEADING"
            else if(lx >= 0.05)             print "MOVING_FORWARD"
            else                             print "STOPPED"
        }')
        if [ "$STATE" != "$PREV_STATE" ]; then
            echo ">>> STATE: $PREV_STATE → $STATE"
            echo "$(date +%H:%M:%S)  $PREV_STATE → $STATE" >> "$LOG/state_log.txt"
            if [ "$STATE" = "RECOVERY_BACKUP" ] || [ "$STATE" = "RECOVERY_SPIN" ]; then
                RECOVERY_COUNT=$((RECOVERY_COUNT + 1))
            fi
        else
            echo "[STATE] $STATE"
        fi
        PREV_STATE="$STATE"
    fi

    # ── Oscillation detection ─────────────────────────────────────────────
    if [ -n "$ANG_Z" ] && [ -n "$PREV_ANG" ] && [ "$ANG_Z" != "N/A" ]; then
        SIGN_NOW=$(echo  "$ANG_Z"   | awk '{print ($1>=0)?"pos":"neg"}')
        SIGN_PREV=$(echo "$PREV_ANG" | awk '{print ($1>=0)?"pos":"neg"}')
        IS_NONZERO=$(echo "$ANG_Z" | awk '{print (($1>0.05)||($1<-0.05))?1:0}')
        if [ "$SIGN_NOW" != "$SIGN_PREV" ] && [ "$IS_NONZERO" = "1" ]; then
            OSC_COUNT=$((OSC_COUNT + 1))
        fi
    fi
    PREV_ANG="$ANG_Z"
    [ "$OSC_COUNT" -ge 3 ]      && echo "⚠️  OSCILLATION (ang sign-flips: $OSC_COUNT)"
    [ "$RECOVERY_COUNT" -gt 0 ] && echo "⚠️  RECOVERY events this session: $RECOVERY_COUNT"

    # ── AMCL pose + covariance ────────────────────────────────────────────
    POSE_FULL=$(timeout 3 ros2 topic echo /amcl_pose --once 2>/dev/null)
    PX=$(echo "$POSE_FULL" | grep -A1  "position:"    | grep "x:" | awk '{printf "%.3f", $2}')
    PY=$(echo "$POSE_FULL" | grep -A2  "position:"    | grep "y:" | awk '{printf "%.3f", $2}')
    STAMP=$(echo "$POSE_FULL" | grep -m1 "^    sec:"  | awk '{print $2}')
    STALENESS=""
    if [ -n "$STAMP" ]; then
        AGE=$((NOW - STAMP))
        [ "$AGE" -gt 10 ] && STALENESS=" ⚠️ STALE ${AGE}s"
    fi
    echo "[LOC]  amcl_pose: x=${PX:-N/A}  y=${PY:-N/A}${STALENESS}"

    # Covariance: 36-element flat array — index 1=cov_xx, 8=cov_yy, 36=cov_yaw (1-based in awk)
    COV_VALS=$(echo "$POSE_FULL" | awk '
        /covariance:/{in_c=1; n=0; next}
        in_c && /- /{
            n++; val=$2
            if(n==1)  cxx=val
            if(n==8)  cyy=val
            if(n==36){printf "%.5f %.5f %.5f\n", cxx, cyy, val; in_c=0}
        }')
    COV_XX=$(echo  "$COV_VALS" | awk '{print $1}')
    COV_YY=$(echo  "$COV_VALS" | awk '{print $2}')
    COV_YAW=$(echo "$COV_VALS" | awk '{print $3}')
    COV_STATUS=""
    if [ -n "$COV_XX" ]; then
        COV_STATUS=$(echo "$COV_XX $COV_YY $COV_YAW" | awk '{
            if ($1>0.1 || $2>0.1) print " ⚠️ HIGH_XY_COV (localization uncertain)"
            else if ($3>0.05)     print " ⚠️ HIGH_YAW_COV"
            else                  print " OK"
        }')
        echo "[LOC]  amcl_cov: xx=${COV_XX}  yy=${COV_YY}  yaw=${COV_YAW}${COV_STATUS}"
        echo "$(date +%H:%M:%S)  cov_xx=$COV_XX cov_yy=$COV_YY cov_yaw=$COV_YAW" >> "$LOG/cov_log.txt"
    else
        echo "[LOC]  amcl_cov: N/A"
    fi

    # ── EKF / odom topic health ───────────────────────────────────────────
    EKF_RAW=$(timeout 2 ros2 topic echo /odometry/filtered --once --no-arr 2>/dev/null)
    EKF_V=$(echo "$EKF_RAW" | grep -A3  "twist:" | grep "x:" | head -1 | awk '{printf "%.4f", $2}')
    EKF_W=$(echo "$EKF_RAW" | grep -A10 "twist:" | grep "z:" | head -1 | awk '{printf "%.4f", $2}')
    if [ -n "$EKF_V" ]; then
        echo "[EKF]  lin=${EKF_V} m/s  ang=${EKF_W:-N/A} rad/s  ✅"
    else
        echo "[EKF]  ⚠️  /odometry/filtered ไม่มีข้อมูล — EKF ไม่ทำงาน!"
        # หา odom topic จริงที่ใช้แทน
        if [ $((LOOP_COUNT % 10)) -eq 1 ]; then
            REAL_ODOM=$(ros2 topic list 2>/dev/null | grep -E "odom|filtered" | tr '\n' ' ')
            echo "[EKF]  topic odom ที่พบ: ${REAL_ODOM:-ไม่พบเลย}"
            echo "$(date +%H:%M:%S)  EKF missing — odom topics: $REAL_ODOM" >> "$LOG/rosout_warnings.txt"
        fi
    fi

    # ── Scan Hz ───────────────────────────────────────────────────────────
    if [ -f "$HZ_SCAN_FILE" ] && [ -s "$HZ_SCAN_FILE" ]; then
        SCAN_HZ=$(cat "$HZ_SCAN_FILE")
        SCAN_WARN=$(echo "$SCAN_HZ" | awk '{if($1<8) print " ⚠️ TOO_LOW (need >=10 Hz)"; else print " OK"}')
        echo "[SCAN] Hz=${SCAN_HZ}${SCAN_WARN}"
    else
        echo "[SCAN] Hz=checking..."
    fi

    # ── IMU Hz ────────────────────────────────────────────────────────────
    if [ -f "$HZ_IMU_FILE" ] && [ -s "$HZ_IMU_FILE" ]; then
        IMU_HZ=$(cat "$HZ_IMU_FILE")
        IMU_WARN=$(echo "$IMU_HZ" | awk '{if($1<50) print " ⚠️ TOO_LOW (need >=100 Hz)"; else print " OK"}')
        echo "[IMU]  Hz=${IMU_HZ}${IMU_WARN}"
    else
        echo "[IMU]  Hz=checking..."
    fi

    # ── Nav2 goal status ──────────────────────────────────────────────────
    if [ -f "$NAV_STATUS_FILE" ] && [ -s "$NAV_STATUS_FILE" ]; then
        NAV_GOAL_STATUS=$(cat "$NAV_STATUS_FILE")
        echo "[NAV2] goal_status=${NAV_GOAL_STATUS}"
    fi

    # ── TF freshness ─────────────────────────────────────────────────────
    if [ -f "$TF_FILE" ] && [ -s "$TF_FILE" ]; then
        TF_SEC=$(cat "$TF_FILE")
        if [ -n "$TF_SEC" ] && [ "$TF_SEC" -gt 0 ] 2>/dev/null; then
            TF_AGE=$((NOW - TF_SEC))
            TF_WARN=$([ "$TF_AGE" -gt 5 ] && echo " ⚠️ DELAY ${TF_AGE}s" || echo " OK")
            echo "[TF]   last_stamp_age=${TF_AGE}s${TF_WARN}"
        fi
    fi

    echo "---"
    sleep 3
done
