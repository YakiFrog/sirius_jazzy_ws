#!/usr/bin/env bash
# Start Roboteq with optional per-robot direction signs.

set -eo pipefail

WS_DIR="${HOME}/sirius_jazzy_ws"
PROFILE_FILE="${SIRIUS_ROBOTEQ_PROFILE:-${HOME}/.config/sirius/roboteq_profile.sh}"
PUB_ODOM_TF="${1:-false}"

source /opt/ros/jazzy/setup.bash
source "${WS_DIR}/install/setup.bash"

# A profile is deliberately kept outside the shared repository so each robot
# can select its own motor wiring and encoder polarity.
if [ -f "${PROFILE_FILE}" ]; then
    # shellcheck disable=SC1090
    source "${PROFILE_FILE}"
fi

MOTOR_SIGN_R="${SIRIUS_MOTOR_SIGN_R:-1.0}"
MOTOR_SIGN_L="${SIRIUS_MOTOR_SIGN_L:--1.0}"
ENCODER_SIGN_R="${SIRIUS_ENCODER_SIGN_R:--1.0}"
ENCODER_SIGN_L="${SIRIUS_ENCODER_SIGN_L:-1.0}"

echo "Roboteq direction profile: ${PROFILE_FILE}"
echo "  motor  R=${MOTOR_SIGN_R} L=${MOTOR_SIGN_L}"
echo "  encoder R=${ENCODER_SIGN_R} L=${ENCODER_SIGN_L}"

exec ros2 launch roboteq_ros2_driver roboteq_ros2_driver.launch.py \
    pub_odom_tf:="${PUB_ODOM_TF}" \
    motor_sign_r:="${MOTOR_SIGN_R}" \
    motor_sign_l:="${MOTOR_SIGN_L}" \
    encoder_sign_r:="${ENCODER_SIGN_R}" \
    encoder_sign_l:="${ENCODER_SIGN_L}"
