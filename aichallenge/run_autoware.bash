#!/bin/bash

mode="${1}"
id="${2:-${ROS_DOMAIN_ID:-0}}"
vehicle_count="${AIC_VEHICLE_COUNT:-1}"
out_dir="${3:+${3}/d${id}}"
out_dir="${out_dir:-/output/$(date +%Y%m%d-%H%M%S)/d${id}}"

case "${mode}" in
"awsim")
    opts=("simulation:=true" "use_sim_time:=true" "run_rviz:=true")
    ;;
"awsim-no-viz")
    opts=("simulation:=true" "use_sim_time:=true" "run_rviz:=false")
    ;;
"vehicle")
    opts=("simulation:=false" "use_sim_time:=false" "run_rviz:=false")
    ;;
"rosbag")
    opts=("simulation:=false" "use_sim_time:=true" "run_rviz:=true")
    ;;
*)
    echo "invalid argument (use 'awsim' or 'vehicle' or 'rosbag')"
    exit 1
    ;;
esac

control_method="${AIC_CONTROL_METHOD:-tiny_lidar_net}"
case "${control_method}" in
"mpc"|"pure_pursuit"|"tiny_lidar_net"|"pilot_net"|"joycon")
    ;;
*)
    echo "invalid AIC_CONTROL_METHOD '${control_method}'"
    exit 1
    ;;
esac
opts+=("control_method:=${control_method}")

# A candidate checkpoint is an explicit A/B-test input. An unset override leaves
# the launch/package default untouched, so experiments cannot silently replace
# the production checkpoint.
tiny_lidar_ckpt_path="${TINY_LIDAR_CKPT_PATH:-}"
if [[ -n "${tiny_lidar_ckpt_path}" ]]; then
    if [[ "${control_method}" != "tiny_lidar_net" ]]; then
        echo "TINY_LIDAR_CKPT_PATH is only valid with AIC_CONTROL_METHOD=tiny_lidar_net"
        exit 1
    fi
    if [[ ! -f "${tiny_lidar_ckpt_path}" ]]; then
        echo "TINY_LIDAR_CKPT_PATH does not exist in the runtime container: ${tiny_lidar_ckpt_path}"
        exit 1
    fi
    opts+=("tiny_lidar_ckpt_path:=${tiny_lidar_ckpt_path}")
fi

# The learned residual is a separately gated A/B artifact.  Empty means the
# admitted base policy remains bit-for-bit unchanged.
tiny_lidar_residual_ckpt_path="${TINY_LIDAR_RESIDUAL_CKPT_PATH:-}"
tiny_lidar_residual_architecture="${TINY_LIDAR_RESIDUAL_ARCHITECTURE:-stateless}"
if [[ -n "${tiny_lidar_residual_ckpt_path}" ]]; then
    if [[ "${control_method}" != "tiny_lidar_net" ]]; then
        echo "TINY_LIDAR_RESIDUAL_CKPT_PATH is only valid with AIC_CONTROL_METHOD=tiny_lidar_net"
        exit 1
    fi
    if [[ ! -f "${tiny_lidar_residual_ckpt_path}" ]]; then
        echo "TINY_LIDAR_RESIDUAL_CKPT_PATH does not exist in the runtime container: ${tiny_lidar_residual_ckpt_path}"
        exit 1
    fi
    opts+=("tiny_lidar_residual_ckpt_path:=${tiny_lidar_residual_ckpt_path}")
    opts+=("tiny_lidar_residual_architecture:=${tiny_lidar_residual_architecture}")
fi

# A spatial candidate can run beside production without owning the command.
# Empty keeps the production callback bit-for-bit on its admitted path.
tiny_lidar_spatial_shadow_ckpt_path="${TINY_LIDAR_SPATIAL_SHADOW_CKPT_PATH:-}"
if [[ -n "${tiny_lidar_spatial_shadow_ckpt_path}" ]]; then
    if [[ "${control_method}" != "tiny_lidar_net" ]]; then
        echo "TINY_LIDAR_SPATIAL_SHADOW_CKPT_PATH is only valid with AIC_CONTROL_METHOD=tiny_lidar_net"
        exit 1
    fi
    if [[ ! -f "${tiny_lidar_spatial_shadow_ckpt_path}" ]]; then
        echo "TINY_LIDAR_SPATIAL_SHADOW_CKPT_PATH does not exist in the runtime container: ${tiny_lidar_spatial_shadow_ckpt_path}"
        exit 1
    fi
    opts+=("tiny_lidar_spatial_shadow_ckpt_path:=${tiny_lidar_spatial_shadow_ckpt_path}")
fi

tiny_lidar_spatial_authority_enabled="${TINY_LIDAR_SPATIAL_AUTHORITY_ENABLED:-false}"
case "${tiny_lidar_spatial_authority_enabled}" in
    true|false) ;;
    *)
        echo "TINY_LIDAR_SPATIAL_AUTHORITY_ENABLED must be true or false"
        exit 1
        ;;
esac
if [[ "${tiny_lidar_spatial_authority_enabled}" == "true" ]]; then
    if [[ -z "${tiny_lidar_spatial_shadow_ckpt_path}" ]]; then
        echo "spatial authority requires TINY_LIDAR_SPATIAL_SHADOW_CKPT_PATH"
        exit 1
    fi
    if [[ -n "${tiny_lidar_residual_ckpt_path}" ]]; then
        echo "spatial authority cannot be combined with TINY_LIDAR_RESIDUAL_CKPT_PATH"
        exit 1
    fi
fi
opts+=("tiny_lidar_spatial_authority_enabled:=${tiny_lidar_spatial_authority_enabled}")

# The gap teacher is an explicit data-collection mode under the existing
# tiny_lidar_net interface. It cannot be selected accidentally by another
# controller or by an unknown spelling.
tiny_lidar_control_mode="${TINY_LIDAR_CONTROL_MODE:-}"
if [[ -n "${tiny_lidar_control_mode}" ]]; then
    if [[ "${control_method}" != "tiny_lidar_net" ]]; then
        echo "TINY_LIDAR_CONTROL_MODE is only valid with AIC_CONTROL_METHOD=tiny_lidar_net"
        exit 1
    fi
    case "${tiny_lidar_control_mode}" in
    "fixed"|"fixed_lidar_brake"|"ai"|"gap_teacher"|"precontact_teacher")
        ;;
    *)
        echo "invalid TINY_LIDAR_CONTROL_MODE '${tiny_lidar_control_mode}'"
        exit 1
        ;;
    esac
    opts+=("tiny_lidar_control_mode:=${tiny_lidar_control_mode}")
fi

export ROS_DOMAIN_ID=$id

mkdir -p "${out_dir}"
exec >"${out_dir}/autoware.log" 2>&1

cd "${out_dir}" || exit
# Persist ROS node logs under the run output directory (so autostart_orchestrator logs are collectible).
export ROS_HOME="${out_dir}/ros"
export ROS_LOG_DIR="${ROS_HOME}/log"
mkdir -p "${ROS_LOG_DIR}"

# set -m keeps bash from setting SIGINT to SIG_IGN on the backgrounded child (then the forwarded INT would be a no-op).
set -m
ros2 launch aichallenge_system_launch aichallenge_system.launch.xml \
    "${opts[@]}" "domain_id:=$id" "vehicle_count:=${vehicle_count}" &
trap 'kill -INT $! 2>/dev/null' TERM INT
while kill -0 $! 2>/dev/null; do wait; done
