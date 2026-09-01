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

# The participant launch owns the admitted spatial production defaults.  The
# environment is an override boundary only; leaving an override unset must not
# silently replace those defaults with the old shadow-only values.
tiny_lidar_control_mode="${TINY_LIDAR_CONTROL_MODE:-}"
tiny_lidar_spatial_shadow_ckpt_path="${TINY_LIDAR_SPATIAL_SHADOW_CKPT_PATH:-}"
tiny_lidar_spatial_shadow_expected_sha256="${TINY_LIDAR_SPATIAL_SHADOW_EXPECTED_SHA256:-}"
tiny_lidar_spatial_shadow_use_base_steering="${TINY_LIDAR_SPATIAL_SHADOW_USE_BASE_STEERING:-false}"
tiny_lidar_spatial_shadow_max_abs_delta_rad="${TINY_LIDAR_SPATIAL_SHADOW_MAX_ABS_DELTA_RAD:-1.2}"
case "${tiny_lidar_spatial_shadow_use_base_steering}" in
    true|false) ;;
    *)
        echo "TINY_LIDAR_SPATIAL_SHADOW_USE_BASE_STEERING must be true or false"
        exit 1
        ;;
esac
if [[ ! "${tiny_lidar_spatial_shadow_max_abs_delta_rad}" =~ ^[0-9]+([.][0-9]+)?$ ]] || \
   [[ "${tiny_lidar_spatial_shadow_max_abs_delta_rad}" =~ ^0+([.]0+)?$ ]]; then
    echo "TINY_LIDAR_SPATIAL_SHADOW_MAX_ABS_DELTA_RAD must be positive"
    exit 1
fi
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
    opts+=("tiny_lidar_spatial_shadow_use_base_steering:=${tiny_lidar_spatial_shadow_use_base_steering}")
    opts+=("tiny_lidar_spatial_shadow_max_abs_delta_rad:=${tiny_lidar_spatial_shadow_max_abs_delta_rad}")
fi
if [[ -n "${tiny_lidar_spatial_shadow_expected_sha256}" ]]; then
    if [[ ! "${tiny_lidar_spatial_shadow_expected_sha256}" =~ ^[0-9a-fA-F]{64}$ ]]; then
        echo "TINY_LIDAR_SPATIAL_SHADOW_EXPECTED_SHA256 must be 64 hexadecimal characters"
        exit 1
    fi
    if [[ -z "${tiny_lidar_spatial_shadow_ckpt_path}" ]]; then
        echo "TINY_LIDAR_SPATIAL_SHADOW_EXPECTED_SHA256 requires TINY_LIDAR_SPATIAL_SHADOW_CKPT_PATH"
        exit 1
    fi
    opts+=("tiny_lidar_spatial_shadow_expected_sha256:=${tiny_lidar_spatial_shadow_expected_sha256,,}")
fi

# A recurrent checkpoint is shadow-only. Merely supplying it cannot demote or
# replace the packaged spatial production authority.
tiny_lidar_recurrent_shadow_ckpt_path="${TINY_LIDAR_RECURRENT_SHADOW_CKPT_PATH:-}"
tiny_lidar_recurrent_shadow_expected_sha256="${TINY_LIDAR_RECURRENT_SHADOW_EXPECTED_SHA256:-}"
if [[ -n "${tiny_lidar_recurrent_shadow_ckpt_path}" ]]; then
    if [[ "${control_method}" != "tiny_lidar_net" ]]; then
        echo "TINY_LIDAR_RECURRENT_SHADOW_CKPT_PATH is only valid with AIC_CONTROL_METHOD=tiny_lidar_net"
        exit 1
    fi
    if [[ ! -f "${tiny_lidar_recurrent_shadow_ckpt_path}" ]]; then
        echo "TINY_LIDAR_RECURRENT_SHADOW_CKPT_PATH does not exist in the runtime container: ${tiny_lidar_recurrent_shadow_ckpt_path}"
        exit 1
    fi
    opts+=("tiny_lidar_recurrent_shadow_ckpt_path:=${tiny_lidar_recurrent_shadow_ckpt_path}")
fi
if [[ -n "${tiny_lidar_recurrent_shadow_expected_sha256}" ]]; then
    if [[ ! "${tiny_lidar_recurrent_shadow_expected_sha256}" =~ ^[0-9a-fA-F]{64}$ ]]; then
        echo "TINY_LIDAR_RECURRENT_SHADOW_EXPECTED_SHA256 must be 64 hexadecimal characters"
        exit 1
    fi
    if [[ -z "${tiny_lidar_recurrent_shadow_ckpt_path}" ]]; then
        echo "TINY_LIDAR_RECURRENT_SHADOW_EXPECTED_SHA256 requires TINY_LIDAR_RECURRENT_SHADOW_CKPT_PATH"
        exit 1
    fi
    opts+=("tiny_lidar_recurrent_shadow_expected_sha256:=${tiny_lidar_recurrent_shadow_expected_sha256,,}")
fi

tiny_lidar_recurrent_authority_enabled="${TINY_LIDAR_RECURRENT_AUTHORITY_ENABLED:-}"
if [[ -n "${TINY_LIDAR_RECURRENT_AUTHORITY_ENABLED+x}" ]]; then
    case "${tiny_lidar_recurrent_authority_enabled}" in
        true|false) ;;
        *)
            echo "TINY_LIDAR_RECURRENT_AUTHORITY_ENABLED must be true or false"
            exit 1
            ;;
    esac
fi
tiny_lidar_recurrent_authority_max_abs_correction_rad="${TINY_LIDAR_RECURRENT_AUTHORITY_MAX_ABS_CORRECTION_RAD:-}"
if [[ -n "${TINY_LIDAR_RECURRENT_AUTHORITY_MAX_ABS_CORRECTION_RAD+x}" ]]; then
    if [[ ! "${tiny_lidar_recurrent_authority_max_abs_correction_rad}" =~ ^[0-9]+([.][0-9]+)?$ ]] || \
       [[ "${tiny_lidar_recurrent_authority_max_abs_correction_rad}" =~ ^0+([.]0+)?$ ]]; then
        echo "TINY_LIDAR_RECURRENT_AUTHORITY_MAX_ABS_CORRECTION_RAD must be positive"
        exit 1
    fi
fi
if [[ "${tiny_lidar_recurrent_authority_enabled}" == "true" && \
      -z "${tiny_lidar_recurrent_shadow_ckpt_path}" ]]; then
    echo "recurrent authority requires TINY_LIDAR_RECURRENT_SHADOW_CKPT_PATH"
    exit 1
fi
if [[ "${tiny_lidar_recurrent_authority_enabled}" == "true" && \
      -z "${tiny_lidar_recurrent_shadow_expected_sha256}" ]]; then
    echo "recurrent authority requires TINY_LIDAR_RECURRENT_SHADOW_EXPECTED_SHA256"
    exit 1
fi
if [[ -n "${tiny_lidar_recurrent_authority_enabled}" ]]; then
    opts+=("tiny_lidar_recurrent_authority_enabled:=${tiny_lidar_recurrent_authority_enabled}")
fi
if [[ -n "${tiny_lidar_recurrent_authority_max_abs_correction_rad}" ]]; then
    opts+=("tiny_lidar_recurrent_authority_max_abs_correction_rad:=${tiny_lidar_recurrent_authority_max_abs_correction_rad}")
fi

tiny_lidar_spatial_authority_enabled="${TINY_LIDAR_SPATIAL_AUTHORITY_ENABLED:-}"
if [[ -n "${TINY_LIDAR_SPATIAL_AUTHORITY_ENABLED+x}" ]]; then
    case "${tiny_lidar_spatial_authority_enabled}" in
        true|false) ;;
        *)
            echo "TINY_LIDAR_SPATIAL_AUTHORITY_ENABLED must be true or false"
            exit 1
            ;;
    esac
elif [[ -n "${TINY_LIDAR_SPATIAL_SHADOW_CKPT_PATH+x}" || \
        -n "${tiny_lidar_residual_ckpt_path}" ]]; then
    # Supplying an experiment artifact without an explicit authority grant is
    # shadow/diagnostic by construction.
    tiny_lidar_spatial_authority_enabled="false"
elif [[ "${tiny_lidar_control_mode}" == "gap_teacher" || \
        "${tiny_lidar_control_mode}" == "precontact_teacher" || \
        "${tiny_lidar_control_mode}" == "speed_committed_teacher" ]]; then
    # Teacher steering is diagnostic authority and must not inherit the
    # participant production adapter merely because it shares the node.
    tiny_lidar_spatial_authority_enabled="false"
fi

tiny_lidar_spatial_authority_max_abs_delta_rad="${TINY_LIDAR_SPATIAL_AUTHORITY_MAX_ABS_DELTA_RAD:-}"
if [[ -n "${TINY_LIDAR_SPATIAL_AUTHORITY_MAX_ABS_DELTA_RAD+x}" ]]; then
    if [[ ! "${tiny_lidar_spatial_authority_max_abs_delta_rad}" =~ ^[0-9]+([.][0-9]+)?$ ]] || \
       [[ "${tiny_lidar_spatial_authority_max_abs_delta_rad}" =~ ^0+([.]0+)?$ ]]; then
        echo "TINY_LIDAR_SPATIAL_AUTHORITY_MAX_ABS_DELTA_RAD must be positive"
        exit 1
    fi
fi
if [[ "${tiny_lidar_spatial_authority_enabled}" == "true" && \
      -n "${TINY_LIDAR_SPATIAL_SHADOW_CKPT_PATH+x}" && \
      -z "${tiny_lidar_spatial_shadow_ckpt_path}" ]]; then
    echo "spatial authority requires TINY_LIDAR_SPATIAL_SHADOW_CKPT_PATH"
    exit 1
fi
if [[ "${tiny_lidar_spatial_authority_enabled}" == "true" ]]; then
    if [[ -n "${tiny_lidar_residual_ckpt_path}" ]]; then
        echo "spatial authority cannot be combined with TINY_LIDAR_RESIDUAL_CKPT_PATH"
        exit 1
    fi
fi
if [[ -n "${tiny_lidar_spatial_authority_enabled}" ]]; then
    opts+=("tiny_lidar_spatial_authority_enabled:=${tiny_lidar_spatial_authority_enabled}")
fi
if [[ -n "${tiny_lidar_spatial_authority_max_abs_delta_rad}" ]]; then
    opts+=("tiny_lidar_spatial_authority_max_abs_delta_rad:=${tiny_lidar_spatial_authority_max_abs_delta_rad}")
fi

# The gap teacher is an explicit data-collection mode under the existing
# tiny_lidar_net interface. It cannot be selected accidentally by another
# controller or by an unknown spelling.
if [[ -n "${tiny_lidar_control_mode}" ]]; then
    if [[ "${control_method}" != "tiny_lidar_net" ]]; then
        echo "TINY_LIDAR_CONTROL_MODE is only valid with AIC_CONTROL_METHOD=tiny_lidar_net"
        exit 1
    fi
    case "${tiny_lidar_control_mode}" in
    "fixed"|"fixed_lidar_brake"|"speed_aware_lidar_brake"|"ai"|"gap_teacher"|"precontact_teacher"|"speed_committed_teacher")
        ;;
    *)
        echo "invalid TINY_LIDAR_CONTROL_MODE '${tiny_lidar_control_mode}'"
        exit 1
        ;;
    esac
    opts+=("tiny_lidar_control_mode:=${tiny_lidar_control_mode}")
fi

tiny_lidar_acceleration="${TINY_LIDAR_ACCELERATION:-}"
if [[ -n "${TINY_LIDAR_ACCELERATION+x}" ]]; then
    if [[ ! "${tiny_lidar_acceleration}" =~ ^[0-9]+([.][0-9]+)?$ ]] || \
       [[ "${tiny_lidar_acceleration}" =~ ^0+([.]0+)?$ ]]; then
        echo "TINY_LIDAR_ACCELERATION must be finite and positive"
        exit 1
    fi
    if [[ "${control_method}" != "tiny_lidar_net" ]]; then
        echo "TINY_LIDAR_ACCELERATION is only valid with AIC_CONTROL_METHOD=tiny_lidar_net"
        exit 1
    fi
    opts+=("tiny_lidar_acceleration:=${tiny_lidar_acceleration}")
fi

tiny_lidar_maximum_forward_speed_mps="${TINY_LIDAR_MAXIMUM_FORWARD_SPEED_MPS:-}"
if [[ -n "${TINY_LIDAR_MAXIMUM_FORWARD_SPEED_MPS+x}" ]]; then
    if [[ ! "${tiny_lidar_maximum_forward_speed_mps}" =~ ^[0-9]+([.][0-9]+)?$ ]]; then
        echo "TINY_LIDAR_MAXIMUM_FORWARD_SPEED_MPS must be finite and non-negative"
        exit 1
    fi
    if [[ "${control_method}" != "tiny_lidar_net" ]]; then
        echo "TINY_LIDAR_MAXIMUM_FORWARD_SPEED_MPS is only valid with AIC_CONTROL_METHOD=tiny_lidar_net"
        exit 1
    fi
    opts+=("tiny_lidar_maximum_forward_speed_mps:=${tiny_lidar_maximum_forward_speed_mps}")
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
