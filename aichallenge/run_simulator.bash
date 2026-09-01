#!/bin/bash

SCRIPT_DIR="$(cd -- "$(dirname -- "$0")/simulator_scripts" && pwd)"
mode="${1:-${SIM_MODE:-simulator}}"
[ $# -gt 0 ] && shift

# dev2 / gate2 等は dev.sh / gate.sh に番号を渡すエイリアス
[[ ${mode} =~ ^(dev|gate)([0-9]+)$ ]] && set -- "${BASH_REMATCH[2]}" "$@" && mode="${BASH_REMATCH[1]}"

# simulator_scripts 内のスクリプトを呼び出す
script="${SCRIPT_DIR}/${mode}.sh"
if [[ ! -f ${script} ]]; then
    echo "[ERROR] unknown mode '${mode}' (supported: $(basename -s .sh "${SCRIPT_DIR}"/*.sh | xargs) dev<N> gate<N>)" >&2
    exit 1
fi

log_dir="${LOG_DIR:-/output}"
mkdir -p "${log_dir}"
log_dir="$(cd -- "${log_dir}" && pwd)"
exec >"${log_dir}/awsim.log" 2>&1

echo "[INFO] Starting AWSIM: ${mode}.sh $*"
# AWSIM writes result-summary.json and dN-result-details.json to its current
# working directory.  Keep the process cwd inside this immutable run directory
# so separate runs cannot overwrite /aichallenge/result-summary.json.
cd -- "${log_dir}"
exec bash "${script}" "$@"
