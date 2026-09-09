#!/bin/bash
set -euo pipefail
mpcc_force_out=/output/20260909-force-step-instrumentation-r4
mpcc_force_source=/task-steering/20260909-mpcc-force-step-model
mpcc_force_managed=/aichallenge/simulator/AWSIM/AWSIM_Data/Managed
mkdir "$mpcc_force_out"
trap 'cp "$mpcc_force_source/ObservationProbe.cs" "$mpcc_force_source/ForceProbe.cs" "$mpcc_force_source/InstrumentInput.cs" "$mpcc_force_source/build_probe.sh" "$mpcc_force_out/"; chown -R 1000:1000 "$mpcc_force_out"' EXIT
apt-get update -qq
apt-get install --download-only -y --no-install-recommends mono-mcs libmono-cecil-private-cil
for mpcc_force_deb in /var/cache/apt/archives/*.deb; do dpkg-deb -x "$mpcc_force_deb" /; done
mcs -target:library -out:"$mpcc_force_out/ObservationProbe.dll" \
  -r:"$mpcc_force_managed/netstandard.dll" \
  -r:"$mpcc_force_managed/UnityEngine.VehiclesModule.dll" \
  -r:"$mpcc_force_managed/UnityEngine.CoreModule.dll" \
  -r:"$mpcc_force_managed/UnityEngine.PhysicsModule.dll" \
  "$mpcc_force_source/ObservationProbe.cs" "$mpcc_force_source/ForceProbe.cs"
mpcc_force_cecil=/usr/lib/mono/gac/Mono.Cecil/0.11.0.0__0738eb9f132ed756/Mono.Cecil.dll
mcs -r:"$mpcc_force_cecil" -out:"$mpcc_force_out/instrument.exe" "$mpcc_force_source/InstrumentInput.cs"
mono "$mpcc_force_out/instrument.exe" "$mpcc_force_managed/Assembly-CSharp.dll" \
  "$mpcc_force_out/ObservationProbe.dll" "$mpcc_force_out/Assembly-CSharp.dll"
cp "$mpcc_force_source/ObservationProbe.cs" "$mpcc_force_source/ForceProbe.cs" "$mpcc_force_source/InstrumentInput.cs" "$mpcc_force_source/build_probe.sh" "$mpcc_force_out/"
chown -R 1000:1000 "$mpcc_force_out"
