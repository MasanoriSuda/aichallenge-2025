#!/bin/bash
set -euo pipefail
mpcc_probe_out=/output/20260909-actuation-instrumentation-r2
mkdir "$mpcc_probe_out"
# The Compose image binds the host passwd/group read-only. Extract compiler
# packages only inside this disposable container, without dpkg user scripts.
apt-get update -qq
apt-get install --download-only -y --no-install-recommends mono-mcs libmono-cecil-private-cil
for mpcc_probe_deb in /var/cache/apt/archives/*.deb; do
  dpkg-deb -x "$mpcc_probe_deb" /
done
mcs -target:library -out:"$mpcc_probe_out/ObservationProbe.dll" /task-steering/20260909-mpcc-actuation-application/ObservationProbe.cs
mcs -r:/usr/lib/mono/gac/Mono.Cecil/0.11.0.0__0738eb9f132ed756/Mono.Cecil.dll -out:"$mpcc_probe_out/InstrumentInput.exe" /task-steering/20260909-mpcc-actuation-application/InstrumentInput.cs
mono "$mpcc_probe_out/InstrumentInput.exe" /aichallenge/simulator/AWSIM/AWSIM_Data/Managed/Assembly-CSharp.dll "$mpcc_probe_out/ObservationProbe.dll" "$mpcc_probe_out/Assembly-CSharp.dll"
chown -R 1000:1000 "$mpcc_probe_out"
