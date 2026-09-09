#!/bin/bash
set -euo pipefail
mpcc_probe_out=/output/20260909-longitudinal-physics-instrumentation-r1
mkdir "$mpcc_probe_out"
apt-get update -qq
apt-get install --download-only -y --no-install-recommends mono-mcs libmono-cecil-private-cil
for mpcc_probe_deb in /var/cache/apt/archives/*.deb; do dpkg-deb -x "$mpcc_probe_deb" /; done
mcs -target:library -out:"$mpcc_probe_out/ObservationProbe.dll" /task-steering/20260909-mpcc-longitudinal-model-contract/ObservationProbe.cs
cp /output/20260909-actuation-instrumentation-r2/Assembly-CSharp.dll "$mpcc_probe_out/Assembly-CSharp.dll"
cp /output/20260909-actuation-instrumentation-r2/cil-validation-r2.json "$mpcc_probe_out/cil-validation-r2.json"
cp /task-steering/20260909-mpcc-longitudinal-model-contract/ObservationProbe.cs "$mpcc_probe_out/ObservationProbe.cs"
chown -R 1000:1000 "$mpcc_probe_out"
