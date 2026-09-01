#!/bin/bash

set -euo pipefail

mkdir -p submit
tar \
  --exclude='*/__pycache__' \
  --exclude='*/.pytest_cache' \
  --exclude='*.py[co]' \
  -zcvf submit/aichallenge_submit.tar.gz \
  -C ./aichallenge/workspace/src \
  aichallenge_submit
