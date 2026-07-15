#!/bin/bash

tar zcvf submit/aichallenge_submit.tar.gz \
  --exclude='*/__pycache__' \
  --exclude='*/.pytest_cache' \
  --exclude='*.pyc' \
  --exclude='*.pyo' \
  -C ./aichallenge/workspace/src aichallenge_submit
