#!/bin/bash
set -e

# 如果你用 venv，需要改成你自己的路径，比如 venv/bin/activate
# source venv/bin/activate

python3 -m pip install --upgrade build setuptools wheel
python3 -m build
python3 -m pip install -e .