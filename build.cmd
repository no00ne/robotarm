@echo off
setlocal

REM 激活 conda 环境
call conda activate zhuaWaWaJi



REM 构建项目
python -m build

REM 开发模式安装（方便调试）
python -m pip install -e .

echo.
echo ===== Build finished successfully =====
