"""GPU PRIME offload helper for ROS2 launch files.

Optimus 노트북(Intel iGPU + NVIDIA discrete GPU)에서 GUI 프로세스(Gazebo,
RViz2, rqt 등)를 NVIDIA GPU로 강제 라우팅한다. 순수 NVIDIA 데스크톱이나
NVIDIA 미설치 환경, 헤드리스 환경에서는 자동으로 no-op이 된다.

배포 안전을 위해 launch 인자 ``gpu`` (auto/nvidia_offload/native)로
override 가능하다.

사용법::

    from launch import LaunchDescription
    from launch_utils import setup_gpu_offload

    def generate_launch_description():
        return LaunchDescription([
            *setup_gpu_offload(),
            # 나머지 launch actions...
        ])
"""

import os
import shutil
import subprocess

from launch.actions import DeclareLaunchArgument, OpaqueFunction, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration


def detect_gpu_mode() -> str:
    """현재 시스템 환경에서 GPU 모드 자동 감지.

    Returns:
        'nvidia_offload' — Optimus (Intel iGPU + NVIDIA discrete). PRIME 환경변수 필요.
        'native'         — 순수 NVIDIA 데스크톱, NVIDIA 미설치, 또는 감지 실패. 환경변수 불필요.
        'none'           — DISPLAY 없음 (헤드리스). GUI 의미 없음.
    """
    if not os.environ.get('DISPLAY'):
        return 'none'
    if not shutil.which('nvidia-smi'):
        return 'native'
    try:
        out = subprocess.run(
            ['xrandr', '--listproviders'],
            capture_output=True, text=True, timeout=2,
        ).stdout
        if 'NVIDIA' in out and 'Sink Offload' in out:
            return 'nvidia_offload'
        return 'native'
    except Exception:
        return 'native'


def _apply_gpu_env(context):
    mode = LaunchConfiguration('gpu').perform(context)
    if mode == 'auto':
        mode = detect_gpu_mode()
    if mode == 'nvidia_offload':
        return [
            SetEnvironmentVariable('__NV_PRIME_RENDER_OFFLOAD', '1'),
            SetEnvironmentVariable('__GLX_VENDOR_LIBRARY_NAME', 'nvidia'),
            SetEnvironmentVariable('__VK_LAYER_NV_optimus', 'NVIDIA_only'),
            SetEnvironmentVariable('__NV_PRIME_RENDER_OFFLOAD_PROVIDER', 'NVIDIA-G0'),
        ]
    return []


def setup_gpu_offload():
    """LaunchDescription에 unpack(*)으로 넣을 액션 리스트 반환.

    [DeclareLaunchArgument('gpu', default='auto'), OpaqueFunction(...)] 두 개를 반환.
    """
    return [
        DeclareLaunchArgument(
            'gpu',
            default_value='auto',
            description='GPU mode: auto | nvidia_offload | native',
        ),
        OpaqueFunction(function=_apply_gpu_env),
    ]
