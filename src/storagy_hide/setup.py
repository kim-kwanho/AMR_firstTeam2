from glob import glob

from setuptools import find_packages, setup

package_name = 'storagy_hide'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/params', glob('params/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='AMR_firstTeam1',
    maintainer_email='firstteam1@todo.todo',
    description='숨는팀 패키지: 위장 상태제어(FSM) / ArUco 정밀 도킹 / 사람 감지 + 동적 코스트맵',
    license='Apache-2.0',
    extras_require={'test': ['pytest']},
    entry_points={
        'console_scripts': [
            # R1 — 위장 상태제어 FSM + SetLamp/Emotion 서버
            'state_machine = storagy_hide.state_machine:main',
            # R2 — ArUco 정밀 도킹 + 복귀
            'aruco_dock = storagy_hide.aruco_dock:main',
            # R3 — 사람 감지 + 120도 keepout 영역 산출
            'human_perception = storagy_hide.human_perception:main',
            # R4 — 동적 코스트맵 주입
            'dynamic_costmap = storagy_hide.dynamic_costmap:main',
        ],
    },
)
