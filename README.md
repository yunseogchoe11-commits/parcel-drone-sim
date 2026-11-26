# parcel-drone-sim — 파일 안내

자율 택배 드론 졸업설계의 가상환경(PX4 SITL + Gazebo) 저장소.
파일이 추가되거나 역할이 바뀌면 이 문서를 같이 고친다.

## 규칙
- **파일·폴더 이름은 바꾸지 않는다.** PX4가 정확한 이름으로 찾는다.
- 결과 폴더 이름: `results/날짜_실험이름/` (예: `results/261031_flow_hover_1p5m/`)
- **"생성됨" 표시 파일은 직접 고치지 않는다.** 만드는 스크립트나 설정 파일을 고친 뒤 다시 생성한다.
- 설명은 각 폴더의 `설명.md`와 이 README에 적는다.

## 폴더 구조
~~~
parcel-drone-sim/
├─ .vscode/          VS Code 설정 (터미널 자동 생성)
├─ missions/         미션 경로 (예정)
├─ scripts/          자율 임무 코드 (미션 비행)
├─ models/           Gazebo 모델 (기체, 강당 바닥, AprilTag)
├─ worlds/           Gazebo 월드 (강당)
├─ results/          실험 결과 (그래프, 요약)
└─ tools/            계산·생성·분석 도구
   ├─ world/         강당 월드·바닥 그림 생성
   ├─ vehicle/       기체 무게·무게중심·모터·카메라 설정
   └─ analysis/      비행 로그 분석 (호버·여유 시험·무게 시험·로그 첨부)
~~~

## .vscode
| 파일 | 역할 |
|---|---|
| tasks.json | 폴더를 열면 터미널 3개(SIM, QGC, CMD)를 자동으로 만든다 |

## models — 기체와 바닥
| 경로 | 역할 | 직접 수정 |
|---|---|---|
| x500_mydrone/model.sdf | (센서 위치는 `set_sensors.py`로) **모터 4개**(회전 방향, 최대 회전수, 추력 계수 — 11/20부터 실제 모터 AIR2216 II 값) + **광류 카메라·거리계** (11/24부터 M3 앞-왼쪽 모터 아래: (169, 175, −5)·(189, 175, −5) mm) + **아래쪽 카메라**(`set_camera.py`가 추가, 11/24부터 그리퍼 하우징 아래 (0, 0, −144) mm, 120° 렌즈). 모터 값은 `tools/vehicle/set_mass_cg.py`가 고침 | 모터·카메라는 도구로 |
| x500_mydrone/model.config | 모델 이름표 | X |
| x500_mydrone/thumbnails/, LICENSE | PX4에서 복사된 부속 파일 | X |
| x500_mydrone_base/model.sdf | **몸체**: 무게·무게중심·관성, 외형(기본 X500, 다리 포함), 로터 위치, 기본 센서(IMU, 기압계, 지자기, GPS). IMU는 FC 위치(하판 아랫면 위 36 mm) | 무게·무게중심·관성·로터 위치는 `tools/vehicle/set_mass_cg.py`가 자동 수정 (옛 방식: `cg_calc.py apply`) |
| x500_mydrone_base/model.config | 모델 이름표 | X |
| auditorium_floor/model.sdf, model.config | 강당 바닥 모델 (크기, 그림 연결) | X (생성됨) |
| auditorium_floor/materials/textures/floor_wood.jpg | 나무 마루 바닥 그림 (30 × 20 m, 코트 선 포함) | X (생성됨) |
| auditorium_floor/materials/textures/floor_mat.png | 이전 무늬 매트 그림 (지금은 안 씀) | — |
| apriltag_36h11_0/tag.png | AprilTag tag36h11 0번 그림 (흰 여백 포함 10×10 셀). tag.obj·tag.mtl은 `add_apriltag.py`가 생성 | X |

## worlds — 월드
| 파일 | 역할 | 직접 수정 |
|---|---|---|
| auditorium.sdf | 실내 강당 월드 (바닥, 벽, H/D/P 표시, 택배 상자) | X (생성됨) |
| 설명.md | 강당 크기, 동선 좌표표 | O |

## tools — 도구
### tools/ (바로 아래: 자주 쓰는 것)
| 파일 | 역할 | 실행 |
|---|---|---|
| run_mydrone_flow.sh | **우리 기체 + 광류 실행** (센서·IMU 위치, 무게중심 보정값(10/13 실험 E), 기수·나침반 설정, 수평 속도 제한, 남은 Gazebo 정리, 끝나면 Gazebo 자동 종료) | `bash tools/run_mydrone_flow.sh` |
| **run_mydrone_flow_test.sh** | **지금 기체 설정으로 실행** (생성됨). run_mydrone_flow.sh를 복사해서 무게중심·모터에 따라 바뀌는 값만 다시 계산: CA_ROTOR*_PX/PY, EKF2_OF/RNG/IMU_POS_X, SIM_GZ_EC_MIN/MAX(모터 회전 범위), BAT1_CAPACITY 5200. 원본은 그대로 둠 | `bash tools/run_mydrone_flow_test.sh` (`set_mass_cg.py`가 만듦) |
| weight_sweep.sh | **무게 자동 시험**: 무게마다 설정 → 시뮬 켜기 → 여유 시험 → 끄기 → 분석을 반복, 끝에 표·그래프 | `bash tools/weight_sweep.sh 결과폴더 모터 무게1 무게2 ...` (시뮬 꺼진 상태에서) |
| **auditorium_config.py** | **강당 설정값** (크기, H/D/P 위치, 택배, 판자 폭). 월드·미션·분석이 같이 읽음. 강당을 바꿀 때 여기만 고침 | — |

### tools/world/ — 강당 월드 만들기
| 파일 | 역할 | 실행 |
|---|---|---|
| make_auditorium_world.py | 설정값으로 강당 월드와 바닥 모델 생성 | `python3 tools/world/make_auditorium_world.py` |
| make_floor_texture.py | 참고 사진으로 나무 마루 그림 생성 (numpy, Pillow 필요) | `python3 tools/world/make_floor_texture.py` |
| ref/floor_ref_topview.png | 바닥 참고 사진 (학교 체육관 거리뷰 캡처, 시험용) | — |
| add_apriltag.py | P 택배 상자 윗면에 AprilTag(tag36h11 0번, 바깥 0.18 m) 붙이기. 월드에 이 PC의 절대 경로가 들어가므로 auditorium.sdf는 올리지 않고 각 PC에서 실행 | `python3 tools/world/add_apriltag.py` (떼기: `remove`) |
| apriltag_config.txt | 태그 종류·크기 (생성됨, tag_approach_test.py가 읽음) | — |

### tools/vehicle/ — 기체 무게·무게중심·모터·카메라
| 파일 | 역할 | 실행 |
|---|---|---|
| **parts.csv** | **부품 무게·위치 표** (무게중심 계산 입력) | — |
| cg_calc.py | 무게중심 계산. `apply`를 붙이면 몸체 모델에 적용 | `python3 tools/vehicle/cg_calc.py apply` |
| **set_mass_cg.py** | **총무게·무게중심(x, y, z)·모터 한 번에 설정** → 두 model.sdf + run_mydrone_flow_test.sh. 모터 기본 `air2216`(실제 모터, 4.57 kgf), `air2216_low`(배터리 14.4 V + 손실 10 %, 3.90 kgf), `px4`(옛 모델 3.49 kgf). 옵션 `--inertia ixx iyy izz`(관성 직접), `--arm 0.177`(모터 위치), `--cg-y`·`--cg-z`(무게중심 y·z mm), `--ground`(착지 바닥 높이 mm) | `python3 tools/vehicle/set_mass_cg.py 2600 -12.1 --cg-y -1.0 --cg-z -34.8` (되돌리기: `reset`) |
| last_setting.txt | 마지막 set_mass_cg.py 실행에서 바뀐 값 표 (생성됨, 보고용) | — |
| set_camera.py | 아래쪽 카메라 달기 (기본 11/24: IMX219 120° 렌즈 = 가로 90°, 640×480, 15 Hz, 그리퍼 하우징 아래 (0, 0, −0.144) m, 바로 아래). 토픽 /down_camera/image. `set_mass_cg.py reset` 뒤에는 다시 실행 | `python3 tools/vehicle/set_camera.py [--z -0.144 --pitch 90 --hfov 90]` (떼기: `remove`) |
| camera_config.txt | 카메라 위치·화각 (생성됨, tag_approach_test.py가 읽음) | — |
| **set_sensors.py** | **센서 3개 위치 한 번에**: 광류·거리계·아래쪽 카메라 (mm, 앞/왼/위, 하판 아랫면 기준) → model.sdf pose·주석 + set_camera.py + PX4 센서 위치(EKF2_OF/RNG_POS_X/Y/Z, EKF2_MIN_RNG)를 지금 무게중심 기준으로 다시 계산 | `python3 tools/vehicle/set_sensors.py --flow 169 175 -5 --range 189 175 -5 --camera 0 0 -144 90` (보기: `show`) |
| **set_gripper.py** | **그리퍼를 기체 모델에 붙이기** (선배 STEP → meshes/): 장착판·하우징·착륙 다리 4개(고정), 가운데 손가락 2개(관절, /gripper/finger_l·r), 택배 붙이기·떼기(DetachableJoint, /parcel_H·/parcel attach·detach). 기체 모양도 실제 CAD(x500 다리 없음). 몸통 무게 = 지금 - 택배 0.2 - 그리퍼 0.72 kg, 무게중심은 드론 전체(택배 없이)가 (−13.1, −1.1, −19.3) mm 되게. set_mass_cg·set_sensors **다음에** 실행 | `python3 tools/vehicle/set_gripper.py` (떼기: `remove`) |

### tools/analysis/ — 비행 로그 분석
| 파일 | 역할 | 실행 |
|---|---|---|
| analyze_mission_log.py | **미션 비행 분석** (계획 경로 vs 추정 vs 실제, H·D·P 도착 오차, 기수 오차) | `python3 tools/analysis/analyze_mission_log.py 결과폴더 [로그경로]` |
| analyze_flow_log.py | 광류 호버 분석 (추정 위치 vs 실제 위치, 고도, 센서) | `python3 tools/analysis/analyze_flow_log.py 결과폴더 [로그경로]` |
| analyze_log.py | 호버 로그 분석 (모터 출력, 피치 토크) — 10/13 실험용 | `python3 tools/analysis/analyze_log.py [로그경로] [이름]` |
| plot_summary.py | THR_MDL_FAC별 피치 토크 요약 그래프 — 10/13 실험용 | `python3 tools/analysis/plot_summary.py` |
| analyze_hover.py | **호버·여유 시험 분석**: 호버 스로틀(PX4 추정), 모터 출력 앞/뒤, 피치 토크, 자세·위치 흔들림. `--margin`이면 여유 시험 전체(모터 최대·포화 시간·기울기·넘침)까지 | `python3 tools/analysis/analyze_hover.py 결과폴더 [--margin] [로그경로]` |
| sweep_table.py | weight_sweep.sh 결과를 표(sweep.txt)·그래프(sweep.png)로. 판정: 호버 ≤ 60 % 정상, ≤ 70 %·포화 ≤ 1 % 가능 | `python3 tools/analysis/sweep_table.py results/결과폴더` |
| attach_log.py | 결과 폴더에 비행 로그(flight.ulg)·PX4 파라미터 전체(params.txt)·기본값과 다른 것(params_changed.txt) 붙이기. ulg는 .gitignore라 올릴 때 `git add -f` | `python3 tools/analysis/attach_log.py results/폴더 ...` |
| check_overshoot.py | 여유 시험 옆 이동 넘침을 제어 반응 vs 위치 추정 오차로 나눔 | `python3 tools/analysis/check_overshoot.py results/폴더` |

## scripts — 자율 임무
| 파일 | 역할 | 실행 |
|---|---|---|
| mission_hdph.py | **미션 비행 H → D → P → H** (Offboard, 기수 회전, D·P에서 0.5 m 하강 5초 정지). H/D/P 위치는 auditorium_config.py에서 읽음 | `python3 scripts/mission_hdph.py` (run_mydrone_flow.sh로 켠 뒤) |
| hover_test.py | 호버 시험: 이륙 → 1.5 m에서 정지(기본 30 s) → 착륙 | `python3 scripts/hover_test.py [초]` |
| margin_test.py | **여유 시험**: 1.5 m 호버 15 s → 2.5 m 급상승 → 복귀 → 속도 1.5 m/s로 북 2 m 왕복 → 착륙 (모터 포화·기울기 확인) | `python3 scripts/margin_test.py` |
| tag_approach_test.py | **AprilTag 정밀 접근**: H → P(광류) → 태그 인식 → 북·동 25 cm 이동으로 카메라 방향 측정 → 태그 위로 반복 맞춤(1.5 m → 1.0 m) → H 착륙. 필요: set_camera.py, add_apriltag.py, `pip3 install pupil-apriltags opencv-python`, `apt install python3-gz-transport13 python3-gz-msgs10` | `python3 scripts/tag_approach_test.py 결과폴더` (`--view`: 카메라만 확인) |
| **mission_tag.py** | **미션 + 카메라 정밀 접근 (공설용)**: mission_hdph.py와 같은 동선 H → D → P → H, P에서만 아래쪽 카메라로 택배 AprilTag를 보고 1.5 m → 1.0 m 맞춤 → 0.5 m 하강(집기 흉내). 카메라→이동 방향은 기수로 계산 (25 cm 보정 이동 없음). 필요: tag_approach_test.py와 같음 | `python3 scripts/mission_tag.py 결과폴더` |
| camera_view.py | 아래쪽 카메라 화면을 따로 창으로 띄움 (태그 인식 테두리·거리 표시). Gazebo Image Display가 안 뜰 때, 녹화용 | `python3 scripts/camera_view.py` (끄기: q) |
| **mission_gripper.py** | **그리퍼 배달·수거 미션 (공설용)**: S에서 빈손 출발 → H 택배를 카메라(태그 id 1)로 찾아 집기 → D에 놓기 → P 택배(태그 id 0)를 찾아 집기 → H로 들고 와서 착륙. 끝나면 Gazebo 정답(쥔 위치, D·H 오차) 요약. 필요: set_gripper.py, 월드에 택배 2개 + 태그 | `python3 scripts/mission_gripper.py 결과폴더` |
| repeat_gripper.sh | 그리퍼 미션 자동 반복 (시뮬 켜기 → 미션 → shutdown → 분석), H 착륙 오차 요약 | `bash tools/repeat_gripper.sh 3 이름` |
| analysis/est_vs_true.py | 로그에서 드론이 믿는 위치 - 실제 위치를 시간별로 (어느 구간에서 틀어지는지) | `python3 tools/analysis/est_vs_true.py 로그.ulg 시작초 간격` |

## results — 실험 결과
| 폴더 | 내용 |
|---|---|
| 261013_hovering/ | 무게중심 보정 호버 실험 A~E (THR_MDL_FAC 0.75에서 피치 토크 ≈ 0) |
| 261031_mydrone_flow_hover_cg/ | 나무 마루 강당, 우리 기체 광류 호버 1.5 m. 호버 중 이동 2.6 cm, 광류 품질 218/255 |
| 261103_mission_final_5_vel08/ | **최종 미션** H→D→P→H (기수 수정 + 속도 제한 0.8 m/s). D 22 · P 19 · H 49 cm, 평균 추정 오차 26 cm, 기수 오차 2.3° 이하 |
| 261120_hover_2400g_* , 261120_hover_2600g_* | 무게·무게중심·모터별 호버. 같은 2,600 g에서 PX4 기본 모터 75.3 % → 실제 모터 56.7 % |
| 261120_mission_2600g_air2216/ | 2,600 g·실제 모터 미션. D 19 · P 8 · H 18 cm |
| 261120_tag_approach/ | AprilTag 정밀 접근 (카메라 가정값). 광류만 37.4 cm → 태그 맞춤 후 2.3 cm |
| 261120_실측모터_정리.md | 11/20 모터·무게·카메라 변경 정리 |
| 261121_2600g*, 261121_sweep_* | 2.6 kg 여유 시험, 무게 자동 시험 (실제 모터 / 최악 조건) |
| 261121_2600g_정리.md | **최대 무게**: 실제 모터 설계 목표(≤ 60 %) 약 2.7 kg·한계 약 3.0 kg / 최악 약 2.3 kg·약 2.7 kg, 바꾼 파라미터 표 |
| 261122_* , 261122_정리.md | 실측 무게중심(x −11.9 mm) 반영 호버·여유 시험 (관성 근사 / CAD 관성) |
| 261122_mission_2600g_air2216/ , _low/ | 무게중심 −11.9 mm 미션. 실제 모터 D 20 · P 1 · H 22 cm, 모터 최대 0.694 / 최악 조건 D 18 · P 6 · H 26 cm, 모터 최대 0.825, 포화 0 |
| 261122_tag_approach/ | 무게중심 −11.9 mm·실제 모터 AprilTag 접근. 광류만 40.3 cm → 태그 맞춤 후 1.9 cm |
| 설명.md | 결과 폴더 설명 |

## PX4 쪽에 연결된 것 (바로가기, 직접 수정 X)
`~/PX4-Autopilot/Tools/simulation/gz/` 안에 우리 파일을 가리키는 바로가기가 있다.
- models/x500_mydrone, models/x500_mydrone_base, models/auditorium_floor
- worlds/auditorium.sdf

## 자동 설정 (~/.bashrc)
| 줄 | 뜻 |
|---|---|
| `export PX4_PARAM_MIS_TAKEOFF_ALT=1.5` | 이륙 고도 1.5 m |
| `export PX4_PARAM_SIM_BAT_MIN_PCT=100` | 가상 배터리 무제한 (연습용. 실제 실험 때 배터리 사양으로 변경) |

## 추가 설치 (11/20~)
~~~bash
pip3 install --user --break-system-packages "mavsdk<4" pyulog matplotlib numpy pillow pupil-apriltags opencv-python
sudo apt install python3-gz-transport13 python3-gz-msgs10   # tag_approach_test.py (Gazebo 카메라 영상)
~~~

## 결과 올리는 규칙 (11/22~)
- 결과 폴더마다 `summary.txt`·그래프 + `attach_log.py`로 `params.txt`. 비행 로그(flight.ulg)는 **대표 비행만** 올리고 나머지는 랩실 PC에 보관 (저장소 용량)
- 무게·모터를 바꾼 시험은 `set_mass_cg.py`가 만든 `last_setting.txt`(또는 weight_sweep의 setting_무게g.txt)를 같이 남김
- 새 파일을 추가하면 이 README의 표와 변경 기록에 같이 적음

## 실행
~~~bash
# CMD: 정리 (결과가 비어 있어야 함)
pkill -9 -f "gz sim"; pkill -9 -f px4; sleep 2; pgrep -a gz

# SIM: 우리 기체 + 광류 (추천, 정리까지 자동)
bash ~/parcel-drone-sim/tools/run_mydrone_flow.sh

# SIM: 다른 실행 방법
PX4_SYS_AUTOSTART=4021 PX4_GZ_WORLD=auditorium PX4_SIM_MODEL=gz_x500_mydrone ./build/px4_sitl_default/bin/px4   # 우리 기체 + 광류 (GPS 끔)
PX4_SYS_AUTOSTART=4001 PX4_GZ_WORLD=auditorium PX4_SIM_MODEL=gz_x500_mydrone ./build/px4_sitl_default/bin/px4   # 우리 기체 + GPS
PX4_GZ_WORLD=auditorium make px4_sitl gz_x500_flow                                                                 # PX4 기본 광류 기체

# QGC
./QGroundControl-x86_64.AppImage

# 무게·무게중심·모터를 바꿔서 실행 (11/20~)
python3 tools/vehicle/set_mass_cg.py 2600 -11.9            # CMD: 설정 (모터 기본 = 실제 모터)
bash ~/parcel-drone-sim/tools/run_mydrone_flow_test.sh     # SIM: 원본 대신 이걸로
python3 scripts/margin_test.py                            # CMD: 여유 시험 → pxh> shutdown
python3 tools/analysis/analyze_hover.py 결과폴더 --margin  # CMD: 분석
bash tools/weight_sweep.sh 결과폴더 air2216 2600           # CMD: 위 과정을 자동으로 (시뮬 꺼진 상태)

# CMD: 미션 비행 → 끝나면 분석
python3 scripts/mission_hdph.py
python3 tools/analysis/analyze_mission_log.py 결과폴더이름 "로그경로.ulg"   # CMD: 미션 분석

# pxh>
commander takeoff      # 이륙 (1.5 m)
commander land         # 착륙 → "Landing detected" 확인
shutdown               # PX4 정상 종료 (로그 저장)
~~~

## 변경 기록
| 날짜 | 내용 |
|---|---|
| 10/13 | 저장소 생성. 우리 기체 모델(무게 2,283 g, 무게중심 x −17.7 mm), 호버 실험 A~E |
| 10/31 | 터미널 자동 생성, 강당 월드(30 × 20 m, 나무 마루), 동선 H→D 10 m→P 10 m→H, 광류 분석 도구, 우리 기체에 광류·거리계 추가 (앞 8 cm, 아래 2 cm) |
| 10/31 | IMU를 FC 위치로, 실행 스크립트 run_mydrone_flow.sh (PX4 센서 위치 설정 포함) |
| 10/31 | 실행 스크립트에 10/13 무게중심 보정값 고정 (CA_ROTOR 앞 0.192 / 뒤 −0.156, THR_MDL_FAC 0.75). 기체 번호 변경 시 초기화되던 문제 해결 |
| 10/31 | 실행 스크립트: PX4 종료 시 Gazebo 자동 종료 (메시지가 계속 찍히던 문제) |
| 10/31 | 미션 비행 스크립트 scripts/mission_hdph.py 추가 (MAVSDK 3.x, `mavsdk<4` 설치) |
| 10/31 | 미션 분석 도구 tools/analyze_mission_log.py 추가 |
| 10/31 | 실행 스크립트에 EKF2_MAG_TYPE=6 고정 (기본값 '자동'은 이륙 순간 기수가 튀어 경로가 옆으로 휘고 추락까지 발생) |
| 11/2 | 이륙 시 기수가 왼쪽으로 도는 원인 확정: 바닥·0.5 m 아래의 흐릿한 광류가 기수 추정을 튀게 함. 실행 스크립트에 SENS_FLOW_MINHGT=0.5 고정 → 이륙 후 기수 오차 11~26° → 0~6° |
| 11/2 | 나침반 설정 고정: EKF2_HEAD_NOISE=0.05, EKF2_MAG_NOISE=0.01, EKF2_MAG_DECL=-2.5 → 이륙 후 기수 오차 0.1~0.3° (시뮬레이션 전용 값, 실기체는 강당 자기장 확인 필요) |
| 11/2 | 기수 틀어짐 원인 수정 (SENS_FLOW_MINHGT 0.5, EKF2_HEAD_NOISE 0.05, MAG_NOISE 0.01, MAG_DECL −2.5), 미션 결과 기록 |
| 11/3 | 미션 회전 속도 10 → 30°/s (scripts/mission_hdph.py YAW_RATE) |
| 11/3 | 수평 속도를 PX4가 제한 (MPC_XY_VEL_MAX·MPC_XY_CRUISE 0.8, 미션 수평 이동 ramp=False) → 도착 전 감속, 고도 오차 해소. 최종 미션 5-3: D 22 · P 19 · H 49 cm |
| 11/4 | 결과 폴더 정리: 최종 미션(5-3)만 남기고 이전 미션 결과 삭제 (원본 로그는 바탕화면 드론 시뮬/logs에 보관). set_cg.py 삭제 |
| 11/4 | tools 폴더 분류: world/(월드), vehicle/(무게중심), analysis/(로그 분석). run_mydrone_flow.sh·auditorium_config.py는 tools/ 바로 아래 유지 |
| 11/20 | (김수민) 모터를 실제 모터 AIR2216 II KV920 + T1045 II 공식 추력표로: k 8.549e-6 → 1.230e-5, 최대 1000 → 954.8 rad/s, 합계 3.49 → 4.57 kgf. set_mass_cg.py(무게·무게중심·모터 한 번에), run_mydrone_flow_test.sh, hover_test.py, analyze_hover.py, attach_log.py |
| 11/20 | (김수민) 아래쪽 카메라(set_camera.py, IMX219 가정), AprilTag(add_apriltag.py), 정밀 접근(tag_approach_test.py): 37.4 cm → 2.3 cm |
| 11/21 | (김수민) 여유 시험(margin_test.py), 무게 자동 시험(weight_sweep.sh, sweep_table.py), 최악 조건 모터(air2216_low), check_overshoot.py. 최대 무게 실제 모터 약 2.7 kg(≤ 60 %)·약 3.0 kg(≤ 70 %) |
| 11/22 | (김수민) 실제 모터 확인 → 문서 "가정 모터" → "실제 모터". set_mass_cg.py: 기본 모터 air2216, 실행 스크립트 주석 자동 갱신, BAT1_CAPACITY 5200 추가, --inertia·--arm 옵션. 실측 무게중심 x −11.9 mm 반영 (무게는 2,600 g 유지). README 정리 |
| 11/22 | (김수민) set_sensors.py 추가: 광류·거리계·카메라 위치를 한 줄로 바꾸고 PX4 센서 위치 파라미터(Y·Z, EKF2_MIN_RNG 포함) 자동 계산. set_mass_cg.py가 센서 위치를 모델에서 읽음 (예전엔 광류 x 80 mm 고정) |
| 11/22 | (김수민) 무게중심 −11.9 mm로 미션 비행(실제 모터·최악 조건)과 AprilTag 접근 다시 시험 → 11/20과 같은 수준, 최악 조건 미션도 포화 없이 완주 |
| 11/25 | (김수민) 그리퍼 Gazebo에 넣음 (set_gripper.py, models/x500_mydrone/meshes/ 9개, 선배 11/24 STEP 변환), 기체 모양 실제 CAD로, 착륙 = 그리퍼 다리 끝. 월드에 H 택배(parcel_H) 추가, 동선 선배 변경 H→D 5 m · D→P 5 m · P→H √50 m. mission_gripper.py (들고 출발 → 놓기 → 카메라로 찾아 집기 → 들고 귀환). mission_tag.py 0.5 m 맞춤 추가(11/25: 34.8 → 1.2 cm), camera_view.py |
| 11/26 | (김수민) 미션 변경: S(H 뒤 1 m)에서 빈손 출발 → H 택배(태그 id 1) 인식·집기 → D 놓기 → P 택배(태그 id 0) 인식·집기 → H 착륙. 택배 90° 돌려 놓고 손가락이 긴 쪽(0.25 m)을 잡음, 기수 회전 없음. 집기: 1.5·1.0·0.5 m 카메라 맞춤 → 0.13 m까지 카메라 보며 하강 → 손가락 닫고 가운데로 끼워 맞춘 뒤 고정. 태그로 기수 오차(약 5°) 재서 다음 경로 보정. 손가락 충돌 추가, 거리계 가림·태그 놓침 안전장치. 결과(맞춤1·확인1): 쥔 위치 ±1 cm, D 8~10 cm, H 9~19 cm. 태그 그림 파일 이름을 태그별로(tag0/tag1), 카메라 가까운 면 0.003 m. tools/repeat_gripper.sh(자동 반복), tools/analysis/est_vs_true.py(추정-실제 오차 구간별) |
| 11/24 | (김수민) mission_tag.py 추가: 미션 동선 그대로 날다가 P에서 카메라로 태그 인식 → 정밀 접근 → 집기 고도까지 하강 (tag_approach_test.py는 H → P 단독 시험으로 유지) |
| 11/24 | (김수민) 선배 확정 센서 위치 반영 (그리퍼가 하판 아래를 가려서): 광류·거리계 → M3(앞-왼쪽) 모터 아래 (169, 175, −5)·(189, 175, −5) mm, 카메라 → 그리퍼 하우징 아래 (0, 0, −144) mm, IMX219 120° 렌즈(가로 90°). 무게중심 택배 2호 포함 (−12.1, −1.0, −34.8) mm (z 처음 반영, 이전 −49.6). set_mass_cg.py: --cg-y·--cg-z·--ground 옵션, CA_ROTOR_PY·EKF2_IMU_POS_Y/Z도 무게중심 기준 계산. set_camera.py 기본값 변경 |
