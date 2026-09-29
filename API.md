# MotionCue-R API 설명서

앱 개발자를 위한 참조 문서입니다. 웹앱(`motioncue_app.html` + `asym_engine.js`)과 펌웨어(`app.c`)가 실제로 주고받는 **통신 프로토콜, 데이터 형식, 설정값, 알고리즘 파라미터, 상태 규칙**을 소스 기준으로 정리했습니다. 네이티브 앱은 이 문서의 규약을 그대로 구현하면 같은 펌웨어와 호환됩니다.

| 항목 | 값 |
|---|---|
| 문서 기준일 | 2026-09-29 |
| 펌웨어 | `at02vib0` (Device Information 펌웨어 문자열) |
| 검출 엔진 | `asym_engine.js` `ENGINE_VER='A7'` |
| 웹앱 | `motioncue_app.html` (연결 카드 3링크, 진동 펄스 설정 포함) |
| 원본 소스 | GitHub Pages 저장소의 `motioncue_app.html`, `asym_engine.js`; 펌웨어 `Simplicity_Studio_Project/bt_soc_empty_freertos/app.c` |

표기 규칙: 바이트는 16진수(`0x14`), 다바이트 정수는 **리틀엔디언(LE)**, 각도는 도(°), 각속도는 dps(°/s), 가속도는 g, 시각은 ms.

---

## 1. 시스템 구성

### 1.1 모듈과 역할
| 부위 | 역할 | 앱이 구독하는 특성 | 비고 |
|---|---|---|---|
| 몸통 (T) | 각도 센서. 자세(오일러·쿼터니언)와 원시 IMU를 50 Hz로 스트리밍 | Orientation, IMU Data, Status, Battery, Device Information | 비대칭 검출은 이 모듈 데이터만 사용 |
| 왼다리 (L) | 진동자극기 | Control(쓰기), Status, Battery | 센서값 불필요. 연결 직후 `0x13` identify 버즈 |
| 오른다리 (R) | 진동자극기 | Control(쓰기), Status, Battery | 위와 같음 |

- 세 모듈은 **동일 펌웨어**를 탑재하며 각각 폰(중앙장치)에 직접 연결됩니다(허브 없음). 모듈은 자기 부위를 모릅니다.
- 부위 배정은 **앱에만** 저장됩니다(§10.1 `mc.devices.v1`). 앱은 저장된 장치 ID로 각 모듈을 다시 찾아 연결합니다.
- 장치명은 `MotionCue-XXXX`(XXXX = BLE 주소 하위 2바이트 대문자 16진수, 예 `MotionCue-0194`)로 세 모듈이 모두 같은 접두어를 씁니다. 부위는 이름으로 구분할 수 없습니다.

### 1.2 사용 흐름
1. 연결: 몸통 → 왼다리 → 오른다리 순으로 페어링(최초 1회, 이후 자동 연결).
2. 러닝 시작: 3 초 서기(영점·자이로 bias) → 15 초 창 평가 반복.
3. 비대칭 검출(기준선 편차 또는 절대 임계 초과가 같은 방향으로 2창 연속) → 설정한 쪽 다리 모듈에 진동 세트(기본 15 s on / 5 s off × 9회) → 5분 휴식 → 평가 재개.
4. 정지 → 세션 캡처(JSON) 저장 가능. 로그 탭은 연결 중 모든 샘플과 이벤트를 기록해 `mclog/1` 파일로 내보냅니다.

---

## 2. BLE 프로토콜

### 2.1 광고·연결
| 항목 | 값 |
|---|---|
| 광고 | Legacy, general discoverable, connectable. 인터벌 100 ms(160 × 0.625 ms). 광고 데이터는 스택 자동 생성(Flags + 장치명). 제조사 데이터 없음 |
| 장치명 | `MotionCue-XXXX` (GAP Device Name 특성에도 기록, 최대 20자) |
| 스캔 필터 | 이름 접두어 `MotionCue` |
| 연결 파라미터 | 연결 직후 펌웨어가 요청: 인터벌 **7.5~15 ms**(6~12 × 1.25 ms), latency 0, supervision timeout 5000 ms. 중앙장치가 거절해도 동작하지만 30 ms 이상이면 자세+IMU 알림 100개/s가 링크에 밀려 약 1 s 지연·32/30 Hz 수신이 관측됨 |
| 동시 연결 | 모듈당 중앙장치 1개. 연결 중에는 광고를 멈추고, 끊기면(전원 ON 상태) 재광고 |
| MTU | 기본값으로 충분(가장 긴 알림 14 B) |
| 페어링/암호화 | 없음(모든 특성 open) |

### 2.2 GATT 서비스·특성
| UUID | 이름 | 속성 | 길이 | 내용 |
|---|---|---|---|---|
| `e8a1f000-9c3b-4f2a-8b7d-1a2b3c4d5e6f` | MotionCue 서비스 | | | 아래 4개 특성을 포함 |
| `e8a1f001-9c3b-4f2a-8b7d-1a2b3c4d5e6f` | IMU Data | notify | 12 B | 원시 6축(§2.4). 구독 시에만 전송, ≈50 Hz |
| `e8a1f002-9c3b-4f2a-8b7d-1a2b3c4d5e6f` | Control | write, write-without-response | **최대 4 B** | 제어 명령(§2.5) |
| `e8a1f003-9c3b-4f2a-8b7d-1a2b3c4d5e6f` | Status | read, notify | **9 B** | 상태(§2.6). 값이 바뀔 때만 notify |
| `e8a1f004-9c3b-4f2a-8b7d-1a2b3c4d5e6f` | Orientation | notify | 14 B | 오일러 + 쿼터니언(§2.3), ≈50 Hz |
| Battery Service `0x180F` / `0x2A19` | Battery Level | read, notify | 1 B | 잔량 0~100 %. 약 5 s 주기 갱신(값이 바뀔 때 notify) |
| Device Information `0x180A` | `0x2A29` 제조사, `0x2A24` 모델, `0x2A26` 펌웨어 | read | 문자열 | 펌웨어 = `IMU_STREAM_FW_TAG`(8자, 현재 `at02vib0`) |

각 특성의 CCCD(0x2902)에 notify(0x0001)를 쓰면 구독됩니다. 펌웨어는 Orientation, IMU Data, Status, Battery 각각의 구독 상태를 따로 기억하고 구독 중일 때만 notify 합니다.

### 2.3 Orientation 프레임 (`e8a1f004`, 14 B)
| 오프셋 | 형식 | 내용 | 변환 |
|---|---|---|---|
| 0 | int16 LE | roll (centi-deg) | ÷100 → ° |
| 2 | int16 LE | pitch (centi-deg) | ÷100 → ° |
| 4 | int16 LE | yaw (centi-deg) | ÷100 → ° |
| 6 | int16 LE | q w (Q14) | ÷16384 |
| 8 | int16 LE | q x (Q14) | ÷16384 |
| 10 | int16 LE | q y (Q14) | ÷16384 |
| 12 | int16 LE | q z (Q14) | ÷16384 |

- 쿼터니언은 **센서 → 월드** 회전, 성분 순서 w,x,y,z. 오일러는 이 쿼터니언에서 계산(roll = atan2(2(wx+yz), 1−2(x²+y²)), pitch = asin(2(wy−zx)), yaw = atan2(2(wz+xy), 1−2(y²+z²))).
- **중력-up 벡터(센서 좌표계)** = Rᵀ·ẑ:
  `up = normalize([ 2(xz − wy),  2(yz + wx),  1 − 2(x² + y²) ])`
  이 규약이 펌웨어 `gpred()`와 앱 `gravityUp()`에서 동일하게 쓰입니다. 부호를 바꾸면 lean 부호가 뒤집힙니다.
- 앱은 프레임마다 `performance.now()`(도착 시각, ms)를 타임스탬프로 붙입니다. 펌웨어 프레임에는 시각·시퀀스가 없습니다.
- 마운트 보정(`0x12`)은 융합 입력(가속도·자이로)을 보드 Z축 기준 90° 단위로 회전시키므로 오일러·쿼터니언·3D 표시에만 영향을 주고, IMU Data 원시값에는 적용되지 않습니다.
- 사람기준 roll(**lean**) 계산 절차(앱):
  1. 서기 시점(`leanZero()` 호출 다음 프레임)의 up 을 `leanUp0` 로 저장.
  2. 매 프레임 `lean = leanAbout(leanUp0, up, leanAxis)` : 두 up 벡터를 전방축 `leanAxis`(기본 Z=[0,0,1], 설정 `sLeanAxis`)에 수직인 평면에 투영한 뒤, 그 축 둘레의 부호 있는 회전각(atan2(cross·axis, dot))을 도 단위로 반환.
  3. 드리프트가 없고 마운트 회전에 불변(yaw·pitch 무관)이며, 지표 `roll` 은 오일러 roll 이 아니라 이 lean 을 씁니다.

### 2.4 IMU Data 프레임 (`e8a1f001`, 12 B)
| 오프셋 | 형식 | 내용 | 스케일 |
|---|---|---|---|
| 0,2,4 | int16 LE ×3 | gx, gy, gz | × **0.07 dps/LSB** (풀스케일 ±2000 dps) |
| 6,8,10 | int16 LE ×3 | ax, ay, az | × **0.000488 g/LSB** (풀스케일 ±16 g) |

- 센서 프레임 원시값: **자이로 bias 미차감, 마운트 회전 미적용**. 앱이 서기 구간에서 bias 를 추정해 뺍니다(§5.5).
- 센서 ODR 104 Hz, 펌웨어 루프 20 ms(50 Hz)에서 최신 샘플을 전송(같은 루프에서 Orientation 다음에 IMU Data 순).
- 수직축 각속도(지표 `yaw_rate`) = `dot(g − bias, up_acc)`; `up_acc` 는 분석 창의 **가속도 평균 방향**(정규화). 양수 = 위에서 볼 때 반시계(왼쪽으로 도는) 회전.

### 2.5 Control 명령 (`e8a1f002`, 쓰기 ≤ 4 B)
첫 바이트 = opcode. 펌웨어는 opcode 별 **최소 길이** 미만이면 무시합니다(1바이트 명령 허용). 알 수 없는 opcode 는 조용히 무시되므로(오류 응답 없음) 지원 여부는 Status 길이로 판별합니다(§2.6).

| opcode | 길이 | 인자 | 동작 | 앱 사용처 |
|---|---|---|---|---|
| `0x01` | 2 | 0/1 | 안테나 내장/외장 | (개발) |
| `0x02` | 3 | int16 LE deci-dBm | TX 파워 | (개발) |
| `0x03` | 2 | 0~100 | 진동 세기 **설정값**(NVM3 저장). 재영점 ACK·identify 버즈의 진폭. 진동 중이면 즉시 반영 | 미사용(기본 20) |
| `0x04` / `0x05` | 3 | u16 LE (×1000) | Mahony Kp / Ki | (개발) |
| `0x06` | 1 | | 자이로 바이어스 재보정(정지 상태 약 1 s) | 개발 탭 |
| `0x07` | 1 | | 자세 리셋(쿼터니언 단위) | 개발 탭 |
| `0x08` | 1 | | 융합 설정 NVM3 저장 | 설정 탭 |
| `0x09` | 2 | 0~3 | 융합 알고리즘 0 complementary / 1 mahony / 2 madgwick / 3 eskf | 설정 `sAlgo` 변경 시 |
| `0x0A` / `0x0C` | 7 / 25 | | (4 B 한계로 BLE 에서는 도달 불가) | — |
| `0x0B` | 2 | 0/1 | ZUPT(정지 검출 기반 보정) on/off | 연결 직후 스위치 상태 동기 |
| `0x0D` | 2 | 0/1 | 정지 yaw 앵커-락 on/off | 연결 직후 동기 |
| `0x0E` | 1~2 | 0 full / 1 heading | **재영점 요청**. full = roll/pitch/yaw 기준 재설정, heading = yaw 만. 정지가 감지되면 적용되고 500 ms ACK 버즈 | 러닝 시작 서기 단계에서 `0x0E 0x01`(표시용, 완료를 기다리지 않음) |
| `0x0F` | 1 | | 재영점 해제(절대 자세 복귀) | 설정 탭 |
| `0x10` | 2 | pct 0~100 | **(레거시) 연속 진동** = `0x14 [pct, 0, 100]`. E-Stop 중 무시 | 구형 펌웨어 폴백, 개발 탭 |
| `0x11` | 2 | 0/1 | **E-Stop** 해제(0)/발동(1). 발동 = 즉시 정지 + 래치. 해제해도 다음 진동 명령까지 off | 시작 시 `0x11 0x00`, 긴급정지 버튼 `0x11 0x01` |
| `0x12` | 2 | 0~3 | 마운트 보정 Z축 0° / +90° / 180° / −90° (NVM3 저장) + 자세 리셋 | 설정 `sMount` 변경 시 |
| `0x13` | 1 | | **identify**: 유휴(진동 명령 없음)·비E-Stop 일 때 300 ms 버즈 | 다리 모듈 연결 직후 |
| `0x14` | 4 | pct, Hz, duty | **진동 설정·시작**(§8.1) | 자극 on/off |

`0x14` 인자 규칙(펌웨어가 스냅):
- `pct` 0~100 (100 초과는 100). 10 kHz PWM 듀티 = 진폭.
- `Hz` 0 = 연속. 1~255 는 `((Hz+5)/10)*10` 으로 10 단위 반올림 후 10~100 으로 클램프.
- `duty` 0 = off. 1~99 는 `((duty+10)/20)*20` 으로 20 단위 반올림(1~9 → 20), 100 이상 = 연속.
- `pct == 0` 또는 `duty == 0` 이면 정지. `Hz == 0` 또는 스냅 결과 duty 100 이면 연속.
- 예: `14 32 0A 14` = 50 %, 10 Hz, 20 % → 100 ms 주기 중 20 ms ON. `14 32 64 14` = 50 %, 100 Hz, 20 % → 10 ms 주기 중 2 ms ON. `14 00 00 00` = 정지.

### 2.6 Status (`e8a1f003`, 9 B)
| byte | 내용 | 값 |
|---|---|---|
| 0 | 진동 세기 설정값(`0x03`) | 0~100 |
| 1 | 진동 **명령** on/off | 1 = `pct > 0` 인 명령이 살아 있음. 펄스의 순간 ON/OFF 가 아님 |
| 2 | 안테나 | 0 내장 / 1 외장 / 0xFF 미설정 |
| 3:4 | TX 파워 | int16 LE deci-dBm |
| 5 | 재영점 상태 | 0 idle / 1 정지 대기 / 2 완료(기준 적용 중) |
| 6 | E-Stop 래치 | 1 / 0 |
| 7 | 펄스 주파수 Hz | 0 = 연속 또는 off |
| 8 | 펄스 듀티 % | 100 연속, 0 off |

- 구독 직후 **한 번 read** 하면 현재 E-Stop·펄스 지원 여부를 즉시 알 수 있습니다(앱 동작).
- **capability 규칙**: Status 길이 ≥ 9 → `0x14`/`0x13` 지원 펌웨어. 7 → E-Stop 바이트까지만(펄스 없음). 6 → 구형(E-Stop 바이트도 없음). 구형에는 `0x10` 으로 연속 진동만 보냅니다.
- 구형(6 B) E-Stop 추론: 앱이 진동을 명령한 지 500 ms 가 지났는데 byte1 이 0 이면 물리 버튼 E-Stop 으로 간주.
- 어느 모듈에서든 E-Stop 이 관측되면 앱은 **모든 링크에 `0x11 0x01`** 을 보내고 러닝·수동 모드를 정지합니다.

### 2.7 펌웨어 동작 규칙(앱이 전제하는 것)
| 항목 | 규칙 |
|---|---|
| 스트리밍 | 전원 ON·IMU 정상일 때 20 ms 루프. 루프마다 Orientation → IMU Data 순으로 notify(각각 구독 시). 배터리는 250 루프(≈5 s)마다 측정 |
| 부팅 | 자이로 bias 자가보정(정지 가정, 256샘플 ≈1 s) → 융합 초기화. NVM3 에서 진동 세기·마운트·융합 설정 복원 |
| 진동 엔진 | `0x10`/`0x14`/`0x13`/`0x11` 은 모두 진동 엔진(sleeptimer 기반)이 처리. 진동 명령마다 **최대 ON 워치독 130 s** 가 재시작되어 off 명령이 안 와도 자동 정지. 진동 중 EM1 유지 |
| 재영점 ACK | `0x0E` 완료 시 500 ms 버즈(세기 = byte0 설정값, 0 이면 60 %). 진동 명령 중·E-Stop 중에는 생략 |
| 버튼(PA05) | 짧게(50 ms~2 s): 진동 중이면 **E-Stop**, 아니면 재영점(full) 요청. 2 s 길게: 소프트 전원 토글 |
| 소프트오프 | 광고·연결 종료, IMU 전원 off, 진동 off, **E-Stop 래치 해제**, LED off. 소프트온 시 진동은 재개되지 않음 |
| 링크 끊김 | 진동 즉시 off(E-Stop 래치는 유지), 모든 구독 해제, 재광고. 비정상 끊김은 supervision 5 s 후 인지 |
| 이름/식별 | 장치명은 주소 기반. 부위 정보 없음 |
| NVM3 키 | 0x0001 진동 세기, 0x0002 Mahony 게인(미사용), 0x0003 융합 설정, 0x0004 마운트 |

---

## 3. 앱의 연결 관리 (Link)

앱은 부위마다 `Link` 객체(T/L/R)를 두고 같은 절차로 연결합니다. 네이티브 앱도 아래 순서를 지키면 됩니다.

### 3.1 연결 절차(역할별)
| 단계 | 몸통(T) | 다리(L/R) |
|---|---|---|
| 1 | GATT 연결 → 서비스 `e8a1f000` | 동일 |
| 2 | Control 특성 확보(필수) | 동일 |
| 3 | Orientation, Status, IMU Data 특성 확보(모두 필수, 없으면 실패) | Status 만 시도(없어도 됨). Orientation/IMU 는 요청하지 않음 |
| 4 | Orientation notify 구독 | — |
| 5 | Status notify 구독 + **read 1회** → `onStatus` | 동일 |
| 6 | IMU Data notify 구독(실패해도 계속: yaw_rate 는 오일러 폴백, 러닝 판정은 지표 기반) | — |
| 7 | Battery 구독 + read | 동일 |
| 8 | Device Information 읽기(표시) | — |
| 9 | 쓰기 큐 초기화, `connected=true`, `onConn` | 동일 |
| 10 | `0x0B`, `0x0D` 로 UI 스위치 상태 동기(기본 둘 다 1), 기준선 로드 | `0x13` identify 버즈 |

### 3.2 등록·저장·재연결
- **최초 등록**: 부위 버튼 → 장치 선택(이름 접두어 `MotionCue`) → `mc.devices.v1[role] = {id, name}` 저장 → 연결. 선택한 장치가 이미 다른 부위로 저장돼 있으면 재지정 확인 후 이전 부위에서 제거.
- **저장된 모듈 연결**: `navigator.bluetooth.getDevices()` 로 id 일치 장치를 찾아 선택창 없이 연결. 실패 시 `watchAdvertisements` 로 5 s 광고 대기 후 재시도. API 미지원 브라우저는 선택창(같은 장치가 아니면 재지정 확인). 네이티브 앱은 저장된 MAC/ID 로 직접 연결하면 됩니다.
- **자동 재연결**: 의도치 않은 끊김이면 1.5 s 간격 최대 12회 재시도(사용자가 해제한 경우 제외).
- **쓰기 큐**: Control 쓰기는 링크별 **순차 큐**로 직렬화(GATT 동시 쓰기 금지). 미연결이면 명령을 버리고 로그만 남김.
- **끊김 처리**: 몸통이 끊기면 러닝·수동 모드 정지(연결된 다리 모터 off 명령 전송). 다리가 끊기면 그 다리가 현재 자극 대상일 때만 자극 중단 후 대기 상태로, 세션은 계속.
- 로거 이벤트: `ble {action:'connect'|'disconnect', role, name|userDisc}`.

### 3.3 연결 직후 명령 시퀀스 예(몸통)
```
CCCD(ori)=01 00 → CCCD(status)=01 00 → read(status) → CCCD(imu)=01 00 → CCCD(batt)=01 00 → read(batt)
write ctrl: 0B 01      (ZUPT on)
write ctrl: 0D 01      (yaw 락 on)
러닝 시작: 11 00 (E-Stop 해제) → 0E 01 (heading 재영점, 서기) → … 자극: [L] 14 3C 32 64 → 15 s 후 [L] 14 00 00 00
```

---

## 4. 신호 입력 규약(앱 내부)

| 항목 | 규약 |
|---|---|
| 타임스탬프 | 알림 **도착 시각**(`performance.now()`, ms, 단조 증가). 기기 시각 없음 |
| 명목 주기 | 20 ms(50 Hz). 실제 수신 주기 `dEff` 는 창 안 도착 간격의 **중앙값**을 [20, 60] ms 로 클램프해 추정(느린 링크 대응) |
| 버퍼 | 자세 버퍼 `{t, roll, yaw, lean, up[3]}` 와 원시 버퍼 `{t, g[3] dps, a[3] g}` 를 각각 **45 s** 유지(오프라인 재생은 무제한) |
| 도착 지터 보정 | de-burst: 뒤에서부터 `t̂_k = min(t_k, t̂_{k+1} − dEff)` (평균 간격이 명목의 90 % 미만이면 평균 사용). 창 안 데이터를 50 Hz 균일 격자로 **선형 보간**(외삽 없음), 간격 > 3·dEff 위의 격자점은 `flagged` |
| 지표 | `yaw_rate`(기본): 원시 자이로(bias 차감)를 창 평균 가속도 방향에 투영한 dps. 원시 버퍼가 1 s 이상 없으면 오일러 yaw 차분 ÷ dEff 로 폴백(`srcYaw:'euler'`). `roll`: 사람기준 lean(°) |
| bias | 서기 3 s 구간의 자이로 평균(그 구간 std(|a|) < 0.05 g 일 때만 채택, 아니면 이전 값 유지) |
| up 벡터 | 분석은 가속도 평균 up(마운트·재영점 무관). 쿼터니언 up 과의 각도차 `upMismatchDeg` 는 진단값(경고만) |
| 단위 | 각도 °, 각속도 dps, 가속도 g, 주파수 Hz, 시간 ms(내부)·s(설정) |

---

## 5. 검출 엔진 (asym_engine.js, A7)

순수 함수 라이브러리(`window.MC`). DOM·BLE·시계에 접근하지 않으며 시각은 인자로 받습니다. 네이티브 이식 시 이 절의 순서와 상수를 그대로 옮기면 웹앱과 같은 판정이 나옵니다(Python 미러 `tests/asym_mirror.py` 로 교차검증됨).

### 5.1 GATE 상수 (`MC.GATE`)
| 키 | 기본값 | 의미 |
|---|---|---|
| `fsGrid` | 50 | 리샘플 격자 Hz |
| `winSec` | 15 | 기본 분석 창(s). 앱 설정 `sWin` |
| `minSpanFrac` | 0.98 | 창 안 데이터 시간범위가 이 비율 미만 → `insufficient_duration` |
| `maxGapMs` | 250 | 도착 결손 허용(초과 → `sample_gap`) |
| `staleMs` | 250 | 마지막 샘플이 이보다 오래됨 → `stale_data`(실시간 분석에만) |
| `maxAbsDeg` / `maxAbsDps` | 60 / 1000 | 물리 범위 초과 샘플 제거. 제거가 5 % 초과 → `sample_invalid` |
| `refJumpDeg` / `refJumpMinSamples` | 30 / 10 | 각도 지표의 레벨 시프트(영점 변경) → `reference_jump` |
| `cadMin` / `cadMax` | 0.7 / 4.0 Hz | 지표 ACF 주기 탐색 대역(스트라이드) |
| `peakTol` | 0.03 | ACF 피크 선택 허용(최고 피크 높이 − 0.03 이상인 **가장 짧은 랙** 선택) |
| `minStrength` | 0.5 | 지표 주기성(정규화 ACF 피크 높이) 하한 → 미만이면 `weak_periodicity` |
| `minAngleStd` / `minRateStd` | 1.0 ° / 5 dps | 추세제거 신호 RMS 하한 → 미만이면 `low_amplitude` |
| `lobeK` | 0.15 | 로브 진입 임계 = max(lobeK·std, floor) |
| `lobeFloorDeg` / `lobeFloorDps` | 0.5 / 2 | 로브 진입 임계 하한 |
| `lobeExitFrac` | 0.5 | 로브 이탈 = 진입 임계 × 0.5(히스테리시스) |
| `minLobeFrac` / `minLobeSamples` | 0.08 / 3 | 로브 최소 길이 = max(3, round(0.08·fs/f_pre)) 샘플(짧으면 스파이크로 제거) |
| `flatRatio` | 1.15 | 극값 < 1.15 × 로브 평균 절대값 → `flat`(극값 불명확) |
| `minLobeRel` | 0.3 | 극값 < 0.3 × 중앙 극값 → `weak` |
| `lobeCountLo` / `lobeCountHi` | 0.5 / 1.5 | 로브 수가 기대치(2·win·f)의 0.5~1.5배 밖 → `lobe_count` |
| `pairMin` / `pairMax` | 0.2 / 0.8 | 짝 조건: 반대부호 인접 로브 극값 간격 ∈ [0.2, 0.8]·T |
| `minPairs` | 6 | 짝 수 미만 → `data_gaps`(결손 기인) 또는 `unclear_alternation` |
| `siCapPct` | 60 | \|SI\| 상한 초과 → `unclear_alternation` |
| `trimK` / `trimMinPairs` | 1 / 8 | 짝 ≥ 8 이면 부호별 상·하위 1개씩 절사 평균, 아니면 중앙값 |
| `tDisagree` | 0.30 | 스트라이드 후보(acc·metric·lobe)가 중앙값에서 30 % 넘게 이탈 → outlier |
| `turnMeanDps` / `turnSegDps` / `turnSegSec` | 15 / 25 / 2 | (yaw_rate) 창 평균 \|mean\| > 15 또는 2 s 구간(1 s hop) 평균 > 25 → `turn_contamination` |
| `rampRatio` | 2.0 | 창 앞 ⅓ 과 뒤 ⅓ 의 std 비 > 2 → `amplitude_ramp` |
| `eMinFundFrac` | 0.35 | 고조파비 계산 시 기본파 진폭이 0.35·std 미만이면 E = null |
| `stationaryMaxFrac` | 0.2 | 1 s 구간 중 정지(std\|a\| < 0.05 g) 비율 > 0.2 → `not_running/stationary` |
| `acc.stepMin` / `acc.stepMax` | 1.5 / 5.0 Hz | \|a\| 스텝 케이던스 대역(90~300 spm) |
| `acc.enterStdG` / `acc.exitStdG` | 0.20 / 0.15 g | 블록 러닝 판정 std\|a\| 임계(직전 창이 러닝이면 exit 값 사용) |
| `acc.minStrength` | 0.35 | \|a\| ACF 주기성 하한 |
| `acc.blockSec` / `acc.hopSec` / `acc.blockFrac` | 3 / 1 / 0.6 | 3 s 블록을 1 s 간격으로 평가, 통과 블록 ≥ 60 % → 러닝 |
| `acc.stationaryStdG` | 0.05 g | 정지 판정·bias 추정 허용 std |
| `acc.enterSec` / `acc.exitSec` | 2 / 3 s | 라이브 표시용 히스테리시스(연속 통과/실패 시간) |
| `acc.subHarm` | 0.5 | \|a\| ACF 에서 선택 랙의 절반 위치에 높이 ≥ 0.5×인 피크가 있으면 그것을 스텝으로(스텝/스트라이드 2배 모호 해소) |
| `standSec` | 3 | 서기(영점·bias) 시간 |
| `baseline.minWindows` | 8 | 기준선 저장에 필요한 ok 창 수 |
| `baseline.floorPct` / `baseline.kSigma` | 5 / 2 | 개입 임계 = max(2σ, 5 %p) |
| `baseline.consecutive` | 2 | 같은 방향 초과 연속 창 수 |
| `baseline.offsetWindows` | 3 | 세션 첫 3개 ok 창이 모두 같은 방향 초과 → `baseline_mismatch` 경고 |
| `baseline.staleDays` | 49 | 기준선 재측정 권고(7주) |
| `baseline.binSpm` | 15 | 케이던스 구간 폭(spm) |

`MC.gateWith(override)` 는 GATE 위에 오버라이드를 얹은 사본을 만듭니다(`acc`, `baseline` 은 한 단계 깊게 병합). 앱의 설정 탭 "게이트 튜닝"이 이를 `mc.gate.override.v1` 로 저장합니다(§10.7).

### 5.2 분석 파이프라인 (`AsymEngine.analyzeRange(metric, t0, t1, tNow)`)
창 `[t0, t1]`(ms)의 지표 신호를 만들고 순서대로 게이트를 통과시킵니다. 어느 단계에서든 실패하면 그 사유로 즉시 반환합니다.

**A. 창 신호 생성(`windowSig`)** — 실패 상태는 모두 `hold`
1. 원천 샘플 수집: `[t0−2dt, t1+2dt]` (dt = 20 ms). 4개 미만 → `insufficient_duration`.
2. 정합성: 비유한값·\|v\| > maxAbs 제거. 제거율 > 5 % 또는 4개 미만 → `sample_invalid`.
3. 신선도(tNow 가 주어진 실시간 분석만): `tNow − 마지막 샘플 > 250 ms` → `stale_data`.
4. `dEff` 추정 → de-burst → 시간범위 `span < 0.98·win` → `insufficient_duration`.
5. 결손: 최대 간격 > 250 ms → `sample_gap`. `coverage = 1 − Σ_{gap>1.5·dEff}(gap − dEff) / win`.
6. 50 Hz 격자 리샘플(격자 시작 `tGrid0 = max(t0, t̂₀)`, 점 수 n < 20 → `insufficient_duration`).
7. (각도 지표) 레벨 시프트 검사 → `reference_jump`.
8. `meanOffset = mean(y)`, `sig = detrend(y)`(선형 추세 제거) — 이후 모든 게이트는 `sig` 기준(선회 게이트만 추세제거 전 `y`).

**B. 판정(`analyzeRange`)**
| 순서 | 단계 | 실패 상태/사유 |
|---|---|---|
| 1 | `upMismatchDeg` 계산(진단) | — |
| 2 | (yaw_rate) 선회 게이트 `turnGate(y)` | `hold/turn_contamination` |
| 3 | `std = RMS(sig)` | — |
| 4 | 램프 게이트 `rampGate(sig)` | `hold/amplitude_ramp` |
| 5 | 원시 버퍼가 살아 있으면(`runSrc='acc'`): 정지 비율 `stats()` | `not_running/stationary` |
| 6 | 창 러닝 판정 `windowRunning()` → `running`, `stepSpm`, `accStdG`, `strideHz = stepSpm/120` | `not_running/not_running` |
| 7 | 진폭 하한 `std < minRateStd(또는 minAngleStd)` | `info/low_amplitude` |
| 8 | 지표 ACF `acfPeriod(sig, 0.7~4 Hz)` → `metricHz`, `strength` | `info/weak_periodicity` (f 없음 또는 strength < 0.5) |
| 9 | 원시 버퍼 없으면 `running=true, runSrc='metric'` | — |
| 10 | 로브 추출 `lobes(sig, gate)`; gate = max(0.15·std, floor); 최소 길이는 **지표 ACF 주기** 기준 | — |
| 11 | 스트라이드 교차검증 `resolveStride({acc, metric, lobe})` → 중앙값 `strideHz`, `strideSrc='resolved:*'` | `hold/period_ambiguous` (후보 간 전부 불일치) |
| 12 | \|a\| 케이던스가 outlier 면 `stepSpm = 120·strideHz`, `stepSrc='derived'` | — |
| 13 | 로브 수 `nLobes ∉ [0.5, 1.5]·(2·win·f)` | `hold/lobe_count` |
| 14 | 짝짓기 `pairLobes(kept, T=fs/f, 0.2~0.8)` → `nPairs`; 6 미만이면 coverage < 0.9 또는 `gapped` 로브 ≥ 2 | `hold/data_gaps`, 아니면 `info/unclear_alternation` |
| 15 | 절사 대표값 `trimmedSI` → 이상 로브 있으면 0레벨 보정 `zeroShift` → `signedSI = (L − R)/(L + R)·100` | — |
| 16 | 고조파비 `E = 100·A(2f)/A(f)` (Hann Goertzel, 진단) | — |
| 17 | 로브 교대 주기가 outlier | `info/period_ambiguous` |
| 18 | `si > 60` | `info/unclear_alternation` |
| 19 | 성공 | `ok` |

`analyze(winSec, metric, _, tNow)` 는 버퍼 마지막 샘플 시각 `tLast` 기준 `[tLast − win, tLast]` 를 분석합니다(앱의 실시간 창).

### 5.3 상태·사유 표
| state | 의미 | 앱 동작 |
|---|---|---|
| `ok` | 유효한 SI | 기준선/절대 임계 편차 → 판정기(§7.3) |
| `hold` | 입력 품질·선회 등으로 판정 보류 | 대기(`sWait`) 후 재평가. 판정기 카운터 유지 |
| `not_running` | 러닝 아님 | 대기 후 재평가. 판정기 카운터 **리셋** |
| `info` | 러닝이지만 SI 를 신뢰할 수 없음 | 대기 후 재평가. 카운터 유지 |

| reason | state | 뜻(앱 표시 문구) |
|---|---|---|
| `insufficient_duration` | hold | 창 미충족(시간 부족) |
| `stale_data` | hold | 수신 지연(최근 샘플 없음) |
| `sample_gap` | hold | 수신 결손 |
| `sample_invalid` | hold | 샘플 이상 |
| `reference_jump` | hold | 기준 급변(영점 변경?) |
| `turn_contamination` | hold | 선회 중 |
| `amplitude_ramp` | hold | 진폭 급변 |
| `period_ambiguous` | hold 또는 info | 주기 불명확 |
| `lobe_count` | hold | 로브 수 비정상 |
| `data_gaps` | hold | 결손으로 피크 손실 |
| `stationary` | not_running | 정지 구간 포함 |
| `not_running` | not_running | 러닝 아님 |
| `low_amplitude` | info | 흔들림 작음(안정) |
| `weak_periodicity` | info | 주기성 약함 |
| `unclear_alternation` | info | 좌우 교대 불명확 |
| `no_raw_data` | (예약) | IMU raw 미수신 |
| `baseline_mismatch` | (판정기 플래그) | 기준선 불일치 |

### 5.4 결과 객체 R
| 필드 | 형식 | 내용 |
|---|---|---|
| `state`, `reason` | string | §5.3 |
| `metric`, `t0`, `t1`, `winSec` | | 분석 창 |
| `running`, `runSrc` | bool, `'acc'`/`'metric'` | 러닝 판정과 근거 |
| `accStdG`, `stationaryFrac` | number | 창 \|a\| std(블록 중앙값), 정지 비율 |
| `stepSpm`, `stepSrc` | number, `'acc'`/`'derived'` | 스텝 케이던스(분당 걸음) |
| `strideHz`, `strideSrc`, `strideCands` | number, string, `{acc, metric, lobe}` | 스트라이드 주파수(교차검증 결과와 후보) |
| `metricHz`, `strength` | number | 지표 ACF 주기·주기성(0~1) |
| `std`, `meanOffset` | number | 추세제거 신호 RMS, 창 평균(자세 편향; roll 지표에서 의미) |
| `rampRatio`, `turnMeanDps`, `turnSegMaxDps` | number | 게이트 진단값 |
| `gate` | number | 로브 진입 임계 |
| `nLobes`, `nPairs` | int | 로브 수(short 제외), 짝 수 |
| `meanL`, `meanR` | number | 양(+)/음(−) 로브 대표 진폭(0레벨 보정 후) |
| `signedSI`, `si`, `side` | number, number, `'좌'`/`'우'` | 부호 있는 SI(%), 절대값, 부호 규약상 큰 쪽(`signedSI ≥ 0` → 좌) |
| `trimmedCount`, `outlierFlag`, `zeroShift`, `siMethod` | | 절사 개수(4·trimK 또는 0), 이상 피크 존재, 0레벨 보정량, `'trim'`/`'median'` |
| `E` | number 또는 null | 고조파 진폭비 %(진단) |
| `coverage`, `gapMaxMs`, `devHz` | number | 결손 지표, 실제 수신률(1000/dEff) |
| `srcYaw`, `upMismatchDeg` | `'raw'`/`'euler'`/null, number | yaw_rate 원천, up 불일치 각 |
| `peaks[]` | `{i, t, v, sign, role, start, end}` | 로브 목록. role: `paired`, `unpaired`, `edge`, `short`, `flat`, `weak`, `gapped` |
| `sig`, `tGrid0`, `fs` | Float64Array, ms, Hz | 분석에 쓴 격자 신호 |
| `engine` | `'A7'` | |
| (레거시) `ok`, `cadence`, `harmE`, `asymPct` | | `state==='ok'`, `strideHz`, `E‖0`, `si‖0` |

### 5.5 러닝 판정과 스텝 케이던스 (`RawEngine`)
- `accNormSamples`: \|a\| = √(ax²+ay²+az²) (g).
- `stepCadence(t0,t1)`: \|a\| 를 격자화·평균 제거 → `stdG = RMS` → 정규화 ACF 를 1.5~5 Hz 에서 탐색(포물선 보간, `subHarm` 규칙) → `{fStep, strength, stdG}`.
- `windowRunning(t0,t1)`: 3 s 블록을 1 s 간격으로 평가. 블록 통과 = `stdG > thr` **and** `fStep` 존재 **and** `strength > 0.35` (thr = 0.20 g, 직전 창이 러닝이면 0.15 g). 통과 비율 ≥ 0.6 → 러닝. `stepSpm` = 통과 블록 `60·fStep` 의 중앙값, `stdG` = 블록 std 의 중앙값.
- `stats(t0,t1)`: 1 s 구간마다 std\|a\| < 0.05 g 이면 정지 → `stationaryFrac`.
- `estimateBias(t0,t1)`: 구간 자이로 평균. `ok = std|a| < 0.05 g`.
- `tick(tNow)`(라이브 표시, 1 s 마다): 최근 3 s 블록으로 통과/실패를 판단하고 2 s 연속 통과 → running, 3 s 연속 실패 → 해제. 연결 카드 "러닝감지" 표시에만 사용(판정에는 `windowRunning`).

### 5.6 로브·짝·SI 상세
1. **로브 추출** `lobes(sig, g)`: \|x\| > g 에서 진입, 같은 부호이고 \|x\| ≥ 0.5·g 인 동안 유지(부호 반전이나 이탈 임계 미만이면 종료). 로브당 극값 1개(최대 \|x\|). 규칙 순서: ① 동부호 인접 로브 병합(M자 딥) ② `short`(< minLen) 제거 후 재병합 ③ 극값이 결손 보간점이면 `gapped` ④ `flat`(극값 < 1.15 × 로브 평균 \|x\|) ⑤ `weak`(극값 < 0.3 × 중앙 극값) ⑥ 남은 후보의 양 끝은 `edge`. 역할이 없는 로브만 짝 후보(`kept`).
2. **로브 교대 주기** `lobeAlternationT`: 한 로브 건너 같은 부호 로브의 극값 간격 중앙값(간격 3개 미만이면 null).
3. **스트라이드 교차검증** `resolveStride`: 후보 {acc: stepSpm/120, metric: 지표 ACF, lobe: fs/T_lobe} 의 중앙값 f. 각 후보가 \|c − f\|/f > 0.30 이면 outlier. 후보 ≥ 2 이고 하나 빼고 전부 outlier → ambiguous.
4. **짝짓기** `pairLobes`: 후보를 시간순으로 보며 인접 두 로브가 반대 부호이고 극값 간격 ∈ [0.2, 0.8]·T 이면 짝(pos, neg). 짝은 겹치지 않음.
5. **절사 대표값** `trimmedSI`: 짝 ≥ 8 → 부호별로 정렬 후 상·하위 1개씩 제외한 평균(`trimmedCount = 4`), 아니면 중앙값. 제외(또는 전체)된 로브 중 대표값의 ×1.5 초과·×0.5 미만인 것이 `outliers`(→ `outlierFlag`).
6. **0레벨 보정** `zeroShiftFromOutliers`: 이상 로브의 초과 면적이 평균 제거를 밀어낸 만큼 되돌림. `L' = L − shift`, `R' = R + shift`.
7. **SI**: `signedSI = (L' − R')/(L' + R')·100`, `si = |signedSI|`, `side = signedSI ≥ 0 ? '좌' : '우'`. 양(+) 로브 = 좌 규약은 실측 전 잠정이며 앱 설정 `sSignFlip`(표시 반전)으로 교정합니다.
8. **고조파비** `E`: Hann 창 Goertzel 로 f 와 2f 진폭을 구해 `100·A(2f)/A(f)`. 기본파 진폭이 0.35·std 미만이면 null. 표시·기록만(판정 미사용).

### 5.7 스냅샷 (`AsymEngine.snapshot(metric, {showT0, showT1, aT0, aT1})`)
표시 구간 `[showT0, showT1]` 의 격자 신호(평균 오프셋 제거)와 분석 구간 `[aT0, aT1]` 의 R 을 함께 반환: `{metric, srcYaw, R, disp:{t0, y[], dt}, span, aStart, aStop, peaks:[{…, tRel}], gate}`. 게이트에서 조기 종료돼 로브가 없으면 표시용으로 예비 주기로 로브·짝만 다시 계산합니다.

---

## 6. 러닝 자동 중재 상태기계 (`proto`)

### 6.1 상태 전이
| 상태 | 진입 시 동작 | 지속 | 다음 |
|---|---|---|---|
| `idle` | 진동 off, 세션 캡처 종료·저장 | — | 시작 버튼 → `stand` |
| `stand` (준비·서기) | E-Stop 해제(`0x11 0x00`, 시작 시), `0x0E 0x01` 전송, lean 영점(`leanZero`), 세션 캡처 시작(3 s 사전구간 포함) | `standSec` = 3 s | 자이로 bias 추정(§4) → `detect` |
| `detect` (평가) | 시작 마커 기록 | `sWin` s | `evalAsym` |
| `evalAsym` | `analyze(sWin)` → 로그·캡처에 창 기록 → 편차·판정기 | 즉시 | ok 가 아니면 `wait`; 개입이면 `stim`; 아니면 `wait` |
| `wait` (대기) | 진동 off | `sWait` s | `detect` |
| `stim` (중재) | `vibSide` 결정(§8.2). 대상 미연결이면 로그 후 `wait` | 세트 × (`sOn` 진동 + `sOff` 휴지) | 세트 완료 → `pause` |
| `pause` (평가 중지) | 진동 off | `sPause` s | `detect` |

- 타이머는 200 ms 틱으로 남은 시간을 표시하며, 정지 버튼은 어느 상태에서든 `idle` 로 갑니다.
- 기준 측정 모드(`mode:'baseline'`, §7.1)는 `detect → evalBaseline → detect` 를 대기 없이 반복하고 진동을 내지 않습니다.
- 개입 판정 만료 시간(판정기 `expireMs`) = `(2·sWin + sWait)·1000` ms.

### 6.2 evalAsym 상세
1. `r = analyze(sWin, metric)`.
2. 표시: 케이던스(`stepSpm`, `strideHz`), 러닝 여부·std, 고조파비, 평균 편향, 짝 수·절사·이상피크.
3. `dev = (r.state==='ok') ? (baseline ? baselineDeviation(baseline, r) : absoluteDeviation(r, sThr)) : null`.
4. `d = decider.feed(r, dev, now, expireMs)`.
5. CSV 로그 행 추가(§10.5), 로거 `window` 이벤트, 세션 캡처 `windows[]` 에 창 기록.
6. `state ≠ ok` → 상태줄에 사유 표시, `wait`.
7. `ok`: SI 와 편차 표시(게이지는 좌=왼쪽 채움, 우=오른쪽). `d.mismatch` 첫 발생 시 기준선 불일치 경고.
8. `d.action==='intervene'` → `side = LR(d.sign)` → `stim`; 아니면 `wait`(초과 카운트 표시).

`LR(sign)`: `left = (sign > 0) !== sSignFlip` → `'좌'` 아니면 `'우'`.

---

## 7. 기준선(baseline)과 판정

### 7.1 기준 측정
- 러닝 탭 "기준 측정" 시작 → `mode='baseline'` 으로 상태기계 실행. `ok` 창의 `signedSI`(+ `E`, `stepSpm`, `meanOffset`)만 `BaselineStats` 에 누적. `minWindows`(8) 이상이면 정지 시 저장.
- 저장 키: `mc.baseline.v1.<장치명>.<metric>` (localStorage). 몸통 모듈 이름·지표별로 따로 보관.

### 7.2 기준선 JSON (schema 1)
| 필드 | 내용 |
|---|---|
| `schema` | 1 |
| `metric` | `'yaw_rate'` 또는 `'roll'` |
| `forwardAxis` | 전방축 인덱스(0 X, 1 Y, 2 Z) |
| `signFlip` | 측정 당시 표시 반전 여부 |
| `deviceId` | 몸통 장치명 |
| `engineVer` | `'A7'` |
| `winSec` | 창 길이(가져오기 시 현재 설정과 같아야 함) |
| `binSpm` | 15 |
| `date` | ISO 문자열 |
| `n`, `mu`, `sigma` | ok 창 수, signedSI 평균·표준편차(모집단) |
| `eMu`, `eSigma` | 고조파비 평균·표준편차 |
| `spmMean` | 평균 케이던스 |
| `offMu` | 평균 편향 평균 |
| `bins` | `{"165": {n, mu, sigma}, …}` — 케이던스 15 spm 구간별 통계 |

가져오기 검증(`validateBaseline`): schema 1, `n ≥ 1`, `mu`/`sigma` 유한수, (선택) 지표·창 길이 일치. 49일이 지나면 재측정 권고(`baselineIsStale`).

### 7.3 편차와 판정기
- `baselineDeviation(base, R)`: 창 케이던스 구간의 `bins[k]` 가 `n ≥ 4` 이면 그 구간의 μ/σ, 아니면 전체. `thr = max(2σ, 5)`, `delta = signedSI − μ`, `z = delta/σ`, `exceed = |delta| > thr`.
- 기준선이 없으면 `absoluteDeviation(R, sThr)`: μ = 0, `exceed = |signedSI| ≥ sThr`.
- `Decider.feed(R, dev, t, expireMs)`:
  - `not_running` → 카운터 리셋. `hold`/`info` → 카운터 유지.
  - 마지막 ok 창 이후 `expireMs` 경과 → 리셋.
  - 초과이고 부호가 직전과 같으면 `count++`, 다르면 `count = 1`; 미초과면 리셋.
  - `count ≥ 2` → `{action:'intervene', sign, delta}` 후 리셋.
  - 세션 첫 3개 ok 창이 모두 같은 방향으로 초과 → `mismatch = true`(경고만, 개입 규칙 동일).

---

## 8. 진동 자극 제어

### 8.1 파라미터
| 파라미터 | 범위 | 설정 id | 기본 | 의미 |
|---|---|---|---|---|
| 강도 pct | 0~100 % | `sInt` (수동 `mInt`) | 60 | 10 kHz PWM 듀티 = 진폭 |
| 펄스 주파수 | 10~100 Hz, 10 단위 | `sFreq` (`mFreq`) | 50 | 진동을 켜고 끄는 포락선 주파수. 0 = 연속 |
| 펄스 듀티 | off / 20 / 40 / 60 / 80 / 100 % | `sDuty` (`mDuty`) | 100 | 주기 중 ON 비율. 100 = 연속(구형과 동일), off = 자극 없음 |
| 진동 On / 휴지 Off | 1~60 s / 1~60 s | `sOn` / `sOff` | 15 / 5 | 세트 안의 on/off 시간 |
| 세트 반복 | 1~30 | `sSets` | 9 | |
| 중재 후 평가 중지 | 10~1800 s | `sPause` | 300 | |
| 진동 부위 | `large` / `small` | `sSide` | large | 비대칭 큰 쪽 / 작은 쪽 다리 |

### 8.2 라우팅 규칙
1. 개입 부호 `sign` → `side = LR(sign)`(좌/우).
2. `sSide === 'large'` 이면 `vibSide = side`, `'small'` 이면 반대쪽.
3. `SIDE_ROLE = {좌:['L'], 우:['R'], 양쪽:['L','R']}` 로 대상 링크 결정.
4. 대상 링크가 연결돼 있으면 그 링크에, 미연결이면
   - `sStimTorso`(설정 "몸통 모듈로 대체 자극", 기본 off)가 켜져 있고 몸통이 연결돼 있으면 몸통 링크에(경고 로그),
   - 아니면 **자극 생략**(경고 로그). 러닝 모드에서는 세트를 시작하지 않고 `wait` 로 감(5분 `pause` 로 가지 않음).
5. 명령: on = `Link.vib(pct, freq, duty)`, off = `Link.vib(0, 0, 0)`. `wait`/`pause`/정지/수동 정지 시에는 **연결된 모든 링크**에 off.

### 8.3 `Link.vib(pct, freq, duty)`
1. 클램프: pct 0~100, freq 0~100, duty 0~100(미지정 100). `duty == 0` 이면 `pct = 0`.
2. `estopActive && pct > 0` → 무시.
3. 링크 `motorCmd = pct`(on 시작 시각 기록: 구형 E-Stop 추론용).
4. `caps.pulse`(Status ≥ 9 B) 이면 `[0x14, pct, freq, duty]`, 아니면 `[0x10, pct]`(펄스 요청인데 구형이면 1회 경고).

### 8.4 E-Stop
- `estop(from)`: `estopActive = true`, 모든 링크 `motorCmd = 0`, 연결된 링크 전부에 `0x11 0x01`, 러닝·수동 정지, 배너 표시.
- 발생원: 앱 긴급정지 버튼, 어느 모듈의 Status byte6 = 1(또는 구형 추론).
- `clearEstop()`: 연결된 링크 전부에 `0x11 0x00`, `estopActive = false`, 배너 닫기. 러닝 시작·기준 측정 시작·수동 시작 때 항상 먼저 호출.

### 8.5 수동 모드
| id | 기본 | 범위 |
|---|---|---|
| `mOn` | 30 s | 1~120 |
| `mOff` | 10 s | 1~120 |
| `mMin` | 5 분 | 1~60 |
| `mInt` / `mFreq` / `mDuty` | 60 / 50 / 100 | §8.1 |
| `mSide` | 양쪽 | 왼다리 / 오른다리 / 양쪽 |

시작 조건: 어느 링크든 연결. `mMin` 동안 (`mOn` 진동 → `mOff` 휴지) 반복. "누르는 동안 진동" 버튼은 누름/뗌에 on/off. 대상 링크가 없으면 상태줄에 "대상 없음" 표시.

### 8.6 안전 규칙 요약
- 펌웨어 워치독 130 s: 앱의 최대 on 구간(수동 120 s, 자동 60 s)보다 길게 잡혀 있으므로 정상 사용에서는 개입하지 않음. 앱은 on 구간을 이보다 길게 만들면 안 됨.
- 앱이 백그라운드로 가서 타이머가 멈추면 off 를 못 보낼 수 있음(웹앱은 화면 유지 wake lock 사용). 네이티브 앱은 포그라운드 서비스 등으로 타이머를 보장할 것.
- 링크 끊김 시 펌웨어가 스스로 정지하므로 앱은 재연결 후 상태만 복구하면 됨.

---

## 9. 설정 항목 전체

설정 탭(`#p-set`) 입력의 id·기본값·범위. 모두 `mc.settings.v1` 에 저장·복원됩니다(§10.1). 복원 시 기기로는 아무것도 보내지 않습니다(`sMount`, `sAlgo` 포함).

| id | 기본 | 범위/선택지 | 의미 | 영향 |
|---|---|---|---|---|
| `sWin` | 15 | 3~40 s | 검출 창 | 분석·판정기 만료·기준선 창 길이 검증 |
| `sOn` | 15 | 1~60 s | 세트 내 진동 시간 | 상태기계 `stim` |
| `sOff` | 5 | 1~60 s | 세트 내 휴지 | |
| `sSets` | 9 | 1~30 | 세트 반복 | |
| `sPause` | 300 | 10~1800 s | 중재 후 평가 중지 | |
| `sWait` | 10 | 5~300 s | 검출 실패/미검출 시 대기 | 판정기 만료 시간에도 포함 |
| `sInt` | 60 | 0~100 % | 진동 세기 | `0x14` pct |
| `sFreq` | 50 | 10~100 Hz(10 단위) | 펄스 주파수 | `0x14` Hz |
| `sDuty` | 100 | 0/20/40/60/80/100 | 펄스 듀티 | `0x14` duty |
| `sMetric` | `yaw_rate` | `yaw_rate` / `roll` | 비대칭 지표 | 엔진 metric, 기준선 키 |
| `sSide` | `large` | `large` / `small` | 진동 부위(큰 쪽/작은 쪽) | §8.2 |
| `sThr` | 15 | 0~60 % | 절대 임계(기준선 없을 때) | `absoluteDeviation` |
| `sLeanAxis` | 2 | 0 X / 1 Y / 2 Z | 전방축(lean 회전축) | `leanAxis`, 변경 시 lean 영점 |
| `sSignFlip` | false | | 좌/우 표시 반전 | `LR()`, 스냅샷 표시 |
| `sStimTorso` | false | | 다리 미연결 시 몸통 모터로 대체 | §8.2 |
| `sMount` | 0 | 0/1/2/3 | 마운트 보정(오일러·3D 용) | 변경 시 `0x12` 전송 |
| `sAlgo` | 1 (mahony) | 0~3 | 융합 알고리즘 | 변경 시 `0x09` 전송(표시 `dvAlgo` 는 앱 선택값) |
| `swZupt` / `swYaw` | on / on | 스위치(div, 저장 안 됨) | ZUPT / yaw 앵커-락 | 토글 시·연결 직후 `0x0B`/`0x0D` |
| 게이트 튜닝 카드 | — | `TUNE_PARAMS` | GATE 오버라이드 | `mc.gate.override.v1`(§10.7) |

기타 UI 설정: 실시간 차트 y 범위 `mc.ui.chRange`(lean 1~180°, rate 10~2000 dps 단계, auto 여부), 3D 뷰의 USB 위치 `usbPos`(표시용, 기본 `+x`).

---

## 10. 저장소와 데이터 형식

### 10.1 localStorage 키
| 키 | 내용 |
|---|---|
| `mc.settings.v1` | `{id: value}` — 설정 탭 input/select 값(체크박스는 bool). `tune*` 제외 |
| `mc.devices.v1` | `{T:{id,name}, L:{id,name}, R:{id,name}}` — 부위별 저장 모듈(`BluetoothDevice.id`, 표시 이름) |
| `mc.baseline.v1.<device>.<metric>` | 기준선 JSON(§7.2) |
| `mc.gate.override.v1` | GATE 오버라이드 객체(§10.7) |
| `mc.ui.chRange` | `{lean, rate, auto}` 차트 범위 |

모든 접근은 try/catch(사생활 모드·차단 시에도 동작). 웹앱은 origin 별로 분리되므로 PC/폰 간 공유는 JSON 내보내기/가져오기로 합니다.

### 10.2 로거 IndexedDB `mc_logger` (v1)
| 스토어 | 레코드 |
|---|---|
| `chunks` | `{kind:'ori'|'raw', t0, t1, n, w(8 또는 6), wall(Date.now), origin(performance.timeOrigin), t:Float64Array(n), v:Float32Array(n·w)}` — 5 s 마다 flush. ori 값 순서 roll,pitch,yaw,lean,qw,qx,qy,qz / raw gx,gy,gz,ax,ay,az |
| `events` | `{t(performance.now), wall, origin, kind, data}` — 기록 중이 아니어도 항상 저장(페이지를 다시 열어도 남음) |

내보내기 시 다른 페이지 로드(origin 이 다름)의 레코드는 현재 페이지 시간축으로 옮기고(`t + origin − 현재 origin`), 이벤트는 wall 순으로 정렬합니다.

### 10.3 로그 내보내기 `mclog/1` (gzip JSON)
파일명 `mclog_<몸통이름>_<yyyymmdd-HHMM>.json.gz`.
| 필드 | 내용 |
|---|---|
| `schema` | `'mclog/1'` |
| `exportedAt` | ISO |
| `app` | `{build, engine, ua, url, perfOrigin}` |
| `device` | `{name: 몸통, L: 왼다리 이름 또는 null, R: …}` |
| `settings` | `LG.settingsSnapshot()` = 설정 탭 `{id: value}` |
| `gateOverride` | 현재 게이트 오버라이드 |
| `leanAxis`, `signFlip`, `bias`, `metric` | 당시 값 |
| `oriCols` | `['t','roll','pitch','yaw','lean','qw','qx','qy','qz']` |
| `rawCols` | `['t','gx','gy','gz','ax','ay','az']` |
| `events[]` | `{t, wall, kind, data}` (아래 표) |
| `segments[]` | `{t0, t1, ori:[[t,…9]], raw:[[t,…7]]}` — 청크를 2 s 이내 간격으로 이어 붙인 연속 구간. t 는 0.1 ms, 각도 3자리, 쿼터니언·raw 4자리 |
| `captures[]` | 세션 캡처 요약 `{id, date, metric, winSec, marks, windows, expected, durSec, note, mode}` (샘플 제외) |

이벤트 kind 와 data:
| kind | data |
|---|---|
| `app` | `{action:'load', build, engine, ua, url, perfOrigin}` |
| `logger` | `{action:'start', ua, build, engine, url, perfOrigin, connected, device, legs:{L,R}}` / `{action:'stop', nOri, nRaw}` / `{action:'cleared'}` |
| `ble` | `{action:'connect', role, name}` / `{action:'disconnect', role, userDisc}` |
| `hz` | `{o:[5개], i:[5개]}` — 5 s 마다 1초 수신 개수(자세, raw) |
| `state` | `{sid:'rState'|'mState', s, sub}` — 상태줄 변경 |
| `window` | 창 결과 행(§10.5 와 같은 필드: t, t0, t1, state, reason, running, cadence, metric, method, si, signedSI, harmE, side, nPairs, trimmed, coverage, stepSpm, accRunning, baselineMu, delta, z, devHz) |
| `bias` | `{bias:[3], stdG}` |
| `motor` | `{link:'T'|'L'|'R', side?, pct, freq, duty}` |
| `capture` | 세션 캡처 요약 |
| `setting` | `{id, value}` |
| `gate` | `{over}` 게이트 오버라이드 적용 |
| `tag` | `{label, action:'start'|'end'}` 활동 태그(서기/걷기/러닝/빠른 러닝/제자리 러닝/턴/탭·충격/계단/기타) |
| `note` | `{text}` |
| `log` | `{m, c}` 화면 로그 줄(c: `tx`/`rx`/`er`) |
| `error` | `{msg, src?, line?}` |
| `visibility` | `{state}` |

### 10.4 캡처 JSON (schema 1)
세션 캡처(러닝/기준 측정 시작→정지, "💾 JSON 저장"), 창 JSON, 로그 분할(`logtool.py split`)이 같은 형식입니다.
| 필드 | 내용 |
|---|---|
| `schema` | 1 |
| `kind` | `'session'` / `'window'` / `'log'` |
| `id`, `engine`, `date`, `device`, `metric`, `winSec`, `leanAxis`, `signFlip`, `bias`, `note`, `mode`(`run`/`baseline`/`log`) | 메타 |
| `marks` | `{start, detect, stop}` (ms, 캡처 시간축). start 는 러닝 시작 시각, 사전구간 3 s 가 그 앞에 있음 |
| `windows[]` | `{t0, t1, state, reason, si, signedSI, strideHz, nPairs, E, coverage, stepSpm, running, devHz, mode}` — 앱이 실시간으로 낸 창 판정 |
| `ori[]` | `[t, roll, yaw, lean, [ux,uy,uz]]` |
| `raw[]` | `[t, gx, gy, gz, ax, ay, az]` |
| `expected` | 정지 직전 마지막 창의 R 요약(`state, reason, si, signedSI, strideHz, nPairs, nLobes, E, coverage, stepSpm, meanOffset, std, running, runSrc, srcYaw`) |
| `durSec` | 길이 |

재생 규칙: 같은 `t0/t1` 로 `analyzeRange` 를 돌리면 실시간 결과와 일치해야 합니다(셀프테스트 X 섹션·미러가 검증).

### 10.5 비대칭 로그 CSV (열 순서)
`time_iso, running, cadence_hz, metric, method, si_pct, harm_e, side, asym_pct, state, reason, n_pairs, signed_si, trimmed, e_pct, baseline_mu, delta, z, acc_running, step_spm, coverage, dev_hz`
(`method` = 엔진 버전, `asym_pct` = `si_pct` 와 동일(호환), 빈 값 = 해당 없음)

### 10.6 기준선 JSON 파일
`baseline_<device>_<metric>_<yyyy-mm-dd>.json` = §7.2 객체 그대로. 가져오기는 현재 지표·창 길이와 일치해야 합니다.

### 10.7 게이트 오버라이드 (`mc.gate.override.v1`)
`{winSec, acc:{enterStdG, …}, stationaryMaxFrac, …}` 처럼 GATE 와 같은 경로. 튜닝 카드가 노출하는 파라미터:
`winSec, acc.enterStdG, acc.exitStdG, acc.minStrength, acc.blockFrac, acc.stepMin, acc.stepMax, stationaryMaxFrac, minStrength, minRateStd, minAngleStd, rampRatio, turnMeanDps, turnSegDps, lobeK, minPairs, pairMin, pairMax, siCapPct`.
"적용"은 라이브 엔진에 즉시 반영, "저장"은 다음 로드에서도 적용.

---

## 11. 화면 표시 규칙(참고)
| 표시 | 규칙 |
|---|---|
| 수신 Hz | 1 s 마다 Orientation/IMU 알림 개수(정상 ≈50/50) |
| 러닝감지(라이브) | `RawEngine.tick` 1 s 마다: 러닝 ✓/비러닝, std g, spm, 주기성 |
| 상태줄 | `rState`(러닝) / `mState`(수동): 상태 + 부제(사유·세트·부위·파라미터) |
| 비대칭 게이지 | `si/2` %(최대 50 %), 좌는 왼쪽 채움 |
| 스냅샷 | 세션 전체 개요(창 판정 밴드) 또는 창별 파형 + 로브 극값(짝 파랑/주황, 그 외 회색) |
| 연결 카드 | 부위별 행: 이름, 상태(연결됨·배터리·펄스 지원/구형), 버튼 |
| 헤더 | `몸통✓ 왼– 오른–` 요약, 몸통 배터리 |

---

## 12. 개발·검증 도구
| 도구 | 용도 |
|---|---|
| `motioncue_app.html#demo` | 합성 세션 캡처로 캡처·스냅샷·튜닝 경로 점검(BLE 없음) |
| `#sim` / `#logsim` | 합성 50 Hz 알림으로 러닝 시작→창 2개→정지, 다리 링크 스텁으로 `0x14` 라우팅 검사(`document.title` 에 `stim=ok`), 로거 export/import 왕복 |
| `tests/asym_selftest.html` | 엔진 셀프테스트(합성·실측 골든, 88 케이스) |
| `tests/asym_mirror.py` | 엔진 Python 미러: `--selftest`, `--replay <capture.json> --hop 5`(앱 기록 창과 재생 비교) |
| `tests/logtool.py` | `info / windows / events / split` (mclog → 캡처 JSON) |
| `tests/make_real_vectors.py` | 실측 골든 갱신 |

---

## 13. 네이티브 앱 구현 체크리스트
1. **스캔·연결**: 이름 접두어 `MotionCue`. 부위별로 사용자에게 선택받아 ID 저장. 연결 후 §3.1 순서로 구독(다리는 Orientation/IMU 를 요청하지 않음). 연결 파라미터 요청은 펌웨어가 하므로 앱은 수용만.
2. **디코딩**: §2.3/§2.4 스케일. 쿼터니언 → up 공식의 부호를 그대로 사용. 각 프레임에 도착 시각을 붙임.
3. **lean/yaw_rate**: 서기 시 up 기준 저장 → `leanAbout`; 창 평균 가속도 방향 → `yawRate`. bias 는 서기 3 s 평균(정지 조건).
4. **버퍼·리샘플**: 45 s 링버퍼, `dEff` 추정, de-burst, 50 Hz 선형 보간, flagged.
5. **엔진**: §5.1 상수와 §5.2 순서를 그대로. 결과는 `state/reason` 계약. 실측 골든(`tests/real/*.json`)으로 같은 결과가 나오는지 확인.
6. **상태기계·판정기**: §6, §7.3. 만료 시간 `(2·win + wait)`. 기준선 JSON 은 §7.2 형식으로 호환 유지.
7. **자극**: §8. 대상 미연결 시 생략·경고. `0x14` 는 Status ≥ 9 B 일 때만, 아니면 `0x10`. off 는 모든 링크에. 시작 전 `0x11 0x00`.
8. **E-Stop**: 어느 모듈의 Status byte6 이든 1 이면 전체 정지 + 모든 링크 `0x11 0x01`.
9. **끊김**: 몸통 끊김 = 세션 정지, 다리 끊김 = 대상이면 자극 중단·세션 계속. 자동 재연결.
10. **로그**: 가능하면 `mclog/1` 과 캡처 JSON 을 그대로 써서 PC 도구(`logtool.py`, 미러, 셀프테스트)와 호환.
11. **백그라운드**: 타이머가 멈추지 않게(포그라운드 서비스). 펌웨어 워치독 130 s 는 최후 보호.

---

## 부록 A. 단위·부호 규약 요약
| 항목 | 규약 |
|---|---|
| 쿼터니언 | w,x,y,z, 센서→월드, Q14 |
| up(센서좌표) | `[2(xz−wy), 2(yz+wx), 1−2(x²+y²)]` 정규화 |
| lean | 전방축 둘레 회전각, 서기 기준 0 |
| yaw_rate | `dot(g − bias, up_acc)`, 양수 = 위에서 볼 때 반시계 |
| signedSI | `(L−R)/(L+R)·100`, 양(+) 로브 = 좌(잠정, `sSignFlip` 로 반전) |
| 시간 | 앱 도착 시각 ms(performance.now), 캡처/로그도 같은 축. 벽시계는 `wall`(Date.now) |

## 부록 B. 예시 바이트
| 목적 | 쓰기(hex) |
|---|---|
| E-Stop 해제 | `11 00` |
| heading 재영점 | `0E 01` |
| 연속 진동 60 % | `14 3C 00 64` (구형: `10 3C`) |
| 60 %, 50 Hz, 40 % 듀티 | `14 3C 32 28` |
| 정지 | `14 00 00 00` |
| identify 버즈 | `13` |
| 마운트 +90° | `12 01` |
| ZUPT off | `0B 00` |
