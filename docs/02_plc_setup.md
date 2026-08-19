# Mitsubishi 공정 PLC 연계 설정

## 연결 방식

Jetson은 Linux MC Protocol client이고 공정 PLC가 TCP server다. Windows 전용
MX Component는 사용하지 않는다. PLC CPU device에 로봇 데이터를 mirror하는
ladder와 MC Open Setting은 PLC 담당자가 준비해야 한다.

## PLC 담당자에게 받을 계약

| 항목 | 필요한 값 |
|---|---|
| CPU/Ethernet | 정확한 형명과 GX Works 버전 |
| 네트워크 | PLC IP, mask, 허용 Jetson IP |
| MC | TCP port, 3E/4E, binary/ASCII, remote password |
| 읽기 | 로봇별 pose/status의 PLC-local D/W/R/ZR 주소와 word packing |
| 쓰기 | 로봇별 request 주소, level/pulse, one-hot/reset timing |
| 확인 | actual state/speed 또는 request-seq/ack-seq/result |
| fail-safe | Jetson heartbeat 주소와 PLC ladder timeout 동작 |

참고자료의 `RwrD...`는 상위 통신 buffer 표기다. Main PLC에서 MC로 접근할 실제
`D2000`이라는 뜻이 아니므로 주소를 그대로 복사하지 않는다.

## GX Works Open Setting

QnUCPU 계열 후보 예시는 다음과 같다. 실제 CPU 매뉴얼을 우선한다.

```text
Protocol              TCP
Open system           MC Protocol
Communication code    Binary
Local port            9000 (현장 확인값)
Online change         쓰기 FAT를 할 때만 승인 후 허용
```

현재 E71 Open Setting과 실측은 TCP `9000`, MC 3E Binary다. GX Works와
`config/plc.yaml` 양쪽 값을 일치시킨다.

remote password가 켜져 있으면 현재 client는 unlock을 구현하지 않았으므로 연결을
운영 승인하지 않는다.

## Jetson 제어 NIC 제안

격리된 USB-GbE/산업용 switch 구성의 제안값이다.

```text
PLC       192.168.10.30/24       (사용자 확인값)
Jetson    192.168.10.61/24       (권장 수정값)
Gateway   없음
DNS       없음
Default route로 사용하지 않음
```

실제 USB NIC 이름을 먼저 확인한다. `usb0/usb1`은 Jetson gadget interface일 수
있으므로 이름을 추측하지 않는다.

```bash
nmcli device status
ip -br link

sudo nmcli con add type ethernet ifname <enx...> con-name plc-control \
  ipv4.method manual ipv4.addresses 192.168.10.61/24 \
  ipv4.never-default yes ipv4.gateway "" ipv4.dns "" \
  ipv6.method disabled
sudo nmcli con up plc-control
```

공장망에 기존 주소 계획이 있으면 위 제안 대신 현장 값을 사용한다.

## 읽기부터 확인

```bash
ping -c 3 192.168.10.30
python3 tools/plc_probe.py --host 192.168.10.30 --port 9000 --dump D1000 21
```

Jetson `eno1=192.168.10.61/24`에서 ping과 TCP 9000, MC batch-read가 성공했다.
기존 `192.168.0.61/24`는 PLC와 다른 subnet이라 직접 연결에 사용하지 않는다.

`D1000 x21`과 No.9~17 주소는 읽혔지만 pose scale/sign/zero offset은 아직
교정 전이다. MC 정상응답과 값 변화만으로 정밀 자세 계약이 끝난 것은 아니다.

No.9~17 쓰기는 public launch에서 잠겨 있다. 입회 FAT 때만
`allow_field_control_writes:=true`로 열며, D1100.1은 항상 읽기 전용이다.

## 자주 만나는 오류

| 증상 | 확인 |
|---|---|
| TCP 연결 실패 | NIC link, subnet, Open Setting, port |
| `0xC050` | binary/ASCII 불일치 |
| `0xC056` | PLC-local device 주소/범위 오류 |
| `0xC05B` | device 종류 또는 쓰기 권한 |
| `0xC059` | 3E/4E 또는 command 형식 |
| `0xC201` | remote password 잠김 |
| 값은 읽히나 자세가 다름 | DWORD word order, scale, sign, zero offset |

MC end code `0x0000`은 PLC memory 요청 성공이다. 로봇이 실제로 정지/감속했다는
뜻이 아니며, 그 판단은 별도의 PLC actual feedback으로만 한다.
