# 축 캘리브레이션 절차 (SCALE · dir · offset)

`angle_deg[i] = raw_dword[i] × scale[i] × dir[i] + offset_deg[i]`

세 값이 확정되기 전에는 디지털 트윈 각도를 신뢰할 수 없다.
`robots.yaml` 의 `calibrated: false` 인 동안 노드가 `[미교정]` 경고를 계속 띄운다.

---

## 1. scale — LSB 당 각도

**측정 방법**

1. 로봇을 `home` 자세로 두고 `plc_probe --watch` 실행
2. J1 만 티치펜던트로 **정확히 +90°** 조그
3. raw 값 변화량 `Δraw` 기록

```
scale = 90 / Δraw
```

| Δraw | scale | 흔한 표기 |
|---|---|---|
| 90,000 | 0.001 | 밀리도 — 가장 흔함 |
| 9,000 | 0.01 | 1/100 도 |
| 그 외 | `360 / (encoder_ppr × gear_ratio)` | 펄스 단위 |

6축 모두 같은 스케일인지 반드시 확인한다. 감속비가 다르면 축마다 다를 수 있다.

## 2. dir — 회전 부호

+90° 조그했는데 Unity 모델이 **반대로** 돌면 `dir[i] = -1`.
STEP 도면에서 축 라인은 확정됐지만 **+회전 방향은 미확정**이므로 실측이 필요하다.

## 3. offset — 원점 보정

PLC 원점(raw=0) 자세와 Unity 모델의 home(STEP as-built) 자세 차이를 메운다.

```
offset_deg[i] = (실제 로봇의 해당 자세 각도) − (raw × scale × dir)
```

## 4. 검증 체크리스트

- [ ] `cmd_degs = [0,0,0,0,0,0]` → Unity 모델이 home 자세
- [ ] 각 축 단독 +90° → **해당 링크만**, **올바른 축**으로 회전
- [ ] 실로봇 조그 각도 ↔ Unity 각도 오차 ≤ 1°
- [ ] `limits_deg` 안에서 `clamped` 경고가 안 뜸
- [ ] 로딩 · 언로딩 각각 별도 인스턴스 · 토픽으로 검증
- [ ] 완료 후 `robots.yaml` 의 `calibrated: true` 로 변경

## 5. 기록 양식 (회신용)

```yaml
# 실측 결과 — robots.yaml 에 그대로 붙여넣기
calibrated: true
scale:  [0.001, 0.001, 0.001, 0.001, 0.001, 0.001]
dir:    [+1, -1, +1, +1, -1, +1]
offset: [0.0, -90.0, 0.0, 0.0, 90.0, 0.0]
```

> CDR 확인필요 항목 **C-17 · C-18 · C-19** 와 동일하다. 회신 시 위 형식으로 주면
> 그대로 반영된다.
