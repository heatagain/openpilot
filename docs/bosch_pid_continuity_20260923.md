# Bosch MRRevo14F PID 연속성 연구 요약

2026-09-23 offline 연구의 핵심 결과를 보존한 요약입니다. 아래 수치는 당시 연구 기준이며 현재 브랜치의 재검증 결과가 아닙니다.

## 결론

**INCONCLUSIVE; 서로 다른 PID를 이어 history를 승계하는 production 정책은 NO-GO.**

관측된 주요 문제는 같은 raw return이 group에 편입·분리되면서 생기는 PID 분절과 surface migration입니다. 단순 소실 후 재획득으로 해석하여 이전 PID를 복원하면 현재 group owner와 충돌할 수 있습니다. 신규 stitching candidate나 SHADOW rule은 구현하지 않았습니다.

## 근거와 한계

| 항목 | 당시 결과 | 해석 범위 |
|---|---|---|
| Cold provider replay | 23개 입력 segment, 13,684 완료 scan | 기존 decoded offline cache와 과거 영상 GT 사용; 새 영상 라벨링 없음 |
| Warm replay | 2개 연속 구간, 추가 실행 3,000 scan | Cold replay와 중복되므로 독립 근거로 합산하지 않음 |
| 과거 영상 SAME sequential 전이 | 12건 중 양 endpoint 유일 대응 4건, 재매핑 불확실 8건 | 4건 모두 distinct PID; 물리 객체 재획득 성공률로 해석하지 않음 |
| Route269 S7 | 유일 대응 3건에서 raw482가 group와 singleton 사이 이동 | LOG-BASED INFERENCE: group ownership oscillation에 따른 PID 분절 |
| DIFFERENT 대조 | 동시 존재한 두 객체 1쌍은 병합되지 않음; sequential DIFFERENT GT 0건 | 사망→신규 birth 전이의 false-stitch 안전률은 UNEVALUABLE |
| 사건 prefix | 5/5에서 baseline identity/state/publication signature 일치 | Offline provider baseline 검사; 신규 candidate와 downstream 검증 아님 |
| Route280 S15 cut-in | cold replay PID/raw 유지, 첫 완료 scan부터 published | radarState의 lead 선택 시점과 제어 반응은 NOT MEASURED |
| Windows radar 테스트 | Params shim: 244 passed, 3 deselected; 전체: 245 passed, 2 failed | 당시 Group3 OBJECT_ID DBC skew; 직접 pytest는 params_pyx 부재로 collection 실패 |

당시 후속 validator는 입력 수, GT 재매핑 구분, prefix 및 raw ownership lineage를 검사하여 PASS했습니다. 전체 pipeline의 정확성이나 안전성을 검증한 결과가 아닙니다. 이후 Group3 DBC 갱신과 별도 563-test 결과는 [로그 정리 기록](../../../docs/bosch_pr_logging_cleanup_20260929.md)에 구분해 보존되어 있습니다.

`liveTracks→radarState` recorded parity, planner/제어 동치, native IPC, 실차 runtime·반응 및 A1M CPU는 이 연구에서 **NOT VERIFIED / NOT MEASURED**입니다. 다음 연구는 scan별 group owner 경쟁 근거와 같은 운동학 범위의 영상 확인 sequential DIFFERENT 대조를 확보하는 것입니다. 이번 결론으로 production PID/history 변경을 승인하지 않습니다.

## 출처와 상세 자료 보관

- 당시 시작 HEAD: `d5c8b555351e6b38422446aecfb9fb20820d6432`.
- 당시 replay provider SHA-256: `76055c522059469f3bef686476bc9cb0b6fe1499960d4e68cff7a81bf982524d`.
- 원본 보고서 SHA-256: `a81df6f36b08d55939ab0c390676603fc910229448f3b80b9bfeacee32d6d684`.
- 원본 보고서·스크립트·manifest·표·trace 22개는 정리 전 commit `39046371fef4e25374f37a8a2565fb556ef93b9f`의 이 디렉터리에 보존되어 있습니다.
- 2026-10-03 PR 구성 정리에서 이 디렉터리는 요약 1개로 축소했습니다. 원본 22개와 Git에서 제외됐던 로컬 캐시를 포함한 49개 파일은 저장소 밖 `C:\CarrotRadarResearch\analysis\20261003_bosch_pr_research_archive\20260923_pid_continuity_reacquisition\`에 복사하고 SHA-256을 대조했습니다. 상위 폴더의 `sha256_manifest.csv`와 `archive_metadata.json`에 보관 범위와 해시를 기록했습니다. 로컬 보관이며 원격 백업이 아닙니다.

재현에는 당시 provider revision, 기존 decoded corpus 및 과거 GT가 함께 필요합니다. 보관된 스크립트의 경로와 provider hash 조건을 확인한 뒤 사용해야 합니다. 이번 파일 정리에서는 replay나 동작 테스트를 다시 실행하지 않았습니다.
