# Candidate matrix

| Candidate | Causal rule | State | Scope / intent |
|---|---|---|---|
| E | prior current member and scalar cost `<= best + 0.25` | none | previous artifact exact reproduction; shared-selector scope |
| E1 | same members, corpus-derived near-tie, no positive-support superiority, challenger was a representative within bounded 2 s past | recent representative timestamps | suppress return leg of likely A-B-A only |
| E2 | same members, `delta <= 0.25`, no support superiority; challenger must win 2 consecutive scans | challenger/count | stable-membership confirmation |
| E3 | same members, corpus-derived near-tie, no camera/OEM superiority | none | support-change-aware retention |
| E4 | E3 conditions plus current-member coordinate jump exceeds baseline R1 p95 on d/y/v | none | large-jump weak-advantage retention |
| E5 | current-member medoid using d/y/v geometry, then support/age/raw ID tie-break | none | broad analysis-only comparison |

E1-E5는 current observed member만 고른다. E1/E2 state는 PID death, singleton, >300 ms gap, provider/segment reconstruction에서 폐기되며 bounded history만 보유한다. Threshold는 future duration이나 actor label을 쓰지 않고 baseline A-B-A score-delta 분포에서 정한다.

## Full-corpus result

| Policy | Rep changes | Net reduction | Directly suppressed | Ping-pong returns suppressed | Publication-changed scans | PID/member mismatch scans | radarState/planner-input changed | Decision |
|---|---:|---:|---:|---:|---:|---:|---:|---|
| E | 50,553 | 7,952 | 9,661 | 5,631 | 80,769 (`legacy=80,770`) | 19,513 | 5,094 | NO-GO |
| E1 | 56,137 | 2,368 | 2,904 | 2,794 | 16,562 | 4,368 | 569 | NO-GO |
| E2 | 52,021 | 6,484 | 8,093 | 5,074 | 65,437 | 14,620 | 3,852 | NO-GO |
| E3 | 51,780 | 6,725 | 8,145 | 5,115 | 66,676 | 15,374 | 3,857 | NO-GO |
| E4 | 57,587 | 918 | 1,180 | 624 | 17,456 | 3,113 | 746 | NO-GO |
| E5 | 40,129 | 18,376 | 33,103 | 16,378 | 269,591 | 114,096 | 34,205 | NO-GO |

모든 policy는 group partition mismatch 0 및 stale representative 0이었다. 그러나 selector 결과가 ambiguous PID assignment의 `representative +3` score에 되먹임되어 모든 policy가 full corpus에서 exact raw→PID/PID→members parity를 위반했다. 따라서 accepted best candidate는 없다. E1/E4의 작은 publication blast radius는 연구상 유용하지만 identity gate 실패보다 우선할 수 없다.
