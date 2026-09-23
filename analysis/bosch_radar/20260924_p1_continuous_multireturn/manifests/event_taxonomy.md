# Mechanical Event Taxonomy

| Event | Mechanical definition |
|---|---|
| `RAW_DROPOUT` | previously observed raw ID is absent from the fresh completed scan |
| `RAW_REASSIGNMENT` | raw association maps sensor evidence to a different raw-track identity |
| `GROUP_FRACTURE` | continuously observed members of one prior multi-member PID become multiple groups |
| `REP_CHANGE` | one retained physical PID selects a different representative raw member |
| `PUBLICATION_GAP` | previously published physical surface has no point in the current publication |
| `ALIAS_CHANGE` | a published physical PID receives a different public `Track` alias |

These labels can co-occur. `GROUP_FRACTURE` is not called reacquisition. Physical PID, representative raw ID, raw
track ID, CAN slot, and public alias remain separate fields throughout mining.

Boundary cause precedence is: distance diameter, lateral diameter, velocity diameter, stationary/moving conflict,
distance growth, lateral growth, insufficient maturity, then complete-link order/other. The full edge record retains
actual value, threshold, excess, deltas, growth, and sample count.
