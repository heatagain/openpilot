# Downstream protocol

Each policy builds only analysis `RadarPoint` clones:

- `trackId`: exact baseline public alias;
- `dRel/yRel/vRel`: one selected current observed member's whole tuple;
- `aLead`: exact baseline PID-owned estimate already computed before selection;
- `aRel/yvRel/jLead`: NaN, matching the Bosch provider contract;
- `vLead`: current ego speed plus selected member `vRel`.

The existing `DPathRadarController` is replayed independently for P0 and every policy after its first divergence. Before divergence, identical P0 output/state is shared; at the first divergence the pre-update P0 controller is cloned, after which the policy controller runs every scan. This is an exact optimization because the inputs and controller state are identical before the first divergence.

`radarState` deltas classify coordinate-only, lead ID, gain, loss and lead1/lead2 swap outcomes. Exposed selected-lead aLead changes and planner-input tuple changes are counted separately. Native MPC/acados requested acceleration is not available in this Windows study and remains `NOT RUN`.
