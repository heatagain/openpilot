# Final validation

- Corpus downstream replay: PASS, 34 routes / 847 segments / 504,673 scans.
- Baseline cache parity: PASS, scan/qualification/source-current publication/scan-count mismatch 0.
- Complete live invariance: PASS, 63/63 P0–P6 invariant rows zero; complete aLead debug state included.
- Prefix invariance: PASS, 102/102 comparisons.
- Synthetic selector: PASS, 7 policies / 15 cases / 217 rows; tracking mismatch 0, stale 0, nonfresh 0.
- Route2bc S21: PASS, 11 scans and P0–P6 publication/tracking/PID/member regression 0.
- Route280 S15: PASS, first-publication delay 0.0 ms and target-coordinate regression 0.
- Known P0 publication-duration regression: PASS, 0.
- Study script compileall: PASS.
- Study script Ruff: PASS.
- Existing production radar regression: PASS, 241 passed / 6 deselected / 3 environment-config warnings in 1.98 s.
- Production source changed: no.
- Production tests changed: no.
- Native MPC/acados requested-acceleration replay: NOT RUN.
- A1M CPU: NOT MEASURED.
- Real-car runtime, NAS deployment, and real-car application: NOT PERFORMED.
- Final decision: `PUBLICATION CHURN MOSTLY BENIGN`; no retained production candidate.
