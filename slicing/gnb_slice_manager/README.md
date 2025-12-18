# gNB Slice Manager (2B)

Mock extension hook for the OAI MAC scheduler. It consumes UE State Reports over UDP, updates per-UE weights/PRB allocations, and listens for hub-generated RRM commands.

- Reports: `--report-bind 0.0.0.0:60000`
- RRM commands: `--rrm-bind 0.0.0.0:60001`

Logs show the end-to-end chain: receiving a report, updating weights/PRBs, and persisting the snapshot.
