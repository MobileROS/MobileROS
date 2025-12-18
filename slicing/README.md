# Bidirectional Dynamic Slicing Pipeline

This folder contains runnable mocks for the UE slice manager (2A), gNB slice manager hooks (2B), and the Hub slicing controller (2C).

1. Start the gNB slice manager:
```
python3 slicing/gnb_slice_manager/gnb_slice_manager.py
```
2. Run the Hub slicing controller to emit promotions/rollbacks:
```
python3 slicing/hub_slicing_controller/slicing_controller.py --gnb-addr 127.0.0.1:60001
```
3. Start the UE slice manager to forward packets and send reports:
```
python3 slicing/ue_slice_manager/ue_slice_manager.py --report-addr 127.0.0.1:60000
```

Logs will show the report reception -> PRB/weight update -> RRM commands and timeout-based rollbacks.
