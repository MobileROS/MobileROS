# UE Slice Manager (2A)

Mock UE-side slice manager capable of UDP tunneling/transparent forwarding and periodic UE State Report publication.

## Running
```
python3 ue_slice_manager.py --listen 127.0.0.1:50000 --remote 127.0.0.1:50001 --report-addr 127.0.0.1:60000
```
This listens locally, forwards datagrams to the remote endpoint, and mirrors remote traffic back to the UE while emitting JSON reports with a mock criticality score and semantic intent.
