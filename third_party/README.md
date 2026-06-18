# Third-party task stacks

MobileROS does not vendor large robotics task stacks. The scripts in this
directory clone the upstream projects used by the adapters:

- ORB-SLAM3 for visual SLAM.
- OpenPCDet for LiDAR perception.
- OpenAirInterface v2.1.0 for RF simulator and USRP runs.

The MobileROS runtime adapts topic rates, frame/keyframe flow, voxel size, and
slice priority around these stacks. Hardware results must be collected from the
actual OAI/USRP or OAI RF simulator run and compared with
`benchmarks/run_replay.py --mode hardware`.
