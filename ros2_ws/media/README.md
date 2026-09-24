# RViz demo clips

Both clips are 10 seconds, 1400 × 900, 12 fps, silent MP4 recordings from the
Ubuntu/WSLg RViz demo on 2026-09-24. They show synthetic ArUco images and the
`ros2_control` mock arm; neither clip shows physical hardware.

- [Mock IK motion](mock_ik_motion_10s.mp4): a pose goal generated from the demo
  model moves the six-joint arm from a distant pose back to its observation pose.
- [Mock tag mission](mock_tag_mission_10s.mp4): the detector sees synthetic tags,
  and the mission advances through three hover targets.

To repeat the pose-goal demonstration, start `scripts/run_demo.sh` in one Ubuntu
terminal, then source the workspace in another and run:

```bash
export ROS_DOMAIN_ID=71 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
python3 scripts/drive_mock_demo.py far
python3 scripts/drive_mock_demo.py home
```

The helper refuses to send a goal unless the sole active hardware component is
`URCMockArm` using `mock_components/GenericSystem` on demo domain 71.
