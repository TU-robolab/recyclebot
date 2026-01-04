# Quick Testing Guide

## Run Hardware-Free Tests

```bash
# Inside the container
docker exec -it recyclebot-dev-1 bash
source /ros_entrypoint.sh

# Test specific subsystem
cd /home/joao/test_suite
./run_tests.sh vision    # Vision system
./run_tests.sh robot     # Robot system
./run_tests.sh gripper   # Gripper system
./run_tests.sh full      # All tests

# Or from host
docker exec recyclebot-dev-1 bash -c "source /ros_entrypoint.sh && /home/joao/test_suite/run_tests.sh vision"
```

## What Gets Tested

- **Vision**: YOLO detection with fake camera data
- **Robot**: Joint state publishing and MoveIt
- **Gripper**: Service interface and commands
- **Full**: All subsystems sequentially

See `test_suite/README.md` for complete documentation.
