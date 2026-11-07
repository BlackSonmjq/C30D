# Chassis heading regression

Run from the repository root:

```powershell
uv run --with unicorn --with pyelftools python tests/chassis/run_tests.py
```

Runs the production `Balance_task`, wheel PI functions and heading controller
under ARMCC/Unicorn, with deterministic board and RTOS mocks. Covers normal parked
heading hold after the 10-second startup delay, 50 seconds of parked upper-actuator
vibration followed by immediate motion commands, explicit turning, normal moving
heading correction, disabled wheel outputs, and motion during upper operation.
Only parked upper operation pauses heading hold. The parked-to-moving case verifies
that the old heading error is discarded instead of replayed on resumption. Actuator
busy state does not block chassis motion commands.

Source functions are extracted into an ignored build file to exclude unrelated
remote input, flash and kinematics dependencies. This does not validate physical
vibration, wheel traction, electrical noise or actual RTOS scheduling.
