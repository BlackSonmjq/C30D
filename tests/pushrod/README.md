# Pushrod hardware timeout regression

From the repository root:

```powershell
uv run --with unicorn --with pyelftools python tests/mango/run_tests.py tests/pushrod/test_pushrod.c
```

Exercises the real driver with mocked registers. Checks all four outputs, 50000
timer counts at 10 kHz, cutoff without task scheduling, exclusive timed operation,
invalid durations, pending-interrupt cancellation, restart, and APB1 clock scaling.
This is a register/logic test, not an electrical timing or final-position measurement.
