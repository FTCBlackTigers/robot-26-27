# Robot project

- Install the TeamCode APK using the normal Android Run configuration.
- Pedro AutoTune 1.0.0 is required (web UI `http://192.168.43.1:10158`, sources in
  `teamcode/pedroPathing/`). It pulls the Sloth library at runtime; that is accepted. Keep the
  Sloth Load/fast-deploy plugin, Slothboard, and Sloth Panels forks out: Sloth was earlier
  associated with controller startup failures (3680d6d). If the RC app stops starting, bisect
  `com.pedropathing:tuning` first.
- `ForesightTuner.java` has local safety stops (`DriveGuard`, `checkSafety`): distance, time, Stop
  button, stall, and swapped-pod checks. Upstream's loops trust the localizer alone and drove into a wall.
  Its `step()` wrapper lists the values already measured when a later step fails (results otherwise
  appear only at the very end). Keep both if you re-copy the tuner from upstream.
- `Constants.foresightConfig` still holds the quickstart's sample numbers, not this robot's. Do not
  treat path-following problems as tuned-robot problems until the Foresight Tuner has finished once.
- Keep Pedro revhub/path following enabled. Hardware initialization occurs at OpMode INIT.
- Build success is not proof of hub startup; report physical startup verification separately.
- Intake smoke test requires only `INTAKE`: gamepad1 RB/LB hold runs intake/outtake,
  release stops. Main TeleOp uses the same commands on gamepad2 and also requires drive hardware.

| Skill | Use |
|---|---|
| focused-verification | Verify build/dependency changes and distinguish APK checks from physical robot startup. |
