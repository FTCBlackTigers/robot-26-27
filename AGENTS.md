# Robot project

- Install the TeamCode APK using the normal Android Run configuration.
- Keep Sloth, Slothboard, and Sloth Panels out of runtime dependencies: previous installs
  were associated with controller startup failures (3680d6d). Pedro AutoTune 1.0.0
  transitively includes Sloth; its sources are preserved in TeamCode/disabled-autotune/.
- Keep Pedro revhub/path following enabled. Hardware initialization occurs at OpMode INIT.
- Build success is not proof of hub startup; report physical startup verification separately.
- Intake smoke test requires only `INTAKE`: gamepad1 RB/LB hold runs intake/outtake,
  release stops. Main TeleOp uses the same commands on gamepad2 and also requires drive hardware.

| Skill | Use |
|---|---|
| focused-verification | Verify build/dependency changes and distinguish APK checks from physical robot startup. |
