# Robot project

- Install the TeamCode APK using the normal Android Run configuration.
- Pedro AutoTune 1.0.0 is required (web UI `http://192.168.43.1:10158`, sources in
  `teamcode/pedroPathing/`). It pulls the Sloth library at runtime; that is accepted. Keep the
  Sloth Load/fast-deploy plugin, Slothboard, and Sloth Panels forks out: Sloth was earlier
  associated with controller startup failures (3680d6d). If the RC app stops starting, bisect
  `com.pedropathing:tuning` first.
- Keep Pedro revhub/path following enabled. Hardware initialization occurs at OpMode INIT.
- Build success is not proof of hub startup; report physical startup verification separately.
- Intake smoke test requires only `INTAKE`: gamepad1 RB/LB hold runs intake/outtake,
  release stops. Main TeleOp uses the same commands on gamepad2 and also requires drive hardware.

| Skill | Use |
|---|---|
| focused-verification | Verify build/dependency changes and distinguish APK checks from physical robot startup. |
