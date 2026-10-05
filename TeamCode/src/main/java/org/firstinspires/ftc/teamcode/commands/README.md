# Commands

A **command** is one robot action ("intake until we have a piece", "shoot"), built from
**subsystem** methods. Subsystems say *what the hardware can do*; commands say *when and in what
order*. `Robot.java` binds commands to buttons (TeleOp); autos chain them (`AutonomousBase`).

Source: SolversLib docs, https://docs.seattlesolvers.com/command-base/command-system

## Layout

One folder per subsystem (`drive/`, `intake/`, ...), plus `automations/` for sequences that use
several subsystems (same layout as robot-25-26). Right now there's only:

- `drive/DriveWithController`: driving from the gamepad (TeleOp default command)
- `intake/IntakeIn`: the example to copy when writing a new command
- `drive/AimAtTarget`: vision shell, not used yet

**When to write a class vs. a one-liner:** if the action just calls one method, write it inline in
`Robot.java`, e.g. `new InstantCommand(shooter::stop, shooter)` or
`new StartEndCommand(intake::outtake, intake::stop, intake)`. Make a class when the command has
its own logic: an end condition (`isFinished`), cleanup (`end`), or several steps.

## The command lifecycle

Every command is a small state machine. The scheduler calls these for you:

| Method | When | Use it for |
|---|---|---|
| `initialize()` | once, when scheduled | start motors, set targets, reset timers |
| `execute()` | every loop while running | anything that must be updated continuously |
| `isFinished()` | every loop, after `execute()` | return `true` to end the command |
| `end(boolean interrupted)` | once, when finished **or interrupted** | stop motors, clean up |

Write a command class by extending `CommandBase` and calling `addRequirements(subsystem)` in the
constructor.

## Requirements (the important rule)

`addRequirements(...)` tells the scheduler which subsystems a command uses. **Only one command
can use a subsystem at a time.** If a new command needs a subsystem that's busy, the running
command is interrupted (its `end(true)` runs) and the new one starts. Forgetting
`addRequirements` means two commands can fight over the same motor.

## Default commands

`subsystem.setDefaultCommand(cmd)` runs `cmd` whenever nothing else uses that subsystem.
`Robot.configureTeleOp` uses this for driving: `DriveWithController` reads the sticks, and any
other command that requires the drivetrain (like `AimAtTarget`) takes over and then hands control
back.

## Scheduler

`CommandOpMode` runs the `CommandScheduler` every loop (`super.run()`). Each loop it polls the
buttons, runs every subsystem's `periodic()`, then runs the scheduled commands. Never block
(`sleep`, `while` loops) inside a command: that freezes the whole robot. Use `WaitCommand`,
`WaitUntilCommand` or `isFinished()` instead.

## Built-in commands (no class needed)

| Command | Does |
|---|---|
| `new InstantCommand(runnable, subsystems...)` | runs once and ends right away |
| `new RunCommand(runnable, subsystems...)` | runs every loop, never ends by itself |
| `new StartEndCommand(onStart, onEnd, subsystems...)` | one action at start, another at the end |
| `new FunctionalCommand(init, execute, end, isFinished, subsystems...)` | a full command inline |
| `new WaitCommand(ms)` | waits |
| `new WaitUntilCommand(() -> condition)` | waits until true |
| `new ConditionalCommand(ifTrue, ifFalse, () -> condition)` | picks one of two |
| `new SelectCommand(map, selector)` | picks from a map (state machines) |
| `new RepeatCommand(cmd, times)` / `RetryCommand` | repeat, or retry until success |
| `new UninterruptibleCommand(cmd)` | can't be interrupted |

## Groups (commands made of commands)

| Group | Runs | Ends when |
|---|---|---|
| `SequentialCommandGroup` | one after another | the last one ends |
| `ParallelCommandGroup` | all at once | all end |
| `ParallelRaceGroup` | all at once | the first one ends (others are interrupted) |
| `ParallelDeadlineGroup` | all at once | the first (deadline) command ends |

A group requires every subsystem its children require. Commands inside one group can't share a
subsystem when they run in parallel.

## Decorators (shortcuts on any command)

| Decorator | Does |
|---|---|
| `.withTimeout(ms)` | ends after `ms` even if not finished |
| `.interruptOn(() -> condition)` | ends early when the condition is true |
| `.andThen(cmd...)` | then run these |
| `.alongWith(cmd...)` | in parallel, wait for all |
| `.raceWith(cmd...)` | in parallel, stop at the first to end |
| `.deadlineWith(cmd...)` | in parallel, stop when this one ends |
| `.beforeStarting(runnable)` / `.whenFinished(runnable)` | small extra actions |
| `.perpetually()` | never ends by itself |

Example: `new InstantCommand(shooter::spinUp, shooter).andThen(new WaitCommand(1000), new IntakeIn(intake).withTimeout(500))`

## Binding to buttons (TeleOp)

All bindings live in `Robot.configureTeleOp`:

```java
GamepadEx operator = new GamepadEx(gamepad2);
operator.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
        .whenHeld(new IntakeIn(intake));   // starts on press, cancelled on release
operator.getGamepadButton(GamepadKeys.Button.A)
        .whenPressed(new InstantCommand(shooter::spinUp, shooter)); // starts on press, ends right away
```

| Binding | Behavior |
|---|---|
| `whenPressed(cmd)` | start on press |
| `whenReleased(cmd)` | start on release |
| `whenHeld(cmd)` | start on press, cancel on release |
| `whileHeld(cmd)` | keep restarting while held, cancel on release |
| `toggleWhenPressed(cmd)` | press to start, press again to cancel |

Prefer `whenHeld` for "hold to run" commands that can finish by themselves (like
`IntakeIn`). `whileHeld` would restart them right after they finish.

For triggers (analog), wrap them: `new Trigger(() -> operator.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.5)`.

## Autonomous

Extend `opmodes/AutonomousBase` and return the whole routine as one command. Chain Pedro's path
commands with ours (paths: see `samples/PedroAutoSample.java`):

```java
protected Command routine(Robot robot) {
    Follower follower = robot.drivetrain.getFollower();
    return new SequentialCommandGroup(
            new FollowPathCommand(follower, toScore),
            new InstantCommand(robot.shooter::spinUp, robot.shooter),
            new FollowPathCommand(follower, toPickup).alongWith(new IntakeIn(robot.intake).withTimeout(2000))
    );
}
```
