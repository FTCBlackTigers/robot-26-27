# Commands

A **command** is one robot action ("intake until we have a piece", "shoot"), built from
**subsystem** methods. Subsystems say *what the hardware can do*; commands say *when and in what
order*. OpModes just bind commands to buttons (TeleOp) or chain them (Autonomous).

Source: SolversLib docs, https://docs.seattlesolvers.com/command-base/command-system

## What's in this folder

| Command | Requires | What it does | Ends when |
|---|---|---|---|
| `IntakeCommand` | intake | Runs the intake inwards, stops it at the end | a piece is detected, or interrupted (button released) |
| `OuttakeCommand` | intake | Runs the intake outwards, stops it at the end | interrupted (button released) |
| `SpinUpShooterCommand` | shooter | Starts the flywheel | flywheel is at speed |
| `ShootCommand` | shooter | Spin up (with timeout) → feed → wait → retract | sequence done |
| `AimAtTargetCommand` | drive | SHELL: turn toward the Limelight target | right away, until it's written |

Intake and shooter are still placeholders (see `subsystems/`), so these commands run their logic
but nothing moves. `hasGamePiece()` and `isReady()` are always false for now, so
`IntakeCommand` only ends on release and `ShootCommand` relies on its spin-up timeout.

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
`MainTeleOp` uses this for driving: the drive default command reads the sticks, and any other
command that requires `drive` (like aiming) takes over and then hands control back.

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

Example: `new SpinUpShooterCommand(shooter).withTimeout(1500).andThen(new InstantCommand(shooter::feed, shooter))`

## Binding to buttons (TeleOp)

```java
GamepadEx operator = new GamepadEx(gamepad2);
operator.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
        .whenHeld(new IntakeCommand(intake));   // starts on press, cancelled on release
operator.getGamepadButton(GamepadKeys.Button.X)
        .whenPressed(new ShootCommand(shooter)); // starts on press, runs to the end
```

| Binding | Behavior |
|---|---|
| `whenPressed(cmd)` | start on press |
| `whenReleased(cmd)` | start on release |
| `whenHeld(cmd)` | start on press, cancel on release |
| `whileHeld(cmd)` | keep restarting while held, cancel on release |
| `toggleWhenPressed(cmd)` | press to start, press again to cancel |

Prefer `whenHeld` for "hold to run" commands that can finish by themselves (like
`IntakeCommand`). `whileHeld` would restart them right after they finish.

## Autonomous

Chain commands with Pedro's path commands in a `SequentialCommandGroup`, see
`samples/PedroAutoSample.java`:

```java
schedule(new SequentialCommandGroup(
        new FollowPathCommand(follower, toScore),
        new ShootCommand(shooter),
        new FollowPathCommand(follower, toPickup).alongWith(new IntakeCommand(intake).withTimeout(2000))
));
```
