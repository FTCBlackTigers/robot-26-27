package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;

/**
 * Shooter: a flywheel that launches game pieces, plus a feeder that pushes a piece into it.
 * <p>
 * PLACEHOLDER: the robot isn't built yet, so every hardware line is commented out and the methods
 * only track {@link #getState()}. Once the mechanism exists:
 * <ol>
 *     <li>Uncomment the hardware fields + the lines in the constructor.</li>
 *     <li>Make the names match the Driver Station robot configuration.</li>
 *     <li>Uncomment the bodies of the methods below and tune the velocity / PIDF values.</li>
 * </ol>
 */
public class ShooterSubsystem extends SubsystemBase {
    public enum State {
        IDLE,
        SPINNING_UP,
        READY
    }

    // Tune once the mechanism exists
    public static double TARGET_VELOCITY = 1500; // encoder ticks per second
    public static double VELOCITY_TOLERANCE = 50; // ticks per second
    public static double FEEDER_REST = 0.0;
    public static double FEEDER_PUSH = 0.5;

    // --- Hardware (commented out until the mechanism exists) ---
    // private final MotorEx flywheel;   // com.seattlesolvers.solverslib.hardware.motors.MotorEx
    // private final ServoEx feeder;     // com.seattlesolvers.solverslib.hardware.ServoEx

    private State state = State.IDLE;
    private double targetVelocity = 0;

    public ShooterSubsystem(HardwareMap hardwareMap) {
        // flywheel = new MotorEx(hardwareMap, "flywheel");
        // flywheel.setRunMode(Motor.RunMode.VelocityControl);
        // flywheel.setVeloCoefficients(0.01, 0, 0);   // P, I, D
        // flywheel.setFeedforwardCoefficients(0, 1);  // kS, kV
        // flywheel.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT); // let the flywheel coast down

        // feeder = new ServoEx(hardwareMap, "feeder");
        // feeder.set(FEEDER_REST);
    }

    /** Start spinning the flywheel to {@link #TARGET_VELOCITY}. */
    public void spinUp() {
        setTargetVelocity(TARGET_VELOCITY);
    }

    public void setTargetVelocity(double velocity) {
        targetVelocity = velocity;
        state = velocity == 0 ? State.IDLE : State.SPINNING_UP;
        // flywheel.setVelocity(velocity);
    }

    public void stop() {
        setTargetVelocity(0);
        retractFeeder();
    }

    /** Push a game piece into the flywheel. Only do this when {@link #isReady()}. */
    public void feed() {
        // feeder.set(FEEDER_PUSH);
    }

    public void retractFeeder() {
        // feeder.set(FEEDER_REST);
    }

    /** True when the flywheel is at speed and it's safe to feed. Always false until hardware exists. */
    public boolean isReady() {
        return state == State.READY;
    }

    public double getTargetVelocity() {
        return targetVelocity;
    }

    public State getState() {
        return state;
    }

    @Override
    public void periodic() {
        // Runs every loop: flip SPINNING_UP -> READY once the flywheel reaches the target
        // if (state != State.IDLE) {
        //     boolean atSpeed = Math.abs(flywheel.getVelocity() - targetVelocity) < VELOCITY_TOLERANCE;
        //     state = atSpeed ? State.READY : State.SPINNING_UP;
        // }
    }
}
