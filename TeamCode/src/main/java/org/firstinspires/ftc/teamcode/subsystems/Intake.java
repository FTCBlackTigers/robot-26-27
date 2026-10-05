package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Intake: pulls game pieces into the robot (intake) or spits them back out (outtake).
 * <p>
 * PLACEHOLDER: the robot isn't built yet, so every hardware line is commented out and the methods
 * only track {@link #getState()}. Once the mechanism exists:
 * <ol>
 *     <li>Uncomment the hardware fields + the lines in the constructor.</li>
 *     <li>Make the names match the Driver Station robot configuration.</li>
 *     <li>Uncomment the bodies of the methods below and tune the powers.</li>
 * </ol>
 */
public class Intake extends SubsystemBase {
    public enum State {
        IDLE,
        INTAKING,
        OUTTAKING
    }

    // Tune once the mechanism exists
    public static double INTAKE_POWER = 1.0;
    public static double OUTTAKE_POWER = -1.0;

    private final Telemetry telemetry;

    // --- Hardware (commented out until the mechanism exists) ---
    // private final MotorEx intakeMotor;           // com.seattlesolvers.solverslib.hardware.motors.MotorEx
    // private final DistanceSensor pieceSensor;    // com.qualcomm.robotcore.hardware.DistanceSensor (if we add one)

    private State state = State.IDLE;

    public Intake(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;

        // intakeMotor = new MotorEx(hardwareMap, "INTAKE");
        // intakeMotor.setRunMode(Motor.RunMode.RawPower);
        // intakeMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        // intakeMotor.setInverted(false);

        // pieceSensor = hardwareMap.get(DistanceSensor.class, "intakeSensor");
    }

    /** Spin the intake inwards to grab game pieces. */
    public void intake() {
        state = State.INTAKING;
        // intakeMotor.set(INTAKE_POWER);
    }

    /** Spin the intake outwards to spit game pieces out (unjam / reject a piece). */
    public void outtake() {
        state = State.OUTTAKING;
        // intakeMotor.set(OUTTAKE_POWER);
    }

    public void stop() {
        state = State.IDLE;
        // intakeMotor.set(0);
    }

    /** True when a game piece is inside the robot. Always false until a sensor exists. */
    public boolean hasGamePiece() {
        // return pieceSensor.getDistance(DistanceUnit.CM) < 5;
        return false;
    }

    public State getState() {
        return state;
    }

    @Override
    public void periodic() {
        telemetry.addData("Intake", state);
    }
}
