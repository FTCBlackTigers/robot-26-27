package org.firstinspires.ftc.teamcode.pedroPathing.procedures;

import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.drivetrain.Drivetrain;
import com.pedropathing.localization.Localizer;
import com.pedropathing.math.Pose;
import com.pedropathing.math.Vector2D;
import com.pedropathing.tuning.autotune.Inputs;
import com.pedropathing.tuning.autotune.Procedure;
import com.pedropathing.tuning.autotune.TuningOpMode;
import com.pedropathing.utils.Angle;
import com.pedropathing.utils.Utils;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.*;
import java.util.function.Function;

import static com.pedropathing.utils.Utils.linearFit;
import static com.pedropathing.utils.Utils.quadraticFit;

public class ForesightTuner extends Procedure {
    Function<HardwareMap, Localizer> localizerFunction;
    Function<HardwareMap, Drivetrain> drivetrainFunction;

    public ForesightTuner(Function<HardwareMap, Localizer> localizerFunction, Function<HardwareMap, Drivetrain> drivetrainFunction) {
        super("Foresight Tuner", "A procedure for tuning the Foresight Algorithm.");
        this.localizerFunction = localizerFunction;
        this.drivetrainFunction = drivetrainFunction;
    }

    /** Values from the steps that already finished, in order. Shown in the error if a later step fails. */
    private final Map<String, Object> measured = new LinkedHashMap<>();

    /**
     * Runs one step. Results are only shown when the whole procedure finishes, so a failure in a late step
     * used to throw away every earlier measurement. Here the error message lists what was already measured,
     * and a step that returns nothing usable (stopped early, NaN) fails instead of feeding the next step.
     */
    private <T> T step(String label, TuningOpMode<T> opMode) throws InterruptedException {
        T value;
        try {
            value = runOpMode(opMode);
        } catch (RuntimeException e) {
            throw new IllegalStateException(opMode.name + " failed: " + e.getMessage() + measuredSoFar(), e);
        }
        if (!usable(value)) {
            throw new IllegalStateException(opMode.name + " gave no usable result (" + value + "). Run it again with the robot free to move." + measuredSoFar());
        }
        measured.put(label, value);
        return value;
    }

    private static boolean usable(Object value) {
        if (value instanceof Double) {
            double d = (Double) value;
            return !Double.isNaN(d) && !Double.isInfinite(d);
        }
        if (value instanceof List) {
            List<?> list = (List<?>) value;
            if (list.isEmpty()) return false;
            for (Object item : list) {
                if (!usable(item)) return false;
            }
            return true;
        }
        return value != null;
    }

    private String measuredSoFar() {
        if (measured.isEmpty()) return "";
        return " | Measured before this: " + measured;
    }

    @Override
    public void run() throws InterruptedException {
        measured.clear();
        Inputs distanceInput = inputs("Distance", "The distance to drive in inches for the Max Achievable Forward and Strafe Identifiers");
        Inputs.Field<Double> distance = distanceInput.d("Distance").withDefault(48.0);
        awaitInputs(distanceInput);

        double forwardVelocity = step("maxAchievableForwardVelocity", new ForwardVelocity(localizerFunction, drivetrainFunction, distance.get()));
        double strafeVelocity = step("maxAchievableStrafeVelocity", new StrafeVelocity(localizerFunction, drivetrainFunction, distance.get()));

        Inputs velocityInput = inputs("Velocity", "The velocity to drive to in inches for the Max Achievable Forward and Strafe Deceleration Identifiers");
        Inputs.Field<Double> velocity = velocityInput.d("Velocity").withDefault(30.0);
        awaitInputs(velocityInput);

        double forwardDeceleration = step("naturalForwardDeceleration", new ForwardDeceleration(localizerFunction, drivetrainFunction, velocity.get()));
        double strafeDeceleration = step("naturalStrafeDeceleration", new StrafeDeceleration(localizerFunction, drivetrainFunction, velocity.get()));

        List<Double> headingBraking = step("headingBrake [linear, quadratic]", new HeadingBraking(localizerFunction, drivetrainFunction));
        double heading = step("heading kP", new HeadingTuner(localizerFunction, drivetrainFunction));

        double headingLinear = headingBraking.get(0);
        double headingQuadratic = headingBraking.get(1);

        Inputs distanceBrakingInput = inputs("Distance", "The distance to drive in inches for the Forward and Strafe Braking Identifiers. Distance must be at least 15 inches for accurate results.");
        Inputs.Field<Double> distanceBraking = distanceBrakingInput.d("Distance").withDefault(36.0);
        awaitInputs(distanceBrakingInput);
        double safeDistanceBraking = Math.max(distanceBraking.get(), 15.0);

        List<Double> forwardBraking = step("forwardBrake [linear, quadratic]", new ForwardBraking(localizerFunction, drivetrainFunction, headingLinear, headingQuadratic, heading, safeDistanceBraking));
        List<Double> strafeBraking = step("strafeBrake [linear, quadratic]", new StrafeBraking(localizerFunction, drivetrainFunction, headingLinear, headingQuadratic, heading, safeDistanceBraking));

        double forwardLinear = forwardBraking.get(0);
        double forwardQuadratic = forwardBraking.get(1);
        double strafeLinear = strafeBraking.get(0);
        double strafeQuadratic = strafeBraking.get(1);

        List<Double> forwardTranslational = step("forwardTranslational [primary kP, secondary kP, coast kV, brake kV]", new ForwardTranslational(localizerFunction, drivetrainFunction));
        List<Double> strafeTranslational = step("strafeTranslational [primary kP, secondary kP]", new StrafeTranslational(localizerFunction, drivetrainFunction));

        double forwardTranslationalPrimary = forwardTranslational.get(0);
        double forwardTranslationalSecondary = forwardTranslational.get(1);
        double coast = forwardTranslational.get(2);
        double brake = forwardTranslational.get(3);

        double strafeTranslationalPrimary = strafeTranslational.get(0);
        double strafeTranslationalSecondary = strafeTranslational.get(1);

        result("maxAchievableForwardVelocity", forwardVelocity);
        result("maxAchievableStrafeVelocity", strafeVelocity);
        result("naturalForwardDeceleration", forwardDeceleration);
        result("naturalStrafeDeceleration", strafeDeceleration);
        result("headingBrakingLinearCoefficient", headingLinear);
        result("headingBrakingQuadraticCoefficient", headingQuadratic);
        result("heading kP", heading);
        result("forwardBrakingLinearCoefficient", forwardLinear);
        result("forwardBrakingQuadraticCoefficient", forwardQuadratic);
        result("strafeBrakingLinearCoefficient", strafeLinear);
        result("strafeBrakingQuadraticCoefficient", strafeQuadratic);
        result("forwardTranslational Primary kP", forwardTranslationalPrimary);
        result("forwardTranslational Secondary kP", forwardTranslationalSecondary);
        result("strafeTranslational Primary kP", strafeTranslationalPrimary);
        result("strafeTranslational Secondary kP", strafeTranslationalSecondary);
        result("coast kV", coast);
        result("brake kV", brake);

        code(Language.JAVA,
        "public static ForesightConfig foresightConfig = new ForesightConfig(\n" +
                "            c -> {\n" +
                "                Controller primaryTranslationalForward = Controller.proportional("+forwardTranslationalPrimary+");\n" +
                "                Controller secondaryTranslationalForward = Controller.proportional("+forwardTranslationalSecondary+");\n" +
                "                Controller primaryTranslationalLateral = Controller.proportional("+strafeTranslationalPrimary+");\n" +
                "                Controller secondaryTranslationalLateral = Controller.proportional("+strafeTranslationalSecondary+");\n" +
                "\n" +
                "                c.forwardTranslational.set(Controller.piecewise(secondaryTranslationalForward).put(2.5, primaryTranslationalForward));\n" +
                "                c.strafeTranslational.set(Controller.piecewise(secondaryTranslationalLateral).put(2.5, primaryTranslationalLateral));\n" +
                "\n" +
                "                c.coast.set(Controller.proportionalFeedforward("+coast+"));\n" +
                "                c.brake.set(Controller.proportionalFeedforward("+brake+"));\n" +
                "\n" +
                "                c.headingFeedback.set(Controller.proportional("+heading+"));\n" +
                "                c.headingBrakeCoefficients.set(Vector2D.cartesian("+headingLinear+", "+headingQuadratic+"));\n" +
                "\n" +
                "                c.linearBrakeCoefficients.set(Matrix.diag("+forwardLinear+", "+strafeLinear+"));\n" +
                "                c.quadraticBrakeCoefficients.set(Matrix.diag("+forwardQuadratic+", "+strafeQuadratic+"));\n" +
                "\n" +
                "                c.maxAchievableForwardVelocity.set("+forwardVelocity+");\n" +
                "                c.maxAchievableStrafeVelocity.set("+strafeVelocity+");\n" +
                "                c.naturalForwardDeceleration.set("+forwardDeceleration+");\n" +
                "                c.naturalStrafeDeceleration.set("+strafeDeceleration+");\n" +
                "            }\n" +
                "    );");
    }
}

/**
 * Hard limits for the full-power Foresight steps. Upstream, these loops end only when the localizer
 * reports the target distance or speed. They never check time, total travel, or Stop, so a bad Pinpoint
 * reading (axes swapped, not updating, wrong pod type) sent the robot into the wall at full power.
 * Every driving loop below must call {@link #check} and also stop on {@code isStopRequested()}.
 */
final class DriveGuard {
    /** At full power, the robot must have moved at least this far by now, or the localizer is not tracking. */
    static final double STALL_SECONDS = 0.75;
    static final double STALL_INCHES = 2.0;

    private final ElapsedTime timer = new ElapsedTime();
    private final double maxInches;
    private final double maxSeconds;
    private final boolean checkStall;

    DriveGuard(double maxInches, double maxSeconds, boolean checkStall) {
        this.maxInches = maxInches;
        this.maxSeconds = maxSeconds;
        this.checkStall = checkStall;
    }

    void reset() {
        timer.reset();
    }

    /** Brakes and aborts the tuner if the robot is past the time or distance limit (pose starts at zero). */
    void check(Localizer localizer, Drivetrain drivetrain) {
        double traveled = localizer.pose().toVector2D().magnitude();
        double seconds = timer.seconds();
        String problem = null;
        if (checkStall && seconds > STALL_SECONDS && traveled < STALL_INCHES) {
            problem = String.format(Locale.US, "localizer reports only %.1f in after %.1f s of driving. Is the Pinpoint connected, named 'pinpoint', and updating?", traveled, seconds);
        } else if (traveled > maxInches) {
            problem = String.format(Locale.US, "robot is %.1f in from the start, past the %.0f in limit", traveled, maxInches);
        } else if (seconds > maxSeconds) {
            problem = String.format(Locale.US, "drive phase timed out after %.1f s", maxSeconds);
        }
        if (problem != null) {
            drivetrain.stop(true);
            throw new IllegalStateException("Safety stop: " + problem);
        }
    }

    /**
     * Call after a step's loops. Pressing Stop ends the loops early, and upstream then returned a half-measured
     * number (or an empty list, which crashed the procedure) and went straight on to the next full-power step.
     */
    static void failIfStopped(LinearOpMode opMode, Drivetrain drivetrain) {
        if (opMode.isStopRequested()) {
            drivetrain.stop(true);
            throw new IllegalStateException("stopped from the Driver Station before the step finished");
        }
    }

    /** Forward motion must show up mostly in x and strafe motion mostly in y, otherwise the pods are swapped. */
    static void checkAxis(Localizer localizer, Drivetrain drivetrain, boolean forward) {
        double along = Math.abs(forward ? localizer.pose().x() : localizer.pose().y());
        double across = Math.abs(forward ? localizer.pose().y() : localizer.pose().x());
        if (along + across > 12 && across > along) {
            drivetrain.stop(true);
            throw new IllegalStateException(String.format(Locale.US,
                    "Safety stop: robot drove %s, but the localizer shows x=%.1f, y=%.1f. Check the Pinpoint X/Y pod ports and directions (run the localization test first).",
                    forward ? "forward" : "left", localizer.pose().x(), localizer.pose().y()));
        }
    }
}

class ForwardVelocity extends TuningOpMode<Double> {
    Function<HardwareMap, Localizer> localizerFunction;
    Function<HardwareMap, Drivetrain> drivetrainFunction;
    double distance;
    private final ArrayDeque<Double> velocities = new ArrayDeque<>();
    public static double RECORD_NUMBER = 10;

    public ForwardVelocity(Function<HardwareMap, Localizer> localizerFunction, Function<HardwareMap, Drivetrain> drivetrainFunction, double distance) {
        super("Max Forward Velocity", "A tuner for finding the maximum achievable forward velocity. This will drive forward for " + distance + " inches and then likely drift past that position.", false);
        this.localizerFunction = localizerFunction;
        this.drivetrainFunction = drivetrainFunction;
        this.distance = distance;
    }

    @Override
    protected Double runTuningOpMode() throws InterruptedException {
        Localizer localizer = localizerFunction.apply(hardwareMap);
        Drivetrain drivetrain = drivetrainFunction.apply(hardwareMap);

        boolean end = false;

        localizer.setPose(Pose.zero());
        localizer.update();

        DrivePowers power = new DrivePowers(1,0,0);

        for (int i = 0; i < RECORD_NUMBER; i++) {
            velocities.add(0.0);
        }

        Thread.sleep(1000);
        waitForStart();
        localizer.setPose(Pose.zero());
        localizer.update();

        DriveGuard guard = new DriveGuard(distance + 6, 1.0 + distance / 20.0, true);
        guard.reset();

        while (!end && !isStopRequested()) {
            localizer.update();
            DriveGuard.checkAxis(localizer, drivetrain, true);
            guard.check(localizer, drivetrain);
            if (Math.abs(localizer.pose().x()) > distance) {
                end = true;
                drivetrain.stop(true);
            } else {
                drivetrain.drive(power, true);
                double currentVelocity = Math.abs(localizer.twist().toVector2D().x());
                velocities.addLast(currentVelocity);
                velocities.removeFirst();
            }
        }

        drivetrain.stop();
        DriveGuard.failIfStopped(this, drivetrain);
        double average = 0;
        for (double velocity : velocities) {
                average += velocity;
        }
        average /= velocities.size();
        return average;
    }
}

class StrafeVelocity extends TuningOpMode<Double> {
    Function<HardwareMap, Localizer> localizerFunction;
    Function<HardwareMap, Drivetrain> drivetrainFunction;
    double distance;
    private final ArrayDeque<Double> velocities = new ArrayDeque<>();
    public static double RECORD_NUMBER = 10;

    public StrafeVelocity(Function<HardwareMap, Localizer> localizerFunction, Function<HardwareMap, Drivetrain> drivetrainFunction, double distance) {
        super("Max Strafe Velocity", "A tuner for finding the maximum achievable strafe velocity. This will drive left for " + distance + " inches and then likely drift past that position.", false);
        this.localizerFunction = localizerFunction;
        this.drivetrainFunction = drivetrainFunction;
        this.distance = distance;
    }

    @Override
    protected Double runTuningOpMode() throws InterruptedException {
        Localizer localizer = localizerFunction.apply(hardwareMap);
        Drivetrain drivetrain = drivetrainFunction.apply(hardwareMap);

        boolean end = false;

        localizer.setPose(Pose.zero());
        localizer.update();

        DrivePowers power = new DrivePowers(0,1,0);

        for (int i = 0; i < RECORD_NUMBER; i++) {
            velocities.add(0.0);
        }

        Thread.sleep(1000);
        waitForStart();
        localizer.setPose(Pose.zero());
        localizer.update();

        DriveGuard guard = new DriveGuard(distance + 6, 1.0 + distance / 15.0, true);
        guard.reset();

        while (!end && !isStopRequested()) {
            localizer.update();
            DriveGuard.checkAxis(localizer, drivetrain, false);
            guard.check(localizer, drivetrain);
            if (Math.abs(localizer.pose().y()) > distance) {
                end = true;
                drivetrain.stop(true);
            } else {
                drivetrain.drive(power, false);
                double currentVelocity = Math.abs(localizer.twist().toVector2D().y());
                velocities.addLast(currentVelocity);
                velocities.removeFirst();
            }
        }

        drivetrain.stop();
        DriveGuard.failIfStopped(this, drivetrain);
        double average = 0;
        for (double velocity : velocities) {
            average += velocity;
        }
        average /= velocities.size();
        return average;
    }
}

class ForwardDeceleration extends TuningOpMode<Double> {
    Function<HardwareMap, Localizer> localizerFunction;
    Function<HardwareMap, Drivetrain> drivetrainFunction;
    double velocity;

    private final ArrayList<Double> accelerations = new ArrayList<>();
    /** If the target speed isn't reached within this distance, the localizer is wrong: brake instead of continuing. */
    public static double MAX_ACCEL_INCHES = 48;

    private double previousVelocity;
    private long previousTimeNano;
    private boolean stopping;

    public ForwardDeceleration(Function<HardwareMap, Localizer> localizerFunction, Function<HardwareMap, Drivetrain> drivetrainFunction, double velocity) {
        super("Forward Deceleration", "A tuner for finding the deceleration of the robot when moving forward. This will move forward until it reaches " + velocity + " inches per second.", false);

        this.localizerFunction = localizerFunction;
        this.drivetrainFunction = drivetrainFunction;
        this.velocity = velocity;
    }

    @Override
    protected Double runTuningOpMode() throws InterruptedException {
        Localizer localizer = localizerFunction.apply(hardwareMap);
        Drivetrain drivetrain = drivetrainFunction.apply(hardwareMap);

        accelerations.clear();
        previousVelocity = 0;
        previousTimeNano = 0;
        stopping = false;

        localizer.setPose(Pose.zero());
        localizer.update();

        DrivePowers power = new DrivePowers(1, 0, 0);
        Thread.sleep(1000);
        waitForStart();
        localizer.setPose(Pose.zero());
        localizer.update();

        DriveGuard guard = new DriveGuard(MAX_ACCEL_INCHES, 3.0, true);
        guard.reset();
        drivetrain.drive(power, false);

        while (!stopping && !isStopRequested()) {
            localizer.update();
            DriveGuard.checkAxis(localizer, drivetrain, true);
            guard.check(localizer, drivetrain);
            double currentVelocity = localizer.twist().toVector2D().x();
            if (Math.abs(currentVelocity) > velocity) {
                previousVelocity = currentVelocity;
                previousTimeNano = System.nanoTime();

                stopping = true;
                drivetrain.stop(false);
            }
        }

        boolean end = false;
        ElapsedTime coastTimer = new ElapsedTime();

        // Motors are already off here; this only measures the coast-down, so give up after a few seconds.
        while (!end && !isStopRequested() && coastTimer.seconds() < 4.0) {
            localizer.update();
            double currentVelocity = localizer.twist().toVector2D().x();
            long currentTimeNano = System.nanoTime();
            double dt = (currentTimeNano - previousTimeNano) / 1e9;

            if (dt > 0) {
                double acceleration = (currentVelocity - previousVelocity) / dt;
                accelerations.add(acceleration);
            }

            previousVelocity = currentVelocity;
            previousTimeNano = currentTimeNano;

            if (Math.abs(currentVelocity) <= 1) {
                end = true;
            }
        }

        drivetrain.stop(false);
        DriveGuard.failIfStopped(this, drivetrain);

        double average = 0;

        for (double acceleration : accelerations) {
            average += acceleration;
        }

        if (accelerations.isEmpty()) {
            return 0.0;
        }

        average /= accelerations.size();

        return Math.abs(average);
    }
}

class StrafeDeceleration extends TuningOpMode<Double> {
    Function<HardwareMap, Localizer> localizerFunction;
    Function<HardwareMap, Drivetrain> drivetrainFunction;
    double velocity;

    private final ArrayList<Double> accelerations = new ArrayList<>();
    /** If the target speed isn't reached within this distance, the localizer is wrong: brake instead of continuing. */
    public static double MAX_ACCEL_INCHES = 48;

    private double previousVelocity;
    private long previousTimeNano;
    private boolean stopping;

    public StrafeDeceleration(Function<HardwareMap, Localizer> localizerFunction, Function<HardwareMap, Drivetrain> drivetrainFunction, double velocity) {
        super("Strafe Deceleration", "A tuner for finding the deceleration of the robot when moving laterally. This will drive left until it reaches " + velocity + " inches per second.", false);

        this.localizerFunction = localizerFunction;
        this.drivetrainFunction = drivetrainFunction;
        this.velocity = velocity;
    }

    @Override
    protected Double runTuningOpMode() throws InterruptedException {
        Localizer localizer = localizerFunction.apply(hardwareMap);
        Drivetrain drivetrain = drivetrainFunction.apply(hardwareMap);

        accelerations.clear();
        previousVelocity = 0;
        previousTimeNano = 0;
        stopping = false;

        localizer.setPose(Pose.zero());
        localizer.update();

        DrivePowers power = new DrivePowers(0, 1, 0);
        Thread.sleep(1000);
        waitForStart();
        localizer.setPose(Pose.zero());
        localizer.update();

        DriveGuard guard = new DriveGuard(MAX_ACCEL_INCHES, 3.0, true);
        guard.reset();
        drivetrain.drive(power, false);

        while (!stopping && !isStopRequested()) {
            localizer.update();
            DriveGuard.checkAxis(localizer, drivetrain, false);
            guard.check(localizer, drivetrain);
            double currentVelocity = localizer.twist().toVector2D().y();
            if (Math.abs(currentVelocity) > velocity) {
                previousVelocity = currentVelocity;
                previousTimeNano = System.nanoTime();

                stopping = true;
                drivetrain.stop(false);
            }
        }

        boolean end = false;
        ElapsedTime coastTimer = new ElapsedTime();

        // Motors are already off here; this only measures the coast-down, so give up after a few seconds.
        while (!end && !isStopRequested() && coastTimer.seconds() < 4.0) {
            localizer.update();
            double currentVelocity = localizer.twist().toVector2D().y();
            long currentTimeNano = System.nanoTime();
            double dt = (currentTimeNano - previousTimeNano) / 1e9;

            if (dt > 0) {
                double acceleration = (currentVelocity - previousVelocity) / dt;
                accelerations.add(acceleration);
            }

            previousVelocity = currentVelocity;
            previousTimeNano = currentTimeNano;

            if (Math.abs(currentVelocity) <= 1) {
                end = true;
            }
        }

        drivetrain.stop(false);
        DriveGuard.failIfStopped(this, drivetrain);

        double average = 0;

        for (double acceleration : accelerations) {
            average += acceleration;
        }

        if (accelerations.isEmpty()) {
            return 0.0;
        }

        average /= accelerations.size();

        return Math.abs(average);
    }
}

class HeadingBraking extends TuningOpMode<List<Double>> {
    Function<HardwareMap, Localizer> localizerFunction;
    Function<HardwareMap, Drivetrain> drivetrainFunction;

    private static double[] POWERS;
    public static double MAX_BRAKE_TIME = 3; //seconds, the robot shouldn't take longer than this to brake

    public static int trials = 12;
    public static double maxPower = 1;
    public static double minPower = 0.2;
    public static double bias = 1.5; // how much it favors doing trials with higher powers
    public static double brakingPower = 0.001;

    private final ElapsedTime timer = new ElapsedTime();

    private final List<double[]> velocityToBrakingDistance = new ArrayList<>();
    private State state = State.DRIVE;
    private int iteration = 0;
    private int direction;
    private double power;

    private double startHeading;
    private double measuredVelocity;
    private double totalHeading;
    private double previousHeading;
//    private VoltageSensor voltageSensor;

    public HeadingBraking(Function<HardwareMap, Localizer> localizerFunction, Function<HardwareMap, Drivetrain> drivetrainFunction) {
        super("Heading Braking", "A tuner for finding the Heading Braking Coefficients. The robot will turn back at forth at various speed levels.", false);

        this.localizerFunction = localizerFunction;
        this.drivetrainFunction = drivetrainFunction;
    }

    @Override
    protected List<Double> runTuningOpMode() throws InterruptedException {
        Localizer localizer = localizerFunction.apply(hardwareMap);
        Drivetrain drivetrain = drivetrainFunction.apply(hardwareMap);

        localizer.setPose(Pose.zero());
        localizer.update();

        List<Double> coefficients = Collections.emptyList();

        POWERS = biasedGradient(trials, maxPower, minPower, bias);

        Thread.sleep(1000);
        waitForStart();
        localizer.setPose(Pose.zero());
        localizer.update();
        timer.reset();

        while (state != State.DONE && !isStopRequested()) {
            localizer.update();
            double currentHeading = localizer.pose().heading();
            totalHeading += Angle.normalizeSigned(currentHeading - previousHeading);
            previousHeading = currentHeading;

            direction = (iteration % 2 == 0) ? 1 : -1;
            if (iteration < POWERS.length) {
                power = POWERS[iteration];
            }

//            if (state != State.DONE) {
//                double voltage = voltageSensor.getVoltage();
//                double duty = state == State.BRAKE ? -brakingPower * direction: power * direction;
//                double appliedVoltage = voltage * duty;
//            }

            switch (state) {
                case DRIVE: {
                    if (timer.seconds() > 2) {
                        startHeading = totalHeading;
                        measuredVelocity = Math.abs(localizer.velocity().omega);

                        drivetrain.drive(new DrivePowers(0.0, 0.0, -brakingPower * direction), false);
                        state = State.BRAKE;
                        timer.reset();
                        break;
                    }
                    drivetrain.drive(new DrivePowers(0.0, 0.0, power * direction), false);
                    break;
                }
                case BRAKE: {
                    if (Math.abs(localizer.velocity().omega) > 0.001 && timer.seconds() < MAX_BRAKE_TIME) {
                        drivetrain.drive(new DrivePowers(0.0, 0.0, -brakingPower * direction), false);
                        break;
                    }

                    double endHeading = totalHeading;
                    double brakingDistance = Math.abs(endHeading - startHeading);

                    velocityToBrakingDistance.add(new double[]{measuredVelocity, brakingDistance});

                    iteration++;

                    if (iteration >= POWERS.length) {
                        drivetrain.stop();

                        double[] c = quadraticFit(velocityToBrakingDistance);
                        coefficients = List.of(c[0], c[1]);

                        state = State.DONE;
                    } else {
                        timer.reset();
                        state = State.DRIVE;
                    }
                    break;
                }
                case DONE: {}
            }
        }

        DriveGuard.failIfStopped(this, drivetrain);

        return coefficients;
    }

    private enum State {
        DRIVE,
        BRAKE,
        DONE
    }

    private static double[] biasedGradient(
            int count,
            double max,
            double min,
            double bias
    ) {
        if (count < 2) return new double[]{  max};

        double[] values = new double[count];

        for (int i = 0; i < count; i++) {
            double t = (double) i / (count - 1);

            double curved = 1 - Math.pow(t, bias);

            values[i] = min + curved * (max - min);
        }

        return values;
    }
}

class HeadingTuner extends TuningOpMode<Double> {
    Function<HardwareMap, Localizer> localizerFunction;
    Function<HardwareMap, Drivetrain> drivetrainFunction;

    private static final double POWER = 0.4;
    private static final double RUNTIME = 1.2;
    private static final int SAMPLES = 15;
    public static double ALPHA = 18.25;

    private double tau;
    private double K;
    private double kV;
    private double kA;
    private double vMax = 0;
    private final List<Double> times = new ArrayList<>();
    private final List<Double> velocities = new ArrayList<>();
    private final ElapsedTime timer = new ElapsedTime();
    private boolean done = false;
    private double lastTime = 0.0;

    public HeadingTuner(Function<HardwareMap, Localizer> localizerFunction, Function<HardwareMap, Drivetrain> drivetrainFunction) {
        super("Heading Tuner", "A tuner for finding the Heading Tuning Coefficients using system identification. This will spin the robot in place for a couple seconds.", false);
        this.localizerFunction = localizerFunction;
        this.drivetrainFunction = drivetrainFunction;
    }

    @Override
    protected Double runTuningOpMode() throws InterruptedException {
        Localizer localizer = localizerFunction.apply(hardwareMap);
        Drivetrain drivetrain = drivetrainFunction.apply(hardwareMap);

        localizer.setPose(Pose.zero());
        localizer.update();

        times.clear();
        velocities.clear();
        done = false;
        vMax = 0;
        lastTime = 0.0;

        Thread.sleep(1000);
        waitForStart();
        localizer.setPose(Pose.zero());
        localizer.update();
        timer.reset();
        lastTime = timer.seconds();
        drivetrain.drive(new DrivePowers(0.0, 0.0, POWER), false);

        while (!done && !isStopRequested()) {
            double now = timer.seconds();
            double dt = now - lastTime;
            if (dt <= 0) dt = 1e-6;
            lastTime = now;

            localizer.update();

            if (!done) {
                times.add(timer.seconds());

                double turnVel = Math.abs(localizer.velocity().omega);
                vMax = Math.max(vMax, turnVel / POWER);

                velocities.add(turnVel);

                if (timer.seconds() >= RUNTIME) {
                    done = true;
                    systemIdentification();
                    drivetrain.drive(new DrivePowers(0.0, 0.0, 0.0), false);
                }
            }
        }

        drivetrain.drive(new DrivePowers(0.0, 0.0, 0.0), true);
        DriveGuard.failIfStopped(this, drivetrain);
        return calculatekP(ALPHA);
    }

    private double calculatekP(double alpha) {
        kV = 1 / K;
        kA = tau / K;
        return tau * alpha * alpha / K;
    }

    private void systemIdentification() {
        int N = times.size();
        if (N < 4) {
            throw new IllegalArgumentException("Failed calibration.");
        }

        int start = Math.max(0, N - SAMPLES);
        double samples = N - start;
        double sum = 0;
        for (int i = start; i < N; i++) sum += velocities.get(i);
        double A = sum / samples;
        this.K = A / POWER;

        List<Double> y = new ArrayList<>();
        List<Double> x = new ArrayList<>();
        for (int i = 0; i < N; i++) {
            double vel = velocities.get(i) / POWER;
            if (vel > 0.8 * K) continue;
            if (vel < 0.1 * K) continue;
            y.add(Math.log(K - vel));
            x.add(times.get(i));
        }
        double[] linReg = linearFit(
                x.toArray(new Double[0]),
                y.toArray(new Double[0])
        );
        if (linReg[1] == 0) throw new IllegalArgumentException("Failed calibration.");
        this.tau = -1.0/linReg[1];
    }
}

class ForwardBraking extends TuningOpMode<List<Double>> {
    Function<HardwareMap, Localizer> localizerFunction;
    Function<HardwareMap, Drivetrain> drivetrainFunction;
    private final double headingLinear;
    private final double headingQuadratic;
    private final double headingKP;

    private double[] POWERS;
    public double MAX_BRAKE_TIME = 7.0;
    public int trials = 5;
    public double maxPower = 0.7;
    public double minPower = 0.3;
    public double bias = 1.5;
    public double brakingPower = 0.001;
    public double distance;
    public double IDLE_SECONDS = 1;
    /** Allowed overshoot past the run distance (and to the side) before a safety stop. Coasting from top speed is ~15 in. */
    public double SAFETY_MARGIN = 30;
    /**
     * Time limit for one run at 0.7 power or more. Slower runs get proportionally longer (see checkSafety):
     * a fixed 5 s could cut off the 0.3-power runs, which are slow when strafing. A localizer that is not
     * tracking is caught earlier by the stall check there.
     */
    public double MAX_DRIVE_SECONDS = 5;
    private final ElapsedTime driveTimer = new ElapsedTime();
    private Vector2D runStart = Vector2D.cartesian(0, 0);

    private final ElapsedTime timer = new ElapsedTime();
    private final List<double[]> velocityToBrakingDistance = new ArrayList<>();
    private State state = State.DRIVE;
    private int iteration = 0;
    private int direction;
    private double power;
    private Vector2D startPosition;
    private double measuredVelocity;

    public ForwardBraking(Function<HardwareMap, Localizer> localizerFunction, Function<HardwareMap, Drivetrain> drivetrainFunction,
                          double headingLinear, double headingQuadratic, double headingKP, double distance) {
        super("Forward Braking", "A tuner for finding the Forward Braking Coefficients by driving forward and backward at various speeds. Please ensure that you have plenty of room at least " + distance + " inches ahead of the robot, but also tile space behind and laterally around the robot.", false);
        this.localizerFunction = localizerFunction;
        this.drivetrainFunction = drivetrainFunction;
        this.headingLinear = headingLinear;
        this.headingQuadratic = headingQuadratic;
        this.headingKP = headingKP;
        this.distance = distance;
    }

    @Override
    protected List<Double> runTuningOpMode() throws InterruptedException {
        Localizer localizer = localizerFunction.apply(hardwareMap);
        Drivetrain drivetrain = drivetrainFunction.apply(hardwareMap);

        localizer.setPose(Pose.zero());
        localizer.update();

        POWERS = biasedGradient(trials, maxPower, minPower, bias);

        List<Double> coefficients = Collections.emptyList();

        Thread.sleep(1000);
        waitForStart();
        localizer.setPose(Pose.zero());
        localizer.update();
        timer.reset();

        drivetrain.drive(new DrivePowers(maxPower,0,0), false);

        driveTimer.reset();

        while (state != State.DONE && !isStopRequested()) {
            localizer.update();
            checkSafety(localizer, drivetrain);
            direction = (iteration % 2 == 0) ? 1 : -1;
            if (iteration < POWERS.length) {
                power = POWERS[iteration];
            }

            switch (state) {
                case DRIVE: {
                    if ((direction > 0 && Math.abs(localizer.pose().x()) >= distance) || (direction < 0 && Math.abs(localizer.pose().x()) <= 12)) {
                        startPosition = localizer.pose().toVector2D();
                        measuredVelocity = localizer.velocity().toVector2D().magnitude();

                        brake(drivetrain, localizer);
                        state = State.BRAKE;
                        timer.reset();
                        break;
                    }
                    drive(drivetrain, localizer);
                    break;
                }
                case BRAKE: {
                    if (localizer.velocity().toVector2D().magnitude() > 0.25 && timer.seconds() < MAX_BRAKE_TIME) {
                        brake(drivetrain, localizer);
                        break;
                    }

                    collectTrialData(localizer, drivetrain);
                    break;
                }
                case WAIT: {
                    drivetrain.stop();
                    if (timer.seconds() > IDLE_SECONDS) {
                        state = State.DRIVE;
                        driveTimer.reset();
                        runStart = localizer.pose().toVector2D();
                    }
                    break;
                }
                case DONE: {}
            }
        }

        drivetrain.stop(true);
        DriveGuard.failIfStopped(this, drivetrain);

        if (state == State.DONE) {
            double[] c = quadraticFit(velocityToBrakingDistance);
            coefficients = List.of(c[0], c[1]);
        }

        return coefficients;
    }

    private double getHeadingPower(Localizer localizer) {
        double angularVel = localizer.velocity().omega;
        double brakeDist = headingLinear * angularVel +
                headingQuadratic * angularVel * angularVel * Math.signum(angularVel);
        double headingError = Angle.normalizeSigned(-localizer.pose().heading());
        double error = headingError - brakeDist;
        return Utils.clamp(headingKP * error, -0.3, 1.0) / 2;
    }


    private void drive(Drivetrain drivetrain, Localizer localizer) {
        drivetrain.drive(new DrivePowers(power * direction, 0.0, getHeadingPower(localizer)), false);
    }

    /** Keeps the back-and-forth runs inside the run distance plus SAFETY_MARGIN; see {@link DriveGuard}. */
    private void checkSafety(Localizer localizer, Drivetrain drivetrain) {
        double x = localizer.pose().x();
        double y = localizer.pose().y();
        String problem = null;
        if (Math.abs(x) > distance + SAFETY_MARGIN || Math.abs(y) > SAFETY_MARGIN) {
            problem = String.format(Locale.US, "robot at x=%.1f, y=%.1f, outside %.0f in forward / %.0f in sideways", x, y, distance + SAFETY_MARGIN, SAFETY_MARGIN);
        } else if (state == State.DRIVE) {
            double seconds = driveTimer.seconds();
            double moved = localizer.pose().toVector2D().minus(runStart).magnitude();
            double maxSeconds = MAX_DRIVE_SECONDS * Math.max(1.0, 0.7 / Math.max(power, 0.1));
            if (seconds > 2 * DriveGuard.STALL_SECONDS && moved < DriveGuard.STALL_INCHES) {
                problem = String.format(Locale.US, "only %.1f in of movement after %.1f s at power %.2f. Either the localizer is not tracking or the robot cannot move at this power", moved, seconds, power);
            } else if (seconds > maxSeconds) {
                problem = String.format(Locale.US, "drive run at power %.2f took over %.0f s (x=%.1f)", power, maxSeconds, x);
            }
        }
        if (problem != null) {
            drivetrain.stop(true);
            throw new IllegalStateException("Safety stop: " + problem);
        }
    }

    private void brake(Drivetrain drivetrain, Localizer localizer) {
        double headingPower = getHeadingPower(localizer);
        double brake = -brakingPower * direction;
        double minBrake = Math.abs(headingPower) + 0.001;

        if (direction > 0) {
            brake = Math.min(brake, -minBrake);
        } else {
            brake = Math.max(brake, minBrake);
        }

        drivetrain.drive(new DrivePowers(brake, 0, headingPower), false);
    }

    private void collectTrialData(Localizer localizer, Drivetrain drivetrain) {
        Vector2D endPosition = localizer.pose().toVector2D();
        double brakingDistance = endPosition.minus(startPosition).magnitude();

        velocityToBrakingDistance.add(new double[]{measuredVelocity, brakingDistance});

        iteration++;

        if (iteration >= POWERS.length) {
            drivetrain.stop();
            state = State.DONE;
        } else {
            state = State.WAIT;
            timer.reset();
        }
    }

    private enum State {
        DRIVE,
        BRAKE,
        WAIT,
        DONE
    }

    private static double[] biasedGradient(int count, double max, double min, double bias) {
        if (count < 2) return new double[]{max};
        double[] values = new double[count];
        for (int i = 0; i < count; i++) {
            double t = (double) i / (count - 1);
            double curved = 1 - Math.pow(t, bias);
            values[i] = min + curved * (max - min);
        }
        return values;
    }
}

class StrafeBraking extends TuningOpMode<List<Double>> {
    Function<HardwareMap, Localizer> localizerFunction;
    Function<HardwareMap, Drivetrain> drivetrainFunction;
    private final double headingLinear;
    private final double headingQuadratic;
    private final double headingKP;

    private double[] POWERS;
    public double MAX_BRAKE_TIME = 7.0;
    public  int trials = 5;
    public double maxPower = 1;
    public double minPower = 0.3;
    public double bias = 1.5;
    public double brakingPower = 0.001;
    public double distance;
    public double IDLE_SECONDS = 1;
    /** Allowed overshoot past the run distance (and to the side) before a safety stop. Coasting from top speed is ~15 in. */
    public double SAFETY_MARGIN = 30;
    /**
     * Time limit for one run at 0.7 power or more. Slower runs get proportionally longer (see checkSafety):
     * a fixed 5 s could cut off the 0.3-power runs, which are slow when strafing. A localizer that is not
     * tracking is caught earlier by the stall check there.
     */
    public double MAX_DRIVE_SECONDS = 5;
    private final ElapsedTime driveTimer = new ElapsedTime();
    private Vector2D runStart = Vector2D.cartesian(0, 0);

    private final ElapsedTime timer = new ElapsedTime();
    private final List<double[]> velocityToBrakingDistance = new ArrayList<>();
    private State state = State.DRIVE;
    private int iteration = 0;
    private int direction;
    private double power;
    private Vector2D startPosition;
    private double measuredVelocity;

    public StrafeBraking(Function<HardwareMap, Localizer> localizerFunction, Function<HardwareMap, Drivetrain> drivetrainFunction,
                         double headingLinear, double headingQuadratic, double headingKP, double distance) {
        super("Strafe Braking", "A tuner for finding the Strafe Braking Coefficients by strafing left and right at various speeds. Please ensure that you have plenty of room at least " + distance + " inches to the left and right of the robot and space in front and behind the robot.", false);
        this.localizerFunction = localizerFunction;
        this.drivetrainFunction = drivetrainFunction;
        this.headingLinear = headingLinear;
        this.headingQuadratic = headingQuadratic;
        this.headingKP = headingKP;
        this.distance = distance;
    }

    @Override
    protected List<Double> runTuningOpMode() throws InterruptedException {
        Localizer localizer = localizerFunction.apply(hardwareMap);
        Drivetrain drivetrain = drivetrainFunction.apply(hardwareMap);

        localizer.setPose(Pose.zero());
        localizer.update();

        POWERS = biasedGradient(trials, maxPower, minPower, bias);

        List<Double> coefficients = Collections.emptyList();

        Thread.sleep(1000);
        waitForStart();
        localizer.setPose(Pose.zero());
        localizer.update();
        timer.reset();

        drivetrain.drive(new DrivePowers(0,maxPower,0), false);

        driveTimer.reset();

        while (state != State.DONE && !isStopRequested()) {
            localizer.update();
            checkSafety(localizer, drivetrain);
            direction = (iteration % 2 == 0) ? 1 : -1;
            if (iteration < POWERS.length) {
                power = POWERS[iteration];
            }

            switch (state) {
                case DRIVE: {
                    if ((direction > 0 && Math.abs(localizer.pose().y()) > distance) ||
                            (direction < 0 && Math.abs(localizer.pose().y()) <= 6)) {
                        startPosition = localizer.pose().toVector2D();
                        measuredVelocity = localizer.velocity().toVector2D().magnitude();

                        brake(drivetrain, localizer);
                        state = State.BRAKE;
                        timer.reset();
                        break;
                    }
                    drive(drivetrain, localizer);
                    break;
                }
                case BRAKE: {
                    if (localizer.velocity().toVector2D().magnitude() > 0.25 && timer.seconds() < MAX_BRAKE_TIME) {
                        brake(drivetrain, localizer);
                        break;
                    }

                    collectTrialData(localizer, drivetrain);
                    break;
                }
                case WAIT: {
                    drivetrain.stop();
                    if (timer.seconds() > IDLE_SECONDS) {
                        state = State.DRIVE;
                        driveTimer.reset();
                        runStart = localizer.pose().toVector2D();
                    }
                    break;
                }
                case DONE: {}
            }
        }

        drivetrain.stop(true);
        DriveGuard.failIfStopped(this, drivetrain);

        if (state == State.DONE) {
            double[] c = quadraticFit(velocityToBrakingDistance);
            coefficients = List.of(c[0], c[1]);
        }

        return coefficients;
    }

    private double getHeadingPower(Localizer localizer) {
        double angularVel = localizer.velocity().omega;
        double brakeDist = headingLinear * angularVel +
                headingQuadratic * angularVel * angularVel * Math.signum(angularVel);
        double headingError = Angle.normalizeSigned(-localizer.pose().heading());
        double error = headingError - brakeDist;
        return Utils.clamp(headingKP * error, -0.3, 1.0) / 2;
    }

    private void drive(Drivetrain drivetrain, Localizer localizer) {
        drivetrain.drive(new DrivePowers(0.0, power * direction, getHeadingPower(localizer)), false);
    }

    /** Keeps the left-and-right runs inside the run distance plus SAFETY_MARGIN; see {@link DriveGuard}. */
    private void checkSafety(Localizer localizer, Drivetrain drivetrain) {
        double x = localizer.pose().x();
        double y = localizer.pose().y();
        String problem = null;
        if (Math.abs(y) > distance + SAFETY_MARGIN || Math.abs(x) > SAFETY_MARGIN) {
            problem = String.format(Locale.US, "robot at x=%.1f, y=%.1f, outside %.0f in sideways / %.0f in forward", x, y, distance + SAFETY_MARGIN, SAFETY_MARGIN);
        } else if (state == State.DRIVE) {
            double seconds = driveTimer.seconds();
            double moved = localizer.pose().toVector2D().minus(runStart).magnitude();
            double maxSeconds = MAX_DRIVE_SECONDS * Math.max(1.0, 0.7 / Math.max(power, 0.1));
            if (seconds > 2 * DriveGuard.STALL_SECONDS && moved < DriveGuard.STALL_INCHES) {
                problem = String.format(Locale.US, "only %.1f in of movement after %.1f s at power %.2f. Either the localizer is not tracking or the robot cannot move at this power", moved, seconds, power);
            } else if (seconds > maxSeconds) {
                problem = String.format(Locale.US, "drive run at power %.2f took over %.0f s (y=%.1f)", power, maxSeconds, y);
            }
        }
        if (problem != null) {
            drivetrain.stop(true);
            throw new IllegalStateException("Safety stop: " + problem);
        }
    }

    private void brake(Drivetrain drivetrain, Localizer localizer) {
        double headingPower = getHeadingPower(localizer);
        double brake = -brakingPower * direction;
        double minBrake = Math.abs(headingPower) + 0.001;

        if (direction > 0) {
            brake = Math.min(brake, -minBrake);
        } else {
            brake = Math.max(brake, minBrake);
        }

        drivetrain.drive(new DrivePowers(0, brake, headingPower), false);
    }

    private void collectTrialData(Localizer localizer, Drivetrain drivetrain) {
        Vector2D endPosition = localizer.pose().toVector2D();
        double brakingDistance = endPosition.minus(startPosition).magnitude();

        velocityToBrakingDistance.add(new double[]{measuredVelocity, brakingDistance});

        iteration++;

        if (iteration >= POWERS.length) {
            drivetrain.stop();
            state = State.DONE;
        } else {
            state = State.WAIT;
            timer.reset();
        }
    }

    private enum State {
        DRIVE,
        BRAKE,
        WAIT,
        DONE
    }

    private static double[] biasedGradient(int count, double max, double min, double bias) {
        if (count < 2) return new double[]{max};
        double[] values = new double[count];
        for (int i = 0; i < count; i++) {
            double t = (double) i / (count - 1);
            double curved = 1 - Math.pow(t, bias);
            values[i] = min + curved * (max - min);
        }
        return values;
    }
}

class ForwardTranslational extends TuningOpMode<List<Double>> {
    Function<HardwareMap, Localizer> localizerFunction;
    Function<HardwareMap, Drivetrain> drivetrainFunction;

    public static double ALPHA_LARGE = 10.2;
    public static double ALPHA_SMALL = 6.2;
    private final double VEL_AGGRESSIVENESS = 0.85;
    private final double POWER = 0.4;
    private final double RUNTIME = 1.2;
    private final int SAMPLES = 15;

    private double tau;
    private double K;
    private double kV;
    private double kA;
    private double vMax = 0;
    private final List<Double> times = new ArrayList<>();
    private final List<Double> velocities = new ArrayList<>();
    private final ElapsedTime timer = new ElapsedTime();
    private boolean done = false;
    private double lastTime = 0.0;

    public ForwardTranslational(Function<HardwareMap, Localizer> localizerFunction, Function<HardwareMap, Drivetrain> drivetrainFunction) {
        super("Forward Translational", "A tuner for finding the Forward Translational kP coefficients using system identification. This will move around 12-24 inches in front of the robot and then stop.", false);
        this.localizerFunction = localizerFunction;
        this.drivetrainFunction = drivetrainFunction;
    }

    @Override
    protected List<Double> runTuningOpMode() throws InterruptedException {
        Localizer localizer = localizerFunction.apply(hardwareMap);
        Drivetrain drivetrain = drivetrainFunction.apply(hardwareMap);

        localizer.setPose(Pose.zero());
        localizer.update();

        times.clear();
        velocities.clear();
        done = false;
        vMax = 0;
        lastTime = 0.0;

        Thread.sleep(1000);
        waitForStart();
        localizer.setPose(Pose.zero());
        localizer.update();
        timer.reset();
        lastTime = timer.seconds();
        drivetrain.drive(new DrivePowers(POWER, 0.0, 0.0), false);

        while (!done && !isStopRequested()) {
            double now = timer.seconds();
            double dt = now - lastTime;
            if (dt <= 0) dt = 1e-6;
            lastTime = now;

            localizer.update();

            if (!done) {
                times.add(timer.seconds());

                double forwardVelocity = Math.abs(localizer.twist().toVector2D().x());
                vMax = Math.max(vMax, forwardVelocity / POWER);

                velocities.add(forwardVelocity);

                if (timer.seconds() >= RUNTIME) {
                    done = true;
                    systemIdentification();
                    drivetrain.drive(new DrivePowers(0.0, 0.0, 0.0), false);
                }
            }
        }

        drivetrain.drive(new DrivePowers(0.0, 0.0, 0.0), true);
        DriveGuard.failIfStopped(this, drivetrain);

        double kP_large = calculatekP(ALPHA_LARGE);
        double kP_small = calculatekP(ALPHA_SMALL);

        //  kP_large, kP_small, coast kV, and brake kV (scaled by aggressiveness factor)
        return List.of(kP_large, kP_small, kV, kV * VEL_AGGRESSIVENESS);
    }

    private double calculatekP(double alpha) {
        kV = 1 / K;
        kA = tau / K;
        return tau * alpha * alpha / K;
    }

    private void systemIdentification() {
        int N = times.size();
        if (N < 4) {
            throw new IllegalArgumentException("Failed calibration.");
        }

        int start = Math.max(0, N - SAMPLES);
        double samples = N - start;
        double sum = 0;
        for (int i = start; i < N; i++) sum += velocities.get(i);
        double A = sum / samples;
        this.K = A / POWER;

        List<Double> y = new ArrayList<>();
        List<Double> x = new ArrayList<>();
        for (int i = 0; i < N; i++) {
            double vel = velocities.get(i) / POWER;
            if (vel > 0.8 * K) continue;
            if (vel < 0.1 * K) continue;
            y.add(Math.log(K - vel));
            x.add(times.get(i));
        }
        double[] linReg = linearFit(
                x.toArray(new Double[0]),
                y.toArray(new Double[0])
        );
        if (linReg[1] == 0) throw new IllegalArgumentException("Failed calibration.");
        this.tau = -1.0/linReg[1];
    }
}

class StrafeTranslational extends TuningOpMode<List<Double>> {
    Function<HardwareMap, Localizer> localizerFunction;
    Function<HardwareMap, Drivetrain> drivetrainFunction;
    public static double ALPHA_LARGE = 10.2;
    public static double ALPHA_SMALL = 6.2;
    private final double POWER = 0.4;
    private final double RUNTIME = 1.2;
    private final int SAMPLES = 15;

    private double tau;
    private double K;
    private double kV;
    private double kA;
    private double vMax = 0;
    private final List<Double> times = new ArrayList<>();
    private final List<Double> velocities = new ArrayList<>();
    private final ElapsedTime timer = new ElapsedTime();
    private boolean done = false;
    private double lastTime = 0.0;

    public StrafeTranslational(Function<HardwareMap, Localizer> localizerFunction, Function<HardwareMap, Drivetrain> drivetrainFunction) {
        super("Strafe Translational", "A tuner for finding the Strafe Translational kP coefficients using system identification. This will move around 12-24 inches to the left and right of the robot and then stop.", false);
        this.localizerFunction = localizerFunction;
        this.drivetrainFunction = drivetrainFunction;
    }

    @Override
    protected List<Double> runTuningOpMode() throws InterruptedException {
        Localizer localizer = localizerFunction.apply(hardwareMap);
        Drivetrain drivetrain = drivetrainFunction.apply(hardwareMap);

        localizer.setPose(Pose.zero());
        localizer.update();

        times.clear();
        velocities.clear();
        done = false;
        vMax = 0;
        lastTime = 0.0;

        Thread.sleep(1000);
        waitForStart();
        localizer.setPose(Pose.zero());
        localizer.update();
        timer.reset();
        lastTime = timer.seconds();
        drivetrain.drive(new DrivePowers(0.0, POWER, 0.0), false);

        while (!done && !isStopRequested()) {
            double now = timer.seconds();
            double dt = now - lastTime;
            if (dt <= 0) dt = 1e-6;
            lastTime = now;

            localizer.update();

            if (!done) {
                times.add(timer.seconds());

                double lateralVelocity = Math.abs(localizer.twist().toVector2D().y());
                vMax = Math.max(vMax, lateralVelocity / POWER);

                velocities.add(lateralVelocity);

                if (timer.seconds() >= RUNTIME) {
                    done = true;
                    systemIdentification();
                    drivetrain.drive(new DrivePowers(0.0, 0.0, 0.0), false);
                }
            }
        }

        drivetrain.drive(new DrivePowers(0.0, 0.0, 0.0), true);
        DriveGuard.failIfStopped(this, drivetrain);

        double kP_large = calculatekP(ALPHA_LARGE);
        double kP_small = calculatekP(ALPHA_SMALL);

        return List.of(kP_large, kP_small);
    }

    private double calculatekP(double alpha) {
        kV = 1 / K;
        kA = tau / K;
        return tau * alpha * alpha / K;
    }

    private void systemIdentification() {
        int N = times.size();
        if (N < 4) {
            throw new IllegalArgumentException("Failed calibration.");
        }

        int start = Math.max(0, N - SAMPLES);
        double samples = N - start;
        double sum = 0;
        for (int i = start; i < N; i++) sum += velocities.get(i);
        double A = sum / samples;
        this.K = A / POWER;

        List<Double> y = new ArrayList<>();
        List<Double> x = new ArrayList<>();
        for (int i = 0; i < N; i++) {
            double vel = velocities.get(i) / POWER;
            if (vel > 0.8 * K) continue;
            if (vel < 0.1 * K) continue;
            y.add(Math.log(K - vel));
            x.add(times.get(i));
        }
        double[] linReg = linearFit(
                x.toArray(new Double[0]),
                y.toArray(new Double[0])
        );
        if (linReg[1] == 0) throw new IllegalArgumentException("Failed calibration.");
        this.tau = -1.0/linReg[1];
    }
}

