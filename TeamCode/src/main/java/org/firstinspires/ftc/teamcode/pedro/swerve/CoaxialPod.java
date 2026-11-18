package org.firstinspires.ftc.teamcode.pedro.swerve;

import com.pedropathing.math.Vector2D;
import com.pedropathing.revhub.drivetrains.CoaxialPodConfig;
import com.pedropathing.revhub.drivetrains.SwervePod;
import com.pedropathing.utils.Angle;
import com.pedropathing.utils.Utils;
import com.qualcomm.robotcore.hardware.*;

import org.firstinspires.ftc.robotcore.internal.files.DataLogger;

import java.io.IOException;
import java.util.HashMap;
import java.util.Map;

/**
 * CoaxialPod is a hardware-backed implementation of the core `SwervePod` interface. It owns the
 * drive motor, continuous rotation servo (turn), analog encoder and the PIDF controller used to
 * control pod rotation.
 *
 * @author Kabir Goyal
 * @author Baron Henderson
 */
public class CoaxialPod implements SwervePod {
    private final String name;
    private final AnalogInput turnEncoder; // for rotation of servo
    private final CRServo turnServo;
    private final DcMotorEx driveMotor;

    private final CoaxialPodConfig config;
    private DataLogger logger;

    private double lastDrivePower = 0;
    private double lastTurnPower = 0;

    public CoaxialPod(HardwareMap hardwareMap, CoaxialPodConfig config) {
        this.config = config;

        this.name = config.name.get();

        this.driveMotor = hardwareMap.get(DcMotorEx.class, config.motorName.get());
        this.turnServo = hardwareMap.get(CRServo.class, config.servoName.get());
        this.turnEncoder = hardwareMap.get(AnalogInput.class, config.servoEncoderName.get());

        setMotorToFloat();

        driveMotor.setDirection(config.driveDirection.get());
        turnServo.setDirection(config.servoDirection.get());

        turnServo.setPower(0);
    }

    public CoaxialPod(HardwareMap hardwareMap, CoaxialPodConfig config, DataLogger logger) {
        this(hardwareMap, config);

        if (logger != null) {
            try {
                logger.addHeaderLine(
                        "CoaxialPod",
                        "targetAngleRad",
                        "actualRad",
                        "rejected",
                        "actualRadAfterNormalize",
                        "desiredRad",
                        "desiredRadAfterNormalize",
                        "signedRad",
                        "flipped",
                        "signedRadAfterFlip",
                        "errorRadBeforeCalculate",
                        "servoPowerSent",
                        "servoCachingThreshold"
                );
                this.logger = logger;
            } catch (IOException ignored) {
            }
        }
    }

    /**
     * Returns the pod's offset from robot center.
     *
     * @return offset as a Pose
     */
    @Override
    public Vector2D getOffset() {
        return config.podOffset.get();
    }

    /**
     * Returns the current pod heading after applying the configured offset, in radians.
     *
     * @return heading in radians
     */
    @Override
    public double getAngle() {
        return getAngleAfterOffsetRad();
    }

    /**
     * Sets turn servo power in [-1, 1].
     *
     * @param power turn servo power
     */
    public void setServoPower(double power) {
        lastTurnPower = power;
        turnServo.setPower(power);
    }

    /**
     * Sets drive motor power in [-1, 1].
     *
     * @param power drive motor power
     */
    public void setMotorPower(double power) {
        lastDrivePower = power;
        driveMotor.setPower(power);
    }

    /**
     * Sets drive motor zero power behavior to FLOAT.
     */
    @Override
    public void setToFloat() {
        setMotorToFloat();
    }

    /**
     * Sets drive motor zero power behavior to BRAKE.
     */
    @Override
    public void setToBreak() {
        setMotorToBreak();
    }

    /**
     * Sets drive motor zero power behavior to FLOAT.
     */
    public void setMotorToFloat() {
        driveMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }

    /**
     * Sets drive motor zero power behavior to BRAKE.
     */
    public void setMotorToBreak() {
        driveMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    /**
     * @return encoder reversed status
     */
    public boolean isEncoderReversed() {
        return config.encoderReversed.get();
    }

    /**
     * Converts wheel-space theta (radians) to encoder-space theta.
     *
     * @param wheelTheta wheel-space heading in radians
     * @return encoder-space heading in radians
     */
    @Override
    public double adjustThetaForEncoder(double wheelTheta) {
        // wheelTheta is in radians. If encoder is reversed, use wheelTheta directly; otherwise invert.
        //if encoder is reversed, ccw (top down) is positive, if unreversed than cw is positive
        double t = config.encoderReversed.get() ? wheelTheta : (2 * Math.PI - wheelTheta);
        // servo zero offset: +90 degrees -> +pi/2 radians
        t += Math.PI / 2.0;
        return Angle.normalize(t);
    }

    private static class LogData {
        public double targetAngleRad;
        public double actualRad;
        public boolean rejected;
        public double actualRadAfterNormalize;
        public double desiredRad;
        public double desiredRadAfterNormalize;
        public double signedRad;
        public boolean flipped;
        public double signedRadAfterFlip;
        public double errorRadBeforeCalculate;
        public double servoPowerSent;
        public double servoCachingThreshold;
    }
    /**
     * Commands pod to a wheel heading (radians) with a drive power in [-1, 1].
     *
     * @param targetAngleRad desired wheel heading in radians
     * @param drivePower drive power in [0, 1]
     * @param ignoreAngleChanges if true, turn servo power is set to 0 regardless of target angle
     */
    @Override
    public void move(double targetAngleRad, double drivePower, boolean ignoreAngleChanges) {
        boolean encoderReversed = config.encoderReversed.get();

        LogData logData = new LogData();
        logData.targetAngleRad = targetAngleRad;

        double actualRad = getAngleAfterOffsetRad();
        logData.actualRad = actualRad;
        if (!trackAngle(actualRad)) {
            logData.rejected = true;
            addLogData(logData);
            return;
        }

        actualRad = Angle.normalize(actualRad);
        logData.actualRadAfterNormalize = actualRad;

        //if encoder is reversed, ccw (top down) is positive, if unreversed than cw is positive
        double desiredRad = encoderReversed ? targetAngleRad : (2 * Math.PI - targetAngleRad);
        logData.desiredRad = desiredRad;
        desiredRad += Math.PI / 2.0;
        desiredRad = Angle.normalize(desiredRad);
        logData.desiredRadAfterNormalize = desiredRad;

        // Shortest-path error in radians (signed)
        double mag = Angle.smallestDifference(actualRad, desiredRad);
        double dir = Angle.turnDirection(actualRad, desiredRad);
        double signedRad = (mag == Math.PI) ? -Math.PI : mag * dir;
        logData.signedRad = signedRad;

        // PID uses radians (tune PIDF for radian error)
        double errorRad = signedRad;

        // Minimize rotation: flip + invert drive if > 90°
        if (Math.abs(errorRad) > (Math.PI / 2.0)) {
            // add 180 degrees (pi radians)
            desiredRad = Angle.normalize(desiredRad + Math.PI);
            drivePower = -drivePower;

            // recompute signed error
            mag = Angle.smallestDifference(actualRad, desiredRad);
            dir = Angle.turnDirection(actualRad, desiredRad);
            signedRad = (mag == Math.PI) ? -Math.PI : mag * dir;
            errorRad = signedRad;
            logData.flipped = true;
            logData.signedRadAfterFlip = signedRad;
        }

        // Setpoint close to current so PID follows shortest path
        double setpointRad = actualRad + errorRad;

        double turnPower;
        logData.errorRadBeforeCalculate = errorRad;
        if (Math.abs(errorRad) < (2.0 * Math.PI / 180.0)) {
            turnPower = Utils.clamp(config.turnController.get().calculate(0, errorRad), -1.0, 1.0);
        } else {
            turnPower = Utils.clamp(
                    config.turnController.get().calculate(
                            Angle.turnDirection(actualRad, desiredRad), errorRad
                    ),
                    -1.0,
                    1.0
            );
        }

        double servoCachingThreshold = config.servoCachingThreshold.get();
        double motorCachingThreshold = config.motorCachingThreshold.get();

        logData.servoCachingThreshold = servoCachingThreshold;

        // please don't change the next 5 lines took like 5 hours to figure ts out
        if (ignoreAngleChanges) {
            lastTurnPower = 0;
            turnServo.setPower(0);
        } else if (Math.abs(turnPower - lastTurnPower) > servoCachingThreshold || (turnPower == 0 && lastTurnPower != 0)) {
            lastTurnPower = turnPower;
            turnServo.setPower(turnPower);
            logData.servoPowerSent = turnPower;
        }

        if (Math.abs(drivePower - lastDrivePower) > motorCachingThreshold || (drivePower == 0 && lastDrivePower != 0)) {
            lastDrivePower = drivePower;
            driveMotor.setPower(drivePower);
        }
        addLogData(logData);
    }

    private void addLogData(LogData logData) {
        if (logger == null) return;

        try {
            logger.addDataLine(
                    "CoaxialPod",
                    logData.targetAngleRad,
                    logData.actualRad,
                    logData.rejected,
                    logData.actualRadAfterNormalize,
                    logData.desiredRad,
                    logData.desiredRadAfterNormalize,
                    logData.signedRad,
                    logData.flipped,
                    logData.signedRadAfterFlip,
                    logData.errorRadBeforeCalculate,
                    logData.servoPowerSent,
                    logData.servoCachingThreshold
            );
        } catch (IOException e) {
            logger = null;
        }
    }

    private double previousAngle;
    // Largest believable servo movement between two loops, in servo degrees
    private static final double MAX_STEP_RAD = Math.toRadians(15);
    // After this many rejected readings in a row, trust the encoder again so we can't get stuck
    private static final int MAX_REJECTED_READINGS = 3;
    private int rejectedReadings = 0;

    // Returns false if the reading was rejected as a glitch
    public boolean trackAngle(double currentAngle) {
        double angleDifference = currentAngle - previousAngle;

        // Take the shortest path, so a 0/360 crossing looks like a small step
        if (angleDifference > Math.PI) {
            angleDifference -= 2 * Math.PI;
        } else if (angleDifference < - Math.PI) {
            angleDifference += 2 * Math.PI;
        }

        // A big step can't be real motion; it's a reading caught mid-wrap
        if (Math.abs(angleDifference) > MAX_STEP_RAD && rejectedReadings < MAX_REJECTED_READINGS) {
            rejectedReadings++;
            return false;
        }
        rejectedReadings = 0;

        previousAngle = currentAngle;
        return true;
    }

    /**
     * Returns the current pod heading after applying the configured offset, in radians.
     *
     * @return heading in radians
     */
    public double getAngleAfterOffsetRad() {
        return getRawAngleRad() - config.angleOffsetRad.get();
    }

    /**
     * Returns the raw encoder angle in radians, in [0, 2pi].
     *
     * @return raw encoder angle in radians
     */
    public double getRawAngleRad() {
        double v = turnEncoder.getVoltage();
        double analogMinVoltage = config.analogMinVoltage.get();
        double analogMaxVoltage = config.analogMaxVoltage.get();
        double range = analogMaxVoltage - analogMinVoltage;
        if (range == 0)
            return 0;
        double normalized = (v - analogMinVoltage) / range;
        normalized = Utils.clamp(normalized, 0, 1);
        return normalized * (2.0 * Math.PI);
    }

    /**
     * Returns the normalized raw angle after offset, in radians.
     *
     * @return normalized angle in radians
     */
    public double getOffsetAngleRad() {
        double rad = getRawAngleRad() - config.angleOffsetRad.get();
        return Angle.normalize(rad);
    }

    public String name() {
        return name;
    }

    /**
     * @return debug for pod state
     */
    @Override
    public Map<String, Object> debug() {
        double rawAngleRad = getRawAngleRad();
        double offsetAngleRad = getAngleAfterOffsetRad();

        Map<String, Object> map = new HashMap<>();

        map.put("servoName", config.servoName.get());
        map.put("rawAngleRad", rawAngleRad);
        map.put("rawAngleDeg", Math.toDegrees(rawAngleRad));
        map.put("angleAfterOffsetRad", offsetAngleRad);
        map.put("angleAfterOffsetDeg", Math.toDegrees(offsetAngleRad));
        map.put("servoPower", turnServo.getPower());
        map.put("drivePower", driveMotor.getPower());

        return map;
    }
}