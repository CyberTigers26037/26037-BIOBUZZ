package org.firstinspires.ftc.teamcode.servos;

import android.annotation.SuppressLint;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.util.ElapsedTime;

@SuppressWarnings("unused")
public class RTPAxon {
    // Encoder for servo position feedback
    private final AnalogInput servoEncoder;
    // Continuous rotation servo
    private final CRServo servo;
    // Whether to move to the closest angle to the requested angle
    private final boolean useClosestAngle;
    private final String logPrefix;

    // Current power applied to servo
    private double power;
    // Maximum allowed power
    private double maxPower;
    // Last measured angle
    private double previousAngle;
    // Accumulated rotation in degrees
    private double totalRotation;
    // Target rotation in degrees
    private double targetRotation;

    // PID controller coefficients and state
    private double kP;
    private double kI;
    private double kD;
    private double integralSum;
    private double lastError;
    private double maxIntegralSum;
    private ElapsedTime pidTimer;

    // Initialization and debug fields
    public int cliffs = 0;
    public double homeAngle;

    // Basic constructor, defaults to FORWARD direction
    public RTPAxon(CRServo servo, AnalogInput encoder, double homeAngle, boolean useClosestAngle, String logPrefix) {
        this.servo = servo;
        this.servoEncoder = encoder;
        this.homeAngle = homeAngle;
        this.useClosestAngle = useClosestAngle;
        this.logPrefix = logPrefix;
        initialize();
    }

    // Initialization logic for servo and encoder
    private void initialize() {
        servo.setPower(0);
        try {
            Thread.sleep(50);
        } catch (InterruptedException ignored) {
        }

        previousAngle = getCurrentAngle();
        totalRotation = 0;

        // Default PID coefficients
        kP = 0.015;
        kI = 0.0005;
        kD = 0.0025;
        integralSum = 0.0;
        lastError = 0.0;
        maxIntegralSum = 100.0;
        pidTimer = new ElapsedTime();
        pidTimer.reset();

        maxPower = 0.25;
        cliffs = 0;
    }

    // Set power to servo, respecting direction and maxPower
    public synchronized void setPower(double power) {
        this.power = Math.max(-maxPower, Math.min(maxPower, power));
        servo.setPower(this.power);
    }

    // Get current power
    public double getPower() {
        return power;
    }

    // Set maximum allowed power
    public void setMaxPower(double maxPower) {
        this.maxPower = maxPower;
    }

    // Get maximum allowed power
    public double getMaxPower() {
        return maxPower;
    }

    // Set PID P coefficient
    public void setKP(double kP) {
        if (kP < 0) kP = 0;
        this.kP = kP;
    }

    // Set PID I coefficient and reset integral
    public void setKI(double kI) {
        if (kI < 0) kI = 0;
        this.kI = kI;
        resetIntegral();
    }

    // Set PID D coefficient
    public void setKD(double kD) {
        if (kD < 0) kD = 0;
        this.kD = kD;
    }

    // Set all PID coefficients
    public void setPidCoeffs(double kP, double kI, double kD){
        setKP(kP);
        setKI(kI);
        setKD(kD);
    }

    // Get PID P coefficient
    public double getKP() {
        return kP;
    }

    // Get PID I coefficient
    public double getKI() {
        return kI;
    }

    // Get PID D coefficient
    public double getKD() {
        return kD;
    }

    // Set maximum allowed integral sum
    public void setMaxIntegralSum(double maxIntegralSum) {
        this.maxIntegralSum = maxIntegralSum;
    }

    // Get maximum allowed integral sum
    public double getMaxIntegralSum() {
        return maxIntegralSum;
    }

    // Get total rotation since initialization
    public double getTotalRotation() {
        return totalRotation;
    }

    // Get current target rotation
    public double getTargetRotation() {
        return targetRotation;
    }

    // Increment target rotation by a value
    public void changeTargetRotation(double change) {
        targetRotation += change;
    }

    private double getClosestAngle(double target) {
        int revolutionBaseline = ((int)targetRotation / 360) * 360;

        double possibleAngle1 = revolutionBaseline + target;
        double possibleAngle2 = revolutionBaseline + (target - 360);
        double possibleAngle3 = revolutionBaseline + (target + 360);
        double delta1 = Math.abs(targetRotation - possibleAngle1);
        double delta2 = Math.abs(targetRotation - possibleAngle2);
        double delta3 = Math.abs(targetRotation - possibleAngle3);
        if ((delta1 <= delta2) && (delta1 <= delta3)) {
            return possibleAngle1;
        }
        else if ((delta2 <= delta1) && (delta2 <= delta3)) {
            return possibleAngle2;
        }
        else {
            return possibleAngle3;
        }
    }

    // Set target rotation and reset PID
    public synchronized void setTargetRotation(double target) {
        targetRotation = useClosestAngle ? getClosestAngle(target) : target;
        //resetPID();
    }

    // Get current angle from encoder (in degrees)
    public double getCurrentAngle() {
        if (servoEncoder == null) return 0;
        return (servoEncoder.getVoltage() / servoEncoder.getMaxVoltage()) * 360;
    }

    // Check if servo is at target (default tolerance)
    public boolean isAtTarget() {
        return isAtTarget(5);
    }

    // Check if servo is at target (custom tolerance)
    public boolean isAtTarget(double tolerance) {
        return Math.abs(targetRotation - totalRotation) < tolerance;
    }

    // Force reset total rotation and PID state
    public void forceResetTotalRotation() {
        totalRotation = 0;
        previousAngle = getCurrentAngle();
        resetPID();
    }

    // Reset PID controller state
    public void resetPID() {
        resetIntegral();
        lastError = 0;
        pidTimer.reset();
    }

    // Reset integral sum
    public void resetIntegral() {
        integralSum = 0;
    }

    // Main update loop: updates rotation, computes PID, applies power
    public synchronized void update() {
        if ( !trackAngle() ) {
            return; //keep the power for this loop instead of reacting to a bad reading
        }

        double dt = pidTimer.seconds();
        pidTimer.reset();

        // Ignore unreasonable dt values
        if (dt < 0.001 || dt > 1.0) {
            return;
        }

        double error = targetRotation - totalRotation;

        // PID integral calculation with clamping
        integralSum += error * dt;
        integralSum = Math.max(-maxIntegralSum, Math.min(maxIntegralSum, integralSum));

        // Integral wind-down in deadzone
        final double INTEGRAL_DEADZONE = 2.0;
        if (Math.abs(error) < INTEGRAL_DEADZONE) {
            integralSum *= 0.95;
        }

        // PID derivative calculation
        double derivative = (error - lastError) / dt;
        lastError = error;

        // PID output calculation
        double pTerm = kP * error;
        double iTerm = kI * integralSum;
        double dTerm = kD * derivative;

        double output = pTerm + iTerm + dTerm;

        // Deadzone for output
        final double DEADZONE = 0.5;
        if (Math.abs(error) > DEADZONE) {
            double power = Math.min(maxPower, Math.abs(output)) * Math.signum(output);
            setPower(power);
        } else {
            setPower(0);
        }
    }

    public double lastHomingError;
    public synchronized void resetHomeAngle() {
        trackAngle();
        lastHomingError = totalRotation;
        homeAngle = getCurrentAngle();
        previousAngle = homeAngle;
        cliffs = 0;
        totalRotation = 0;
        targetRotation = 0;
        resetPID();
    }

    // Largest believable servo movement between two loops, in servo degrees
    private static final double MAX_STEP_DEG = 60;
    // After this many rejected readings in a row, trust the encoder again so we can't get stuck
    private static final int MAX_REJECTED_READINGS = 3;
    private int rejectedReadings = 0;

    // Returns false if the reading was rejected as a glitch
    public synchronized boolean trackAngle() {
        double currentAngle = getCurrentAngle();
        double angleDifference = currentAngle - previousAngle;

        // Take the shortest path, so a 0/360 crossing looks like a small step
        int wrap = 0;
        if (angleDifference > 180) {
            angleDifference -= 360;
            wrap = -1;
        } else if (angleDifference < -180) {
            angleDifference += 360;
            wrap = 1;
        }

        // A big step can't be real motion; it's a reading caught mid-wrap
        if (Math.abs(angleDifference) > MAX_STEP_DEG && rejectedReadings < MAX_REJECTED_READINGS) {
            rejectedReadings++;
            return false;
        }
        rejectedReadings = 0;

        cliffs += wrap;
        totalRotation = currentAngle - homeAngle + cliffs * 360;
        previousAngle = currentAngle;
        return true;
    }


    // Log current state for telemetry/debug
    @SuppressLint("DefaultLocale")
    public String log() {
        return String.format(
                logPrefix + ": Current Volts: %.3f\n" +
                logPrefix + ": Current Angle: %.2f\n" +
                logPrefix + ": Home Angle: %.2f\n" +
                logPrefix + ": Cliffs: %d\n" +
                logPrefix + ": Total Rotation: %.2f\n" +
                logPrefix + ": Target Rotation: %.2f\n" +
                logPrefix + ": Current Power: %.3f\n" +
                logPrefix + ": PID Values: P=%.3f I=%.3f D=%.3f\n" +
                logPrefix + ": PID Terms: Error=%.2f Integral=%.2f",
                servoEncoder.getVoltage(),
                getCurrentAngle(),
                homeAngle,
                cliffs,
                totalRotation,
                targetRotation,
                power,
                kP, kI, kD,
                targetRotation - totalRotation,
                integralSum
        );
    }
}