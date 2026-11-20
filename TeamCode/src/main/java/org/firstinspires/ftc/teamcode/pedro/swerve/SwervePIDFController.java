package org.firstinspires.ftc.teamcode.pedro.swerve;

import com.pedropathing.controllers.Controller;

import org.firstinspires.ftc.robotcore.internal.files.DataLogger;

import java.io.IOException;

public class SwervePIDFController implements Controller {
    // F is 0 inside the deadband and ramps up to full kF by KF_RAMP_END_RAD. Switching F on all
    // at once near the target acts like an on/off switch and makes the pod oscillate.
    private static final double KF_DEADBAND_RAD = Math.toRadians(1.5);
    private static final double KF_RAMP_END_RAD = Math.toRadians(5);
    // If the encoder hasn't reported a new value for this long, assume the pod has stopped
    // instead of holding on to the last speed we saw
    private static final double MAX_STALE_SECONDS = 0.1;
    private final String instanceId;
    public double kP;
    public double kI;
    public double kD;
    public double kF;
    public double kA;
    private double integral = 0.0;
    private double previousError = 0.0;
    private double previousDerivative = 0.0;
    private long previousTime = System.nanoTime();
    private boolean firstUpdate = true;
    // Last encoder reading that was different from the one before it, and when it arrived
    private double previousMeasurement = 0.0;
    private long previousMeasurementTime = System.nanoTime();
    private DataLogger logger;

    public SwervePIDFController(String instanceId, double kP, double kI, double kD, double kF, double kA) {
        this.instanceId = instanceId;
        setPIDF(kP, kI, kD, kF, kA);
    }

    public SwervePIDFController(String instanceId, double kP, double kI, double kD, double kF, double kA, DataLogger logger) {
        this(instanceId, kP, kI, kD, kF, kA);

        setDataLogger(logger);
    }

    public void setDataLogger(DataLogger logger) {
        this.logger = logger;
        if (logger == null) return;

        try {
            logger.addHeaderLine("SwervePIDFController", "instanceId", "kP", "kI", "kD", "kF", "kA", "error", "dt", "currentDerivative", "previousDerivative", "filteredDerivative", "integral", "result", "measurementRad", "newReading", "measurementDt");
        } catch (IOException ignored) {
            this.logger = null;
        }
    }

    private static class LogData {
        public double error;
        public double dt;
        public double currentDerivative;
        public double previousDerivative;
        public double filteredDerivative;
        public double integral;
        public double result;
        public double measurement;
        public boolean newReading;
        public double measurementDt;
    }

    private void logData(LogData logData) {
        if (logger == null) return;

        try {
            logger.addDataLine(
                    "SwervePIDFController",
                    instanceId,
                    kP,
                    kI,
                    kD,
                    kF,
                    kA,
                    logData.error,
                    logData.dt,
                    logData.currentDerivative,
                    logData.previousDerivative,
                    logData.filteredDerivative,
                    logData.integral,
                    logData.result,
                    logData.measurement,
                    logData.newReading,
                    logData.measurementDt
            );
        } catch (IOException e) {
            logger = null;
        }
    }

    public void setPIDF(double kP, double kI, double kD, double kF, double kA) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.kF = kF;
        this.kA = kA;
    }

    /**
     * Without a measurement, fall back to taking the derivative of the error. Since
     * error = target - measurement, -error moves exactly like the measurement while the
     * target holds still. Target changes and flips will cause a derivative kick here.
     */
    public double calculate(double target, double error) {
        return calculate(target, error, -error);
    }

    /**
     * PIDF with "derivative on measurement": D is computed from how fast the pod is actually
     * turning, not how fast the error is changing. Changing the target or flipping the pod
     * makes the error jump, but the pod itself didn't move, so D stays calm.
     *
     * @param target unused, kept for the Controller interface
     * @param error target - measurement, shortest path, in radians
     * @param measurementRad current pod angle in radians
     */
    @Override
    public double calculate(double target, double error, double measurementRad) {
        LogData logData = new LogData();
        logData.error = error;
        logData.measurement = measurementRad;

        long currentTime = System.nanoTime();
        double dt = (currentTime - previousTime) * 1e-9;
        previousTime = currentTime;
        logData.dt = dt;

        if (dt <= 1e-3) {
            double result = error * kP + integral * kI + feedforward(error);
            logData.integral = integral;
            logData.result = result;
            logData(logData);
            previousError = error;
            return result;
        }

        double filteredDerivative = 0.0F;
        if (firstUpdate) {
            previousMeasurement = measurementRad;
            previousMeasurementTime = currentTime;
        } else {
            // The encoder often reports the same value for a few loops before updating. Only
            // a new value tells us how far the pod moved, and it moved that far over the whole
            // time since the last new value, not just this loop's dt.
            double measurementDt = (currentTime - previousMeasurementTime) * 1e-9;
            logData.measurementDt = measurementDt;

            double currentDerivative = 0.0;
            if (measurementRad != previousMeasurement) {
                double delta = measurementRad - previousMeasurement;
                // Take the shortest path, so a 0/360 crossing looks like a small step
                if (delta > Math.PI) {
                    delta -= 2 * Math.PI;
                } else if (delta < -Math.PI) {
                    delta += 2 * Math.PI;
                }

                // error = target - measurement, so the error shrinks as the measurement grows
                currentDerivative = -delta / measurementDt;
                previousMeasurement = measurementRad;
                previousMeasurementTime = currentTime;
                logData.newReading = true;
                filteredDerivative = kA * currentDerivative + (1.0 - kA) * previousDerivative;
            } else if (measurementDt > MAX_STALE_SECONDS) {
                // No new reading for a long time: the pod has stopped, let D fade toward 0
                filteredDerivative = (1.0 - kA) * previousDerivative;
            } else {
                // Same reading as last loop: no new information, keep the speed we had
                filteredDerivative = previousDerivative;
            }

            logData.currentDerivative = currentDerivative;
            logData.previousDerivative = previousDerivative;
            logData.filteredDerivative = filteredDerivative;

            previousDerivative = filteredDerivative;
            integral += error * dt;

            logData.integral = integral;
        }

        previousError = error;
        firstUpdate = false;
        double result = error * kP + integral * kI + filteredDerivative * kD + feedforward(error);
        logData.result = result;
        logData(logData);
        return result;
    }

    /**
     * The "minimum push" to overcome friction. 0 within KF_DEADBAND_RAD of the target, full kF
     * beyond KF_RAMP_END_RAD, and a straight line in between.
     */
    private double feedforward(double error) {
        double ramp = (Math.abs(error) - KF_DEADBAND_RAD) / (KF_RAMP_END_RAD - KF_DEADBAND_RAD);
        ramp = Math.max(0.0, Math.min(1.0, ramp));
        return Math.signum(error) * kF * ramp;
    }

    public double getPreviousError() {
        return previousError;
    }

    public void reset() {
        integral = 0.0F;
        previousError = 0.0F;
        previousDerivative = 0.0F;
        previousTime = System.nanoTime();
        previousMeasurementTime = previousTime;
        firstUpdate = true;
    }
}
