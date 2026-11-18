package org.firstinspires.ftc.teamcode.pedro.swerve;

import com.pedropathing.controllers.Controller;

import org.firstinspires.ftc.robotcore.internal.files.DataLogger;

import java.io.IOException;

public class SwervePIDFController implements Controller {
    private static final double KF_DEADBAND_RAD = Math.toRadians(1.5);
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
    private DataLogger logger;

    public SwervePIDFController(double kP, double kI, double kD, double kF, double kA) {
        setPIDF(kP, kI, kD, kF, kA);
    }

    public SwervePIDFController(double kP, double kI, double kD, double kF, double kA, DataLogger logger) {
        this(kP, kI, kD, kF, kA);

        if (logger == null) return;

        try {
            logger.addHeaderLine("SwervePIDFController", "kP", "kI", "kD", "kF", "kA", "error", "dt", "currentDerivative", "previousDerivative", "filteredDerivative", "integral", "result");
            this.logger = logger;
        } catch (IOException ignored) {
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
    }

    private void logData(LogData logData) {
        if (logger == null) return;

        try {
            logger.addDataLine(
                    "SwervePIDFController",
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
                    logData.result
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

    public double calculate(double target, double error) {
        LogData logData = new LogData();
        logData.error = error;

        long currentTime = System.nanoTime();
        double dt = (currentTime - previousTime) * 1e-9;
        previousTime = currentTime;
        logData.dt = dt;

        if (dt <= 1e-3) {
            double result = error * kP + integral * kI + Math.signum(error) * kF;
            logData.integral = integral;
            logData.result = result;
            logData(logData);
            previousError = error;
            return result;
        }

        double filteredDerivative = 0.0F;
        if (!firstUpdate) {
            double currentDerivative = (error - previousError) / dt;
            filteredDerivative = kA * currentDerivative + (1.0-kA) * previousDerivative;

            logData.currentDerivative = currentDerivative;
            logData.previousDerivative = previousDerivative;
            logData.filteredDerivative = filteredDerivative;

            previousDerivative = filteredDerivative;
            integral += error * dt;

            logData.integral = integral;
        }

        previousError = error;
        firstUpdate = false;
        double ff = Math.abs(error) > KF_DEADBAND_RAD ? Math.signum(error) * kF : 0.0;
        double result = error * kP + integral * kI + filteredDerivative * kD + ff;
        logData.result = result;
        logData(logData);
        return result;
    }

    public double getPreviousError() {
        return previousError;
    }

    public void reset() {
        integral = 0.0F;
        previousError = 0.0F;
        previousDerivative = 0.0F;
        previousTime = System.nanoTime();
        firstUpdate = true;
    }
}
