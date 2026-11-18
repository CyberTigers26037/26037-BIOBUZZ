package org.firstinspires.ftc.teamcode.pedro.swerve;

import com.pedropathing.controllers.Controller;

public class SwervePIDFController implements Controller {
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

    public SwervePIDFController(double kP, double kI, double kD, double kF, double kA) {
        setPIDF(kP, kI, kD, kF, kA);
    }

    public void setPIDF(double kP, double kI, double kD, double kF, double kA) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.kF = kF;
        this.kA = kA;
    }

    public double calculate(double target, double error) {
        long currentTime = System.nanoTime();
        double dt = (currentTime - previousTime) * 1e-9;
        previousTime = currentTime;

        if (dt <= 1e-3) {
            return error * kP + integral * kI + Math.signum(error) * kF;
        }

        double filteredDerivative = 0.0F;
        if (!firstUpdate) {
            double currentDerivative = (error - previousError) / dt;
            filteredDerivative = kA * currentDerivative + (1.0-kA) * previousDerivative;
            previousDerivative = filteredDerivative;
            integral += error * dt;
        }

        previousError = error;
        firstUpdate = false;
        return error * kP + integral * kI + filteredDerivative * kD + Math.signum(error) * kF;
    }

    public void reset() {
        integral = 0.0F;
        previousError = 0.0F;
        previousDerivative = 0.0F;
        previousTime = System.nanoTime();
        firstUpdate = true;
    }
}
