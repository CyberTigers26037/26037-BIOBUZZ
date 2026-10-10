package org.firstinspires.ftc.teamcode.pedro.swerve;

import com.pedropathing.controllers.Controller;

public class SwervePIDFController implements Controller {
    public double kP;
    public double kI;
    public double kD;
    public double kF;
    public double kA;
    private double integral = 0.0F;
    private double previousError = 0.0F;
    private double previousDerivative = 0.0F;
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
        double dt = (double)(currentTime - this.previousTime) * 1.0E-9;
        this.previousTime = currentTime;
        if (dt <= 0.001) {
            return error * this.kP + this.integral * this.kI + Math.signum(error) * this.kF;
        } else {
            double filteredDerivative = 0.0F;
            if (!this.firstUpdate) {
                double currentDerivative = (error - this.previousError) / dt;
                filteredDerivative = kA * currentDerivative + (1.0-kA)*this.previousDerivative;
                this.previousDerivative = filteredDerivative;
                this.integral += error * dt;
            }

            this.previousError = error;
            this.firstUpdate = false;
            return error * this.kP + this.integral * this.kI + filteredDerivative * this.kD + Math.signum(error) * this.kF;
        }
    }

    public void reset() {
        this.integral = 0.0F;
        this.previousError = 0.0F;
        this.previousDerivative = 0.0F;
        this.previousTime = System.nanoTime();
        this.firstUpdate = true;
    }
}
