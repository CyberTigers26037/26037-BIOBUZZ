//
// Source code recreated from a .class file by IntelliJ IDEA
// (powered by Fernflower decompiler)
//

package org.firstinspires.ftc.teamcode.pedro.swerve;

import com.pedropathing.controllers.Controller;

public class PIDControllerWithKd implements Controller {
    public double kP;
    public double kI;
    public double kD;
    private double integral = (double)0.0F;
    private double previousError = (double)0.0F;
    private long previousTime = System.nanoTime();
    private boolean firstUpdate = true;

    public PIDControllerWithKd(double kP, double kI, double kD) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
    }

    public double calculate(double target, double error) {
        long currentTime = System.nanoTime();
        double dt = (double)(currentTime - this.previousTime);// * 1.0E-9;
        this.previousTime = currentTime;
        //if (dt <= 0.001) {
        //    return error * this.kP + this.integral * this.kI;
        //} else {
            this.integral += error * dt;
            double derivative = (double)0.0F;
            if (!this.firstUpdate) {
                derivative = (error - this.previousError) / dt;
            }

            this.previousError = error;
            this.firstUpdate = false;
            return error * this.kP + this.integral * this.kI + derivative * this.kD;
        //}
    }

    public double calculate(double target, double error, double velocity) {
        long currentTime = System.nanoTime();
        double dt = (double)(currentTime - this.previousTime);// * 1.0E-9;
        this.previousTime = currentTime;
        //if (dt <= 0.001) {
        //    return error * this.kP + this.integral * this.kI;
        //} else {
            this.integral += error * dt;
            this.previousError = error;
            this.firstUpdate = false;
            return error * this.kP + this.integral * this.kI - velocity * this.kD;
        //}
    }

    public void reset() {
        this.integral = (double)0.0F;
        this.previousError = (double)0.0F;
        this.previousTime = System.nanoTime();
        this.firstUpdate = true;
    }
}
