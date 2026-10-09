package org.firstinspires.ftc.teamcode.pedro.swerve;

import com.pedropathing.controllers.Controller;
import com.pedropathing.revhub.drivetrains.SwervePod;

import org.firstinspires.ftc.teamcode.pid.PIDController;

public class SwervePIDController implements Controller {
    private final PIDController internalController;
    public SwervePIDController(double kP, double kI, double kD) {
        this.internalController = new PIDController(kP, kI, kD);
    }

    @Override
    public double calculate(double target, double error) {
        return internalController.calculate(target-error, target);
    }

    public void setPID(double kp, double ki, double kd) {
        this.internalController.setPID(kp, ki, kd);
    }
}
