package org.firstinspires.ftc.teamcode.pid;

// Copied from FTCLib as-is from https://github.com/FTCLib/FTCLib/blob/master/core/src/main/java/com/arcrobotics/ftclib/controller/PIDController.java

public class PIDController extends PIDFController {

    /**
     * Default constructor with just the coefficients
     */
    public PIDController(double kp, double ki, double kd) {
        super(kp, ki, kd, 0);
    }

    /**
     * The extended constructor.
     */
    public PIDController(double kp, double ki, double kd, double sp, double pv) {
        super(kp, ki, kd, 0, sp, pv);
    }

    public void setPID(double kp, double ki, double kd) {
        setPIDF(kp, ki, kd, 0);
    }

}