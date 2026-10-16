package org.firstinspires.ftc.teamcode.subassembly;


import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.config.RobotConfig;

public class ArtifactLauncher {
    private final DcMotorEx flywheelMotor;
    private int flywheelRpm = 3000;
    public boolean isRunning;
    private static final int MIN_RPM =  1000;
    private static final int MAX_RPM = 6000;
    private static final double MOTOR_TICKS_PER_REVOLUTION = 28;
    private static final double SECONDS_PER_MINUTE = 60;

    public int getFlywheelRpm(){
        return flywheelRpm;
    }

    public void adjustFlywheelRpm(int amount){
        flywheelRpm = Range.clip(amount, MIN_RPM, MAX_RPM);
    }


    public ArtifactLauncher(HardwareMap hwMap, String devicename) {
        flywheelMotor = hwMap.get(DcMotorEx.class, devicename);
        flywheelMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flywheelMotor.setVelocityPIDFCoefficients(10, 3, 0, 0);
        flywheelMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    }

    private double rpmToTps(int rpm) {
        // Converts revolutions per minute (RPM) to ticks per second (TPS)
        return rpm * MOTOR_TICKS_PER_REVOLUTION / SECONDS_PER_MINUTE;
    }

    private int tpsToRpm(double tps) {
        // Converts ticks per second (TPS) to revolutions per minute (RPM)
        return (int)(tps / MOTOR_TICKS_PER_REVOLUTION * SECONDS_PER_MINUTE);
    }

    public void startFlywheelMotor() {
        flywheelMotor.setVelocity(rpmToTps(flywheelRpm));
        isRunning = true;
    }

    public void stopFlywheelMotor() {
        flywheelMotor.setVelocity(0);
        isRunning = false;
    }

    public int getActualFlywheelRpm() {
        return tpsToRpm(flywheelMotor.getVelocity());
    }


    public boolean isRunning(){
        return isRunning;
    }

    public void setFlywheelRpm(int rpm) {
        flywheelRpm = rpm;
        flywheelRpm = Range.clip(flywheelRpm, MIN_RPM, MAX_RPM);
    }






    public boolean isLauncherAboveMinSpeed() {
        return getActualFlywheelRpm() > MIN_RPM;
    }
}

