package org.firstinspires.ftc.teamcode.subassembly;

import com.pedropathing.ivy.Command;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import static com.pedropathing.ivy.commands.Commands.*;


public class Intake {
    private final DcMotorEx intakeMotor;
    private boolean isRunning;
    private boolean isRunningInReverse;
    private Command startIntakeCommand;
    private Command stopIntakeCommand;


    public Intake(HardwareMap hwMap) {
        intakeMotor = hwMap.get(DcMotorEx.class, "intakeMotor");
        startIntakeCommand = instant(() -> start());
        stopIntakeCommand = instant(() -> stop());

    }

    public void start(){
        intakeMotor.setPower(1.0);
        isRunning = true;
        isRunningInReverse = false;
    }

    public void stop() {
        intakeMotor.setPower(0.0);
        isRunning = false;
        isRunningInReverse = false;
    }

    public boolean isRunning() {
        return isRunning;
    }

    public void startReverse() {
        intakeMotor.setPower(-1.0);
        isRunning = true;
        isRunningInReverse = true;
    }

    public boolean isRunningInReverse() {
        return isRunningInReverse;
    }

    public Command getStartIntakeCommand() {
        return startIntakeCommand;
    }

    public Command getStopIntakeCommand() {
        return stopIntakeCommand;
    }


}