package org.firstinspires.ftc.teamcode.subassembly;

import com.pedropathing.ivy.Command;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import static com.pedropathing.ivy.commands.Commands.*;

@SuppressWarnings("unused")
public class Intake {
    private final DcMotorEx intakeMotor;
    private boolean isRunning;
    private boolean isRunningInReverse;
    private final Command startIntakeCommand;
    private final Command stopIntakeCommand;


    public Intake(HardwareMap hwMap) {
        intakeMotor = hwMap.get(DcMotorEx.class, "intakeMotor");
        startIntakeCommand = instant(this::start);
        stopIntakeCommand = instant(this::stop);

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