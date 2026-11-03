package org.firstinspires.ftc.teamcode.subassembly;

import static com.pedropathing.ivy.commands.Commands.instant;

import com.pedropathing.ivy.Command;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

@SuppressWarnings("unused")
public class TransferSystem {
    private final DcMotor pollenTransferMotor;
    private boolean isRunningPollen;

    private boolean isRunningNectar;
    private Command startTransferCommand;
    private Command stopTransferCommand;

    public TransferSystem(HardwareMap hwMap) {
        startTransferCommand = instant(() -> startPollenTransfer());
        stopTransferCommand = instant(() -> stopPollenTransfer());

        pollenTransferMotor = hwMap.get(DcMotor.class, "pollenTransferMotor");
        pollenTransferMotor.setDirection(DcMotorSimple.Direction.REVERSE);

    }

    public void startAllTransfer(){
        startPollenTransfer();
        startNectarTransfer();
    }

    public void startPollenTransfer(){
        isRunningPollen = true;
        pollenTransferMotor.setPower(1.0);
    }
    public void startPollenTransferBackwards() {
        isRunningPollen = true;
        pollenTransferMotor.setPower(-1.0);
    }

    public void startNectarTransfer() {
        isRunningNectar = true;
    }

    public void stopAllTransfer() {
        stopPollenTransfer();
        startNectarTransfer();
    }

    public void stopPollenTransfer() {
        isRunningPollen = false;
        pollenTransferMotor.setPower(0);
    }

    public void stopNectarTransfer() {
        isRunningNectar = false;
    }


    public boolean isRunningPollen() {
        return isRunningPollen;
    }

    public boolean isRunningNectar() {
        return isRunningNectar;
    }

    public void togglePollenTransfer () {
        if (isRunningPollen()) {
            stopPollenTransfer();
        } else {
            startPollenTransfer();
        }
    }
    public void toggleNectarTransfer () {
        if (isRunningNectar()) {
            stopNectarTransfer();
        } else {
            startNectarTransfer();
        }
    }
    public void toggleAllTransfer () {
        if (isRunningPollen() || isRunningNectar()) {
            stopAllTransfer();
        } else {
            startAllTransfer();
        }
    }


    public Command getStartTransferCommand() {
        return startTransferCommand;
    }

    public Command getStopTransferCommand() {
        return stopTransferCommand;
    }

}
