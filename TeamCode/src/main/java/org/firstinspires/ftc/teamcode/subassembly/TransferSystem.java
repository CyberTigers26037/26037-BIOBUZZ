package org.firstinspires.ftc.teamcode.subassembly;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class TransferSystem {
    private DcMotor pollenTransferMotor;
    private boolean isRunningPollen;
    private boolean isRunningNectar;

    public TransferSystem(HardwareMap hwMap) {
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

}
