package org.firstinspires.ftc.teamcode.subassembly;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class TransferSystem {
    private boolean isRunningPollen;
    private boolean isRunningNectar;

    public TransferSystem(HardwareMap hwMap) {

    }

    public void startAllTransfer(){
        isRunningPollen = true;
        isRunningNectar = true;
    }

    public void startPollenTransfer(){
        isRunningPollen = true;
    }

    public void startNectarTransfer() {
        isRunningNectar = true;
    }

    public void stopAllTransfer() {
        isRunningPollen = false;
        isRunningNectar = false;
    }

    public void stopPollenTransfer() {
        isRunningPollen = false;
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
