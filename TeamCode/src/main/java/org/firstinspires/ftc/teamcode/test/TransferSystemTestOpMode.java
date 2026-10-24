package org.firstinspires.ftc.teamcode.test;


import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.subassembly.TransferSystem;


@TeleOp(name = "TransferTestOpMode", group = "Test")
public class TransferSystemTestOpMode extends OpMode {
    private TransferSystem transfer;

    @Override
    public void init() {
        transfer = new TransferSystem(hardwareMap);
    }

    @Override
    public void loop() {
        if (gamepad1.aWasPressed()) {
            transfer.togglePollenTransfer();
        }

        if (gamepad1.bWasPressed()) {
            transfer.toggleNectarTransfer();
        }

        if (gamepad1.yWasPressed()) {
            transfer.toggleAllTransfer();
        }


        telemetry.addData("Pollen Transfer: ", transfer.isRunningPollen());
        telemetry.addData("Nectar Transfer: ", transfer.isRunningNectar());
    }

}
