package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subassembly.Scrabbler;

@TeleOp
public class ScrabblerTestOpMode extends OpMode {
    private Scrabbler scrabbler;

    @Override
    public void init() {
        this.scrabbler = new Scrabbler(hardwareMap);
    }

    @Override
    public void loop() {
        if (gamepad1.aWasPressed()) {
            scrabbler.lower();
        }

        if (gamepad1.bWasPressed()) {
            scrabbler.raise();
        }

        if (gamepad1.dpad_up) {
            scrabbler.adjustAngle(+ 1);
        }

        if (gamepad1.dpad_down) {
            scrabbler.adjustAngle(-1);
        }
        telemetry.addData("angle", scrabbler.getAngle());
    }
}
