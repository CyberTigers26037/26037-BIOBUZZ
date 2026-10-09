package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.subassembly.LauncherTurret;

@SuppressWarnings("unused")
@TeleOp
public class LauncherturretTestOpMode extends OpMode {
    private LauncherTurret launcherTurret;

    @Override
    public void init() {
        launcherTurret = new LauncherTurret(hardwareMap);
    }

    @Override
    public void loop() {
        if (gamepad1.dpadRightWasPressed()) {
            launcherTurret.setTurretAngle(0);
        }
        if (gamepad1.dpadUpWasPressed()) {
            launcherTurret.setTurretAngle(90);
        }
        if (gamepad1.dpadLeftWasPressed()) {
            launcherTurret.setTurretAngle(180);
        }
        if (gamepad1.dpadDownWasPressed()) {
            launcherTurret.setTurretAngle(270);
        }
        launcherTurret.loop();

        launcherTurret.outputTelemetry(telemetry);
    }
}
