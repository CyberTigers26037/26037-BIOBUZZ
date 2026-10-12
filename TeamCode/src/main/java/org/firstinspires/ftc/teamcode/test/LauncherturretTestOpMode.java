package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.subassembly.LauncherTurret;

@SuppressWarnings("unused")
@TeleOp
public class LauncherturretTestOpMode extends OpMode {
    private LauncherTurret launcherTurret1;
    private LauncherTurret launcherTurret2;

    @Override
    public void init() {

        launcherTurret1 = new LauncherTurret(hardwareMap, "1");
        launcherTurret2 = new LauncherTurret(hardwareMap, "2");
    }

    @Override
    public void loop() {
        if (gamepad1.dpadRightWasPressed()) {
            launcherTurret1.setTurretAngle(0);
            launcherTurret2.setTurretAngle(0);
        }
        if (gamepad1.dpadUpWasPressed()) {
            launcherTurret1.setTurretAngle(90);
            launcherTurret2.setTurretAngle(90);
        }
        if (gamepad1.dpadLeftWasPressed()) {
            launcherTurret1.setTurretAngle(180);
            launcherTurret2.setTurretAngle(180);
        }
        if (gamepad1.dpadDownWasPressed()) {
            launcherTurret1.setTurretAngle(270);
            launcherTurret2.setTurretAngle(270);
        }
        if (gamepad1.rightTriggerWasPressed()) {
            launcherTurret1.toggleLauncher();
            launcherTurret2.toggleLauncher();
        }
        launcherTurret1.loop();
        launcherTurret2.loop();

        launcherTurret1.outputTelemetry(telemetry);
        launcherTurret2.outputTelemetry(telemetry);
    }
}
