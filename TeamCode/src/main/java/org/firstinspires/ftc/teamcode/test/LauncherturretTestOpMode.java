package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.subassembly.TurretSystem;

@SuppressWarnings("unused")
@TeleOp(group = "Test")
public class LauncherturretTestOpMode extends OpMode {
    private TurretSystem turretSystem;

    @Override
    public void init() {
        turretSystem = new TurretSystem(hardwareMap);
    }

    @Override
    public void loop() {
        telemetry.addData("leftjoy", gamepad1.left_stick_x);

        if (gamepad1.rightTriggerWasPressed()) {
            turretSystem.toggleLauncher();
        }
        if (gamepad1.aWasPressed()){
            turretSystem.adjustLauncherRpm(- 100);
        }
        if (gamepad1.yWasPressed()){
            turretSystem.adjustLauncherRpm(+ 100);
        }
        turretSystem.loop(gamepad1.left_stick_x);
        turretSystem.outputTelemetry(telemetry);


    }
}
