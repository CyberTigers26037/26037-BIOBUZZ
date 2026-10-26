package org.firstinspires.ftc.teamcode.subassembly;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class TurretSystem {
    private LauncherTurret launcherTurret1;
    private LauncherTurret launcherTurret2;
    private final ElapsedTime loopTimer = new ElapsedTime();
    private double turretAngle = 0;


    public TurretSystem (HardwareMap hardwareMap) {
        launcherTurret1 = new LauncherTurret(hardwareMap, "1");
        launcherTurret2 = new LauncherTurret(hardwareMap, "2");
    }

    public void loop (double joystickAmount) {
        double dt = loopTimer.seconds();
        loopTimer.reset();
        if (Math.abs(joystickAmount) > 0.01) {
            turretAngle += joystickAmount * 90 * dt;
            turretAngle = Range.clip(turretAngle, -82, 183);
        }
        launcherTurret1.setTurretAngle(turretAngle);
        launcherTurret2.setTurretAngle(turretAngle);
        launcherTurret1.loop();
        launcherTurret2.loop();
    }



    public void toggleLauncher() {
        launcherTurret1.toggleLauncher();
        launcherTurret2.toggleLauncher();
    }

    public void adjustLauncherRpm(int amount) {
        int currpm = launcherTurret1.getLauncherRpm();
        launcherTurret1.setLauncherRpm(currpm + amount);
        launcherTurret2.setLauncherRpm(currpm + amount);
    }

    public void outputTelemetry(Telemetry telemetry) {
        telemetry.addData("turretangle", turretAngle);
        launcherTurret1.outputTelemetry(telemetry);
        launcherTurret2.outputTelemetry(telemetry);
    }

    public void setAngle(double angle) {
        turretAngle = Range.clip(angle, 0, 180);

    }
}
