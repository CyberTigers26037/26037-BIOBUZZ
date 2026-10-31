package org.firstinspires.ftc.teamcode.subassembly;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class TurretSystem {
    private LauncherTurret launcherTurretNectar;
    private LauncherTurret launcherTurretPollen;
    private final ElapsedTime loopTimer = new ElapsedTime();
    private double turretAngle = 90;

    private final boolean nectarLauncherEnabled = false;

    public TurretSystem (HardwareMap hardwareMap) {
        if(nectarLauncherEnabled) {
            launcherTurretNectar = new LauncherTurret(hardwareMap, "Nectar");
        }
        launcherTurretPollen = new LauncherTurret(hardwareMap, "Pollen");

    }

    public void loop (double joystickAmount) {
        double dt = loopTimer.seconds();
        loopTimer.reset();
        if (Math.abs(joystickAmount) > 0.01) {
            turretAngle += joystickAmount * 90 * dt;
            turretAngle = Range.clip(turretAngle, 0, 180);
        }

        if(nectarLauncherEnabled) {
            launcherTurretNectar.setTurretAngle(turretAngle);
            launcherTurretNectar.loop();
        }
        launcherTurretPollen.setTurretAngle(turretAngle);
        launcherTurretPollen.loop();
    }



    public void toggleLauncher() {
        if(nectarLauncherEnabled) {
            launcherTurretNectar.toggleLauncher();
        }
        launcherTurretPollen.toggleLauncher();
    }

    public void adjustLauncherRpm(int amount) {
        int currpm = launcherTurretNectar.getLauncherRpm();
        if(nectarLauncherEnabled) {
            launcherTurretNectar.setLauncherRpm(currpm + amount);
        }
        launcherTurretPollen.setLauncherRpm(currpm + amount);
    }

    public void outputTelemetry(Telemetry telemetry) {
        telemetry.addData("turretangle", turretAngle);
        if(nectarLauncherEnabled) {
            launcherTurretNectar.outputTelemetry(telemetry);
        }
        launcherTurretPollen.outputTelemetry(telemetry);
    }

    public void setAngle(double angle) {
        turretAngle = Range.clip(angle, 0, 180);

    }

    public void adjustRpmForDistance(double distanceToTarget) {
        int newRpm = 3000;
        if(nectarLauncherEnabled) {
            launcherTurretNectar.setLauncherRpm(newRpm);
        }
        launcherTurretPollen.setLauncherRpm(newRpm);
    }
}
