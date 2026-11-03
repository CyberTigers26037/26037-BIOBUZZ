package org.firstinspires.ftc.teamcode.subassembly;

import static com.pedropathing.ivy.commands.Commands.instant;

import com.pedropathing.ivy.Command;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;


public class TurretSystem {
    private LauncherTurret launcherTurretNectar;
    private final LauncherTurret launcherTurretPollen;
    private final Command startLauncherCommand;
    private final Command stopLauncherCommand;
    private final Command waitForLauncherInitCommand;
    private final ElapsedTime loopTimer = new ElapsedTime();
    private double turretAngle = 90;

    private final boolean nectarLauncherEnabled = false;

    public TurretSystem (HardwareMap hardwareMap) {
        if(nectarLauncherEnabled) {
            launcherTurretNectar = new LauncherTurret(hardwareMap, "Nectar");
        }
        launcherTurretPollen = new LauncherTurret(hardwareMap, "Pollen");

        startLauncherCommand = instant(this::startLauncher);
        stopLauncherCommand = instant(this::stopLauncher);
        waitForLauncherInitCommand = Command.build()
                .setDone(launcherTurretPollen::isInitialized);

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


    public int getMotorRpm(){
        return launcherTurretPollen.getMotorRpm();
    }

    public void toggleLauncher() {
        if(nectarLauncherEnabled) {
            launcherTurretNectar.toggleLauncher();
        }
        launcherTurretPollen.toggleLauncher();
    }

    public void startLauncher() {
        launcherTurretPollen.startLauncher();
    }

    public void stopLauncher() {
        launcherTurretPollen.stopLauncher();
    }

    public void adjustLauncherRpm(int amount) {
        int currpm = launcherTurretPollen.getLauncherRpm();
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
        int newRpm = 3500;
        if (distanceToTarget > 60) {
            newRpm = 3800;
        }
        if(nectarLauncherEnabled) {
            launcherTurretNectar.setLauncherRpm(newRpm);
        }
        launcherTurretPollen.setLauncherRpm(newRpm);
    }

    public Command getStartLauncherCommand() {
        return startLauncherCommand;
    }

    public Command getStopLauncherCommand() {
        return stopLauncherCommand;
    }

    public Command getWaitForLauncherInitCommand() {
        return waitForLauncherInitCommand;
    }
}
