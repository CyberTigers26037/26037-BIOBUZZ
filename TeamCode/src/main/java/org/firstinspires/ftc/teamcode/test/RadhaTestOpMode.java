package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.teamcode.subassembly.LauncherTurret;

@SuppressWarnings("unused")
@TeleOp
public class RadhaTestOpMode extends OpMode {
  // Turret limits in degrees, measured from home (limit switch) position
  private static final double TURRET_MIN_DEG = 0;
  private static final double TURRET_MAX_DEG = 360;
  private static final double MAX_DEG_PER_SEC = 90;
  private static final double STICK_DEADZONE = 0.1;

  private LauncherTurret launcherTurret1;
  private LauncherTurret launcherTurret2;
  private double turretsAngle = 0;
  private final ElapsedTime loopTimer = new ElapsedTime();

  @Override
  public void init() { // called when the object is created
    launcherTurret1 = new LauncherTurret(hardwareMap, "1");
    launcherTurret2 = new LauncherTurret(hardwareMap, "2");
    telemetry.addData("Status", "Initialized");
  }

  @Override
  public void start() { // Called when the start button is pressed
    loopTimer.reset();
    telemetry.addData("Status", "Started");
  }

  @Override
  public void loop() { // Called over and over again, never at the same time
    // Get the change in time, delta time, since the last reset() call
    double dt = loopTimer.seconds();
    loopTimer.reset();

    // Set the angle if we are using the gamepads
    turretsAngle = stepAngle(turretsAngle, gamepad1.left_stick_x, dt);

    // dpads override any of the above
    if (gamepad1.dpadRightWasPressed()) {
      turretsAngle = 0;
    } else if (gamepad1.dpadUpWasPressed()) {
      turretsAngle = 90;
    } else if (gamepad1.dpadLeftWasPressed()) {
      turretsAngle = 180;
    } else if (gamepad1.dpadDownWasPressed()) {
      turretsAngle = 270;
    }

    // Set the angles:
    launcherTurret1.setTurretAngle(turretsAngle);
    launcherTurret2.setTurretAngle(turretsAngle);

    // Check if the triggers were pressed
    if (gamepad1.rightTriggerWasPressed()) {
      launcherTurret1.toggleLauncher();
      launcherTurret2.toggleLauncher();
    }

    // Rehome with top two bumper presses
    if (gamepad1.left_bumper && gamepad1.rightBumperWasPressed()) {
      // Home
      launcherTurret1.rehome();
      launcherTurret2.rehome();
      turretsAngle = 0;
    }

    // Start the loops
    launcherTurret1.loop();
    launcherTurret2.loop();

    telemetry.addData("turret1Angle", turretsAngle);

    launcherTurret1.outputTelemetry(telemetry);
    launcherTurret2.outputTelemetry(telemetry);
  }

  private static double stepAngle(double angle, double stick, double dt) {
    if (Math.abs(stick) > STICK_DEADZONE) {
      angle += stick * MAX_DEG_PER_SEC * dt;
    }

    return Range.clip(angle, TURRET_MIN_DEG, TURRET_MAX_DEG);
  }
}
