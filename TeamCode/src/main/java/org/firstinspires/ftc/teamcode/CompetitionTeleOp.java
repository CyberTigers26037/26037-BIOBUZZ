package org.firstinspires.ftc.teamcode;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.subassembly.HiveTargeting;
import org.firstinspires.ftc.teamcode.subassembly.Intake;
import org.firstinspires.ftc.teamcode.subassembly.TransferSystem;
import org.firstinspires.ftc.teamcode.subassembly.TurretSystem;

@TeleOp(name = "CompetitionTeleOp")
public class CompetitionTeleOp extends OpMode {
     /*
      Control hub
        Motors
            Port 0 : lf (GoBILDA 5202/3/4 series)
            Port 1 : lb (GoBILDA 5202/3/4 series)
            Port 2 : rb (GoBILDA 5202/3/4 series)
            Port 3 : rl (GoBILDA 5202/3/4 series)
        Servos
            Port 0 : lfTurn (Continuous Rotation Servo)
            Port 1 : lbTurn (Continuous Rotation Servo)
            Port 2 : rbTurn (Continuous Rotation Servo)
            Port 2 : rfTurn (Continuous Rotation Servo)
        Analog input Devise
            Port 0 : lfTurnEncoder (Analog input)
            Port 1 : lbTurnEncoder (Analog input)
            Port 2 : rbTurnEncoder (Analog input)
            Port 3 : rfTurnEncoder (Analog input)
        I2C
            Port 1 : pinpoint

    Expansion hub
        Motors
            Port 0 : flywheelMotorNectar (GoBILDA 5202/3/4 series)
            Port 1 : flywheelMotorPolllen (GoBILDA 5202/3/4 series)
            Port 2 : intakeMotor (GoBILDA 5202/3/4 series)
            Port 3 : pollenTransferMotor (GoBILDA 5202/3/4 series)
        Servos
            Port 0 : turrentServoNectar (Continuous Rotation Servo)
            Port 1 : turrentServoPollen (Continuous Rotation Servo)
            Port 2 : light (Servo)
        Digital Devices
            Port 0 : turretSwitchNectar (Digital Device)
            Port 1 : turretSwitchPollen (Digital Device)
            Port 2 : numberPlatesensor (REV Touch Sensor)
        Analog input Devise
            Port 0 : turrentEncoderNectar (Analog input)
            Port 1 : turrentEncoderPollen (Analog input)
     */

    private Intake intake;
    private Follower follower;
    private TurretSystem turretSystem;

    private HiveTargeting hiveTargeting;

    private TransferSystem transferSystem;

    private boolean autoTracking;

    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose start = poseFactory.of(120, 9, 90);


    @Override
    public void init() {
        follower = Constants.create(hardwareMap);
        follower.setPose(OpModeStorage.autonomousEndPose != null ? OpModeStorage.autonomousEndPose: start);
        turretSystem = new TurretSystem(hardwareMap);
        intake = new Intake(hardwareMap);
        hiveTargeting = new HiveTargeting(hardwareMap, follower);
        transferSystem = new TransferSystem(hardwareMap);
    }

    private void processGunner(){

        if (gamepad2.rightTriggerWasPressed()) {
            transferSystem.startNectarTransfer();
        }
        else if (gamepad2.rightTriggerWasReleased()){
            transferSystem.stopNectarTransfer();
        }

        if (gamepad2.leftTriggerWasPressed()) {
            if (gamepad2.back) {
                transferSystem.startPollenTransferBackwards();
            } else{
                transferSystem.startPollenTransfer();
            }

        }
        else if (gamepad2.leftTriggerWasReleased()){
            transferSystem.stopPollenTransfer();
        }

        if (gamepad2.leftBumperWasPressed()) {
            turretSystem.toggleLauncher();
        }

        if (gamepad2.rightBumperWasPressed()) {
            turretSystem.toggleLauncher();
        }

        if (gamepad2.back && gamepad2.aWasPressed()){
            intake.startReverse();
        }
        else if (gamepad2.aWasPressed()){
            if(!intake.isRunning()){
                intake.start();
            }
            else{
                intake.stop();
            }
        }

        if (gamepad2.bWasPressed()){

        }

        if (gamepad2.xWasPressed()) {

        }

        if (gamepad2.yWasPressed()) {

        }

        if (gamepad2.dpadUpWasPressed()) {
            turretSystem.adjustLauncherRpm(+ 100);
        }

        if (gamepad2.dpadDownWasPressed()) {
            turretSystem.adjustLauncherRpm(- 100);
        }

        if (gamepad2.dpadRightWasPressed()) {

        }

        if (gamepad2.dpadLeftWasPressed()) {
            autoTracking = !autoTracking;
        }

    }

    private void processDriver(){

    }

    @Override
    public void loop() {
        follower.manual(
                -gamepad1.left_stick_y,
                -gamepad1.left_stick_x,
                -gamepad1.right_stick_x
        );

        follower.update();
        telemetry.addData("leftjoy", gamepad1.left_stick_x);

        if (autoTracking) {
            double angleToTarget = hiveTargeting.getAngleToTarget();
            turretSystem.setAngle(angleToTarget);
        }

        processGunner();
        processDriver();

        telemetry.addData("Auto Tracking",autoTracking);
        turretSystem.loop(gamepad2.right_stick_x);
        turretSystem.outputTelemetry(telemetry);
        hiveTargeting.outputTelemetry(telemetry);

    }


}
