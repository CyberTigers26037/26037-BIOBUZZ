package org.firstinspires.ftc.teamcode.autos;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;


import static com.pedropathing.api.Paths.curve;
import static com.pedropathing.api.Paths.line;
import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;

import org.firstinspires.ftc.teamcode.OpModeStorage;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.pedro.SleepCommand;
import org.firstinspires.ftc.teamcode.subassembly.HiveTargeting;
import org.firstinspires.ftc.teamcode.subassembly.Intake;
import org.firstinspires.ftc.teamcode.subassembly.TransferSystem;
import org.firstinspires.ftc.teamcode.subassembly.TurretSystem;
import org.firstinspires.ftc.teamcode.subassembly.NumberPlateSensor;

@Autonomous
public class WaitforallianceAuto extends OpMode {
    private Follower follower;
    private Intake intake;
    private TurretSystem launcher;
    private TransferSystem transfer;
    private HiveTargeting hiveTargeting;
    private final PoseFactory poseFactory = PoseFactory.degrees();

    protected boolean isBlueAlliance;


    // Poses
    private Pose startPose;
    private Pose armDropPose;
    private Pose parkPose;
    private Pose controlPoint1;
    private Pose intakeFlower;
    private Pose launch;



    // Path methods
    private Path startToArmDrop() {
        return curve(startPose, controlPoint1, armDropPose).linear(startPose, armDropPose);
    }

    private Path armDropTointakeFlower() {
        return line(armDropPose,  intakeFlower).linear(armDropPose,  intakeFlower);
    }

    private Path intakeFlowerTolaunch() {
        return line(intakeFlower,  launch).linear(intakeFlower,  launch);
    }
    private Path launchToparkPose() {
        return line(launch,  parkPose).linear(launch,  parkPose);
    }
    private Command startAutoTarget(){
        return Command.build()
                .setStart(() -> {
                    double angleToTarget = hiveTargeting.getAngleToTarget();
                    launcher.setAngle(angleToTarget);
                    double distanceToTarget = hiveTargeting.getDistanceToTarget();
                    launcher.adjustRpmForDistance(distanceToTarget);
                })
                .setDone(() -> true);
    }

    private Command autoRoutine() {
        return sequential(
           //     new SleepCommand(15.0),
                launcher.getWaitForLauncherInitCommand(),
                launcher.getStartLauncherCommand(),
                transfer.getStartTransferCommand(),
                startAutoTarget(),
                //transfer.getStartTransferCommand(),
                //  intake.getStartIntakeCommand(),
                new SleepCommand(1.0),
                launcher.getStopLauncherCommand(),
                follow(follower, startToArmDrop()),
                follow(follower, armDropTointakeFlower()),
                new SleepCommand(2.0),
              //  follow(follower, intakeFlowerTolaunch()),
                launcher.getStartLauncherCommand(),
                transfer.getStartTransferCommand(),
                new SleepCommand(1.0),
               // follow(follower, launchToparkPose()),
                transfer.getStopTransferCommand(),
                launcher.getStopLauncherCommand(),
                intake.getStopIntakeCommand()
        );
    }

    private void setUpPoses() {
        if (isBlueAlliance) {
            startPose = poseFactory.of(108, 9, 90);
            armDropPose = poseFactory.of(94, 27, 90);
            intakeFlower = poseFactory.of(94, 20, 270);
            launch = poseFactory.of(83, 13, 90);
            parkPose = poseFactory.of(129, 23, 180);
            controlPoint1 = poseFactory.of(118, 35, 90);

        } else {
            startPose = poseFactory.of(32.5, 133.5, 270);
            armDropPose = poseFactory.of(58.5, 124.5, 270);
            parkPose = poseFactory.of(11.5, 119.5, 270);

        }
    }

    @Override
    public void init() {
        NumberPlateSensor numberPlateSensor = new NumberPlateSensor(hardwareMap);
        isBlueAlliance = numberPlateSensor.isNumberPlateBlue();

        setUpPoses();

        Scheduler.reset();
        follower = Constants.create(hardwareMap);
        follower.setPose(startPose);
        follower.update();

        intake = new Intake(hardwareMap);
        launcher = new TurretSystem(hardwareMap);
        transfer = new TransferSystem(hardwareMap);
        launcher.setLauncherRpm(3200);
        hiveTargeting = new HiveTargeting(hardwareMap, follower);
    }

    @Override
    public void start() {
        schedule(autoRoutine());
    }

    @Override
    public void loop() {
        follower.update();
        launcher.loop(0);
        Scheduler.execute();
        // add your other methods needed in the loop here

        telemetry.addData("X", follower.pose().x());
        telemetry.addData("Y", follower.pose().y());
        telemetry.addData("Heading", Math.toDegrees(follower.pose().heading()));
        telemetry.addData("Follower Mode", follower.mode());
        telemetry.update();

    }


    @Override
    public void stop() {
        OpModeStorage.autonomousEndPose = follower.pose(); //saves your position in that file
    }
}