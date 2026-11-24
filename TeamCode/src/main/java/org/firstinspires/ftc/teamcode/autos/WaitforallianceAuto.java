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
   // private Pose controlPoint1;
  //  private Pose controlPoint2;


    // Path methods
    private Path startToArmDrop() {
        return line(startPose, armDropPose).linear(startPose, armDropPose);
    }

    private Path armDropToPark() {
        return line(armDropPose, parkPose).linear(armDropPose, parkPose);
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
                new SleepCommand(1.0),
                follow(follower, startToArmDrop()),
                new SleepCommand(15.0),
                launcher.getWaitForLauncherInitCommand(),
                launcher.getStartLauncherCommand(),
                transfer.getStartTransferCommand(),
                intake.getStartIntakeCommand(),
                new SleepCommand(1.0),
                transfer.getStopTransferCommand(),
                launcher.getStopLauncherCommand(),
                intake.getStopIntakeCommand(),
                new SleepCommand(1.0),
                follow(follower, armDropToPark())

        );
    }

    private void setUpPoses() {
        if (isBlueAlliance) {
            startPose = poseFactory.of(109, 8, 90);
            armDropPose = poseFactory.of(83, 17, 90);
            parkPose = poseFactory.of(130, 22, 270);
           // controlPoint1 = poseFactory.of(134, 43, 90);
            // controlPoint2 = poseFactory.of(71, 40, 90);
        } else {
            startPose = poseFactory.of(62, 13, 270);
            armDropPose = poseFactory.of(62, 17, 270);
            parkPose = poseFactory.of(14, 122, 270);
           // controlPoint1 = poseFactory.of(91, 25, 90);
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