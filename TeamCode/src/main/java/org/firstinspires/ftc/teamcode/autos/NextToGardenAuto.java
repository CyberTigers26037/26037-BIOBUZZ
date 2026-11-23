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
import org.firstinspires.ftc.teamcode.subassembly.Intake;
import org.firstinspires.ftc.teamcode.subassembly.TransferSystem;
import org.firstinspires.ftc.teamcode.subassembly.TurretSystem;
import org.firstinspires.ftc.teamcode.subassembly.NumberPlateSensor;
import org.firstinspires.ftc.teamcode.subassembly.HiveTargeting;

@Autonomous
public class NextToGardenAuto extends OpMode {
    private Follower follower;
    private Intake intake;
    private TurretSystem launcher;
    private TransferSystem transfer;
    private final PoseFactory poseFactory = PoseFactory.degrees();
    private HiveTargeting hiveTargeting;
    protected boolean isBlueAlliance;




    // Poses
    private Pose startPose;
    private Pose armDropPose;
    private Pose parkPose;
    private Pose garden;
    private Pose gardenIntake;
    private Pose controlPoint1;


    // Path methods
    private Path startToArmDrop() {
        return line(startPose, armDropPose).linear(startPose, armDropPose);
    }
    private Path armDropToGarden(){
        return line(armDropPose, garden).linear(armDropPose, garden);
    }
    private Path gardenToGardenIntake(){
        return line(garden, gardenIntake).linear(garden, gardenIntake);
    }
    private Path gardenIntakeToPark() {
        return curve(gardenIntake, controlPoint1, parkPose).linear(gardenIntake, parkPose);
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
                launcher.getWaitForLauncherInitCommand(),
                launcher.getStartLauncherCommand(),
                startAutoTarget(),
                new SleepCommand(1.0),
                transfer.getStartTransferCommand(),
                intake.getStartIntakeCommand(),
                new SleepCommand(3.0),
                follow(follower, startToArmDrop()),
                transfer.getStopTransferCommand(),
                launcher.getStopLauncherCommand(),
                intake.getStopIntakeCommand(),
                follow(follower,armDropToGarden()),
                intake.getStartIntakeCommand(),
                follow(follower, gardenToGardenIntake()),
                new SleepCommand(3.0),
                follow(follower, gardenIntakeToPark()),
                intake.getStopIntakeCommand()

        );
    }

    private void setUpPoses() {
        if(isBlueAlliance) {
            startPose = poseFactory.of(109, 131, 270);
            armDropPose = poseFactory.of(109, 119, 270);
            garden = poseFactory.of(131, 119, 90);
            gardenIntake = poseFactory.of(132, 126, 90);
            parkPose = poseFactory.of(129, 48, 270);
            controlPoint1 = poseFactory.of(108, 92, 0);

        }
        else {
            startPose = poseFactory.of(32.5, 10.5, 90);
            armDropPose = poseFactory.of(32.5, 22.5, 90);
            garden = poseFactory.of(10.5, 22.5, 270);
            gardenIntake = poseFactory.of(9.5, 15.5, 270);
            parkPose = poseFactory.of(10.5, 93.5, 90);
            controlPoint1 = poseFactory.of(33.5, 49.5, 0);
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
        hiveTargeting.outputTelemetry(telemetry);

    }


    @Override
    public void stop() {
        OpModeStorage.autonomousEndPose = follower.pose(); //saves your position in that file
    }
}
