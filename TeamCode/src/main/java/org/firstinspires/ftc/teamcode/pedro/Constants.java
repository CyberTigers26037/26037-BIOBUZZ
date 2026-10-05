package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.algorithm.Foresight;
import com.pedropathing.algorithm.ForesightConfig;
import com.pedropathing.controllers.Controller;
import com.pedropathing.drivetrain.Drivetrain;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Vector2D;
import com.pedropathing.revhub.drivetrains.CoaxialPod;
import com.pedropathing.revhub.drivetrains.CoaxialPodConfig;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.pedropathing.revhub.drivetrains.MecanumConfig;
import com.pedropathing.revhub.drivetrains.Swerve;
import com.pedropathing.revhub.drivetrains.SwerveConfig;
import com.pedropathing.revhub.localizers.PinpointConfig;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.config.RobotConfig;

public class Constants {
    public static Follower create(HardwareMap h) {
        return new Follower(
                new PinpointLocalizer(h, localizerConfig),
                createDrivetrain(h),
                new Foresight(foresightConfig)
        );
    }

    public static Drivetrain createDrivetrain(HardwareMap h) {
        if (RobotConfig.isMecanumChassis()) {
            return new Mecanum(h, mecanumConfig);
        }
        else {
            CoaxialPod leftFrontPod = new CoaxialPod(h, leftFront);
            CoaxialPod rightFrontPod = new CoaxialPod(h, rightFront);
            CoaxialPod leftBackPod = new CoaxialPod(h, leftBack);
            CoaxialPod rightBackPod = new CoaxialPod(h, rightBack);
            return new Swerve(h, swerveConfig,
                leftFrontPod, rightFrontPod, leftBackPod, rightBackPod);
        }
    }

    public static MecanumConfig mecanumConfig = new MecanumConfig(c -> {
        c.frontLeftName.set("front_left_drive");
        c.frontRightName.set("front_right_drive");
        c.backLeftName.set("back_left_drive");
        c.backRightName.set("back_right_drive");
        c.frontLeftDirection.set(DcMotorSimple.Direction.REVERSE);
        c.frontRightDirection.set(DcMotorSimple.Direction.FORWARD);
        c.backLeftDirection.set(DcMotorSimple.Direction.REVERSE);
        c.backRightDirection.set(DcMotorSimple.Direction.FORWARD);
        c.manualBrakeMode.set(true);
    });

    public static PinpointConfig localizerConfig = new PinpointConfig(c -> {
        c.name.set("pinpoint");
        c.podType.set(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        c.xPodOffset.set(RobotConfig.getXPodOffset());
        c.yPodOffset.set(RobotConfig.getYPodOffset());
        c.xPodDirection.set(GoBildaPinpointDriver.EncoderDirection.REVERSED);
        c.yPodDirection.set(GoBildaPinpointDriver.EncoderDirection.REVERSED);
        c.globalDistanceUnit.set(DistanceUnit.INCH);
        c.offsetUnits.set(DistanceUnit.MM);
    });

    public static ForesightConfig foresightConfig = RobotConfig.getForesightConfig();

    public static SwerveConfig swerveConfig = new SwerveConfig(
        c -> {
            c.zeroPowerBehavior.set(SwerveConfig.ZeroPowerBehavior.IGNORE_ANGLE_CHANGES);
            c.manualBrakeMode.set(true);
            c.voltageCompensation.set(false);
        }
    );

    public static CoaxialPodConfig leftFront = new CoaxialPodConfig(
            c -> {
                c.name.set("swerveLeftFront");
                c.motorName.set("lf");
                c.servoName.set("lfTurn");
                c.servoEncoderName.set("lfTurnEncoder");
                c.turnController.set(Controller.pid(0.3, 0, 0.005)
                        .plus(Controller.proportionalFeedforward(0)));
                c.driveDirection.set(DcMotorSimple.Direction.FORWARD);
                c.servoDirection.set(DcMotorSimple.Direction.FORWARD);
                c.analogMinVoltage.set(0.006);
                c.analogMaxVoltage.set(3.212);
                c.podOffset.set(Vector2D.cartesian(1675,1515 ));
                //c.angleOffsetRad.set(Math.toRadians(92.0));
                c.angleOffsetRad.set(Math.toRadians(274));
                c.encoderReversed.set(true);
            }
    );

    public static CoaxialPodConfig rightFront = new CoaxialPodConfig(
            c -> {
                c.name.set("swerveRightFront");
                c.motorName.set("rf");
                c.servoName.set("rfTurn");
                c.servoEncoderName.set("rfTurnEncoder");
                c.turnController.set(Controller.pid(0.3, 0, 0.005)
                        .plus(Controller.proportionalFeedforward(0)));
                c.driveDirection.set(DcMotorSimple.Direction.FORWARD);
                c.servoDirection.set(DcMotorSimple.Direction.FORWARD);
                c.analogMinVoltage.set(0.004);
                c.analogMaxVoltage.set(3.199);
                c.podOffset.set(Vector2D.cartesian(1675,-1515 ));
               // c.angleOffsetRad.set(Math.toRadians(37.0));
                c.angleOffsetRad.set(Math.toRadians(218.6));
                c.encoderReversed.set(true);
            }
    );

    public static CoaxialPodConfig leftBack = new CoaxialPodConfig(
            c -> {
                c.name.set("swerveLeftBack");
                c.motorName.set("lb");
                c.servoName.set("lbTurn");
                c.servoEncoderName.set("lbTurnEncoder");
                c.turnController.set(Controller.pid(0.3, 0, 0.005)
                        .plus(Controller.proportionalFeedforward(0)));
                c.driveDirection.set(DcMotorSimple.Direction.FORWARD);
                c.servoDirection.set(DcMotorSimple.Direction.FORWARD);
                c.analogMinVoltage.set(0.004);
                c.analogMaxVoltage.set(3.199);
                c.podOffset.set(Vector2D.cartesian(-1675,1515 ));
               // c.angleOffsetRad.set(Math.toRadians(115.6));
                c.angleOffsetRad.set(Math.toRadians(299));
                c.encoderReversed.set(true);
            }
    );

    public static CoaxialPodConfig rightBack = new CoaxialPodConfig(
            c -> {
                c.name.set("swerveRightBack");
                c.motorName.set("rb");
                c.servoName.set("rbTurn");
                c.servoEncoderName.set("rbTurnEncoder");
                c.turnController.set(Controller.pid(0.3, 0, 0.005)
                        .plus(Controller.proportionalFeedforward(0)));
                c.driveDirection.set(DcMotorSimple.Direction.FORWARD);
                c.servoDirection.set(DcMotorSimple.Direction.FORWARD);
                c.analogMinVoltage.set(0.002);
                c.analogMaxVoltage.set(3.219);
                c.podOffset.set(Vector2D.cartesian(-1675,-1515 ));
               // c.angleOffsetRad.set(Math.toRadians(148.3));
                c.angleOffsetRad.set(Math.toRadians(322.9));
                c.encoderReversed.set(true);
            }
    );
}