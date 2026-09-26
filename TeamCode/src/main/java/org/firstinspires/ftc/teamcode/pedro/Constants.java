package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.algorithm.ForesightConfig;
import com.pedropathing.controllers.Controller;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Matrix;
import com.pedropathing.math.Vector2D;
import com.pedropathing.revhub.drivetrains.MecanumConfig;
import com.pedropathing.revhub.localizers.PinpointConfig;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Constants {
    public static Follower create(HardwareMap h) {
        // return new Follower(Drivetrain, Localizer, Foresight);
        return null;

    }
    public static MecanumConfig drivetrainConfig = new MecanumConfig(c -> {
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
        c.xPodOffset.set(-168.0);
        c.yPodOffset.set(-84.0);
        c.xPodDirection.set(GoBildaPinpointDriver.EncoderDirection.REVERSED);
        c.yPodDirection.set(GoBildaPinpointDriver.EncoderDirection.REVERSED);
        c.globalDistanceUnit.set(DistanceUnit.INCH);
        c.offsetUnits.set(DistanceUnit.MM);
    });

    public static ForesightConfig foresightConfig = new ForesightConfig(
            c -> {
                Controller primaryTranslationalForward = Controller.proportional(0.1472743261099644);
                Controller secondaryTranslationalForward = Controller.proportional(0.05441392825516179);
                Controller primaryTranslationalLateral = Controller.proportional(0.19791003074622826);
                Controller secondaryTranslationalLateral = Controller.proportional(0.07312246810731465);

                c.forwardTranslational.set(Controller.piecewise(secondaryTranslationalForward).put(2.5, primaryTranslationalForward));
                c.strafeTranslational.set(Controller.piecewise(secondaryTranslationalLateral).put(2.5, primaryTranslationalLateral));

                c.coast.set(Controller.proportionalFeedforward(0.017375166699292434));
                c.brake.set(Controller.proportionalFeedforward(0.01476889169439857));

                c.headingFeedback.set(Controller.proportional(2.8332053493080704));
                c.headingBrakeCoefficients.set(Vector2D.cartesian(0.039691692245534246, 0.005394757423252485));

                c.linearBrakeCoefficients.set(Matrix.diag(0.05042867430309611, 0.060770975748680525));
                c.quadraticBrakeCoefficients.set(Matrix.diag(0.001481022262112584, 0.0011710379102574033));

                c.maxAchievableForwardVelocity.set(58.72766762341472);
                c.maxAchievableStrafeVelocity.set(50.96738964557392);
                c.naturalForwardDeceleration.set(36.28555393231942);
                c.naturalStrafeDeceleration.set(103.3152383644613);
            }
    );



}