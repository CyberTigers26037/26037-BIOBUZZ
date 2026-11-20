package org.firstinspires.ftc.teamcode.pedro.swerve;

import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.revhub.drivetrains.Swerve;
import com.pedropathing.revhub.drivetrains.SwerveConfig;
import com.pedropathing.revhub.drivetrains.SwervePod;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.internal.files.DataLogger;

import java.util.Arrays;
import java.util.List;

public class CyberTigersSwerve extends Swerve {
    private final List<SwervePod> pods;

    public CyberTigersSwerve(HardwareMap hardwareMap, SwerveConfig config, SwervePod... pods) {
        super(hardwareMap, config, pods);

        this.pods = Arrays.asList(pods);
    }

    public void applyDrive(DrivePowers powers) {
        //ANP, 10/12/2026, Swerve reverses the strafe and turn internally
        // which makes Mecanum and Swerve inconsistent.
        DrivePowers newPowers = new DrivePowers(
                powers.forward(),
                -powers.strafe(),
                -powers.turn()

        );
        super.applyDrive(newPowers);
    }

    public void setDataLogger(DataLogger logger) {
        for (SwervePod pod: pods) {
            if (pod instanceof CoaxialPod) {
                ((CoaxialPod)pod).setDataLogger(logger);
            }
        }
    }
}
