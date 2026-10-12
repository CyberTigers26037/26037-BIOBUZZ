package org.firstinspires.ftc.teamcode.pedro.swerve;

import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.revhub.drivetrains.Swerve;
import com.pedropathing.revhub.drivetrains.SwerveConfig;
import com.pedropathing.revhub.drivetrains.SwervePod;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class CyberTigersSwerve extends Swerve {
    public CyberTigersSwerve(HardwareMap hardwareMap, SwerveConfig config, SwervePod... pods) {
        super(hardwareMap, config, pods);
    }

    public void applyDrive(DrivePowers powers) {
        // ANP, 10/12/2026, Swerve reverses the strafe and turn internally
        // which makes Mecanum and Swerve inconsistent.
        DrivePowers newPowers = new DrivePowers(
                powers.forward(),
                -powers.strafe(),
                -powers.turn()
        );
        super.applyDrive(powers);
        
    }
}
