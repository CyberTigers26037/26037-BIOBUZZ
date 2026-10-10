package org.firstinspires.ftc.teamcode.subassembly;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.servos.RTPAxon;

public class LauncherTurret {
    private DigitalChannel limitSwitch;
    private boolean initialized;
    private RTPAxon turretServo;
    private ArtifactLauncher launcher;
    private String deviceNumber;

    public LauncherTurret(HardwareMap hardwareMap, String devNum) {
        deviceNumber = devNum;
        limitSwitch = hardwareMap.get(DigitalChannel.class, "turretSwitch" + deviceNumber);
        limitSwitch.setMode(DigitalChannel.Mode.INPUT);
        CRServo servo = hardwareMap.get(CRServo.class, "turretServo" + deviceNumber);
        AnalogInput encoder = hardwareMap.get(AnalogInput.class, "turretEncoder" + deviceNumber);

        turretServo = new RTPAxon(servo, encoder, 0, false);
        turretServo.setMaxPower(1.0);
        turretServo.setPidCoeffs(0.007, 0.00041, 0.0002);
        launcher = new ArtifactLauncher(hardwareMap,"flywheelMotor" + deviceNumber);
    }

    public void loop() {
        if (!initialized) {
            turretServo.setPower(-0.1);

            if (limitSwitch.getState()) {
                turretServo.setPower(0);
                turretServo.resetHomeAngle();
                initialized = true;
            }
            return;
        }
        turretServo.update();
    }
    public void setTurretAngle(double angle) {
        turretServo.setTargetRotation(angle*2.75);
    }
    public void toggleLauncher () {
        if (launcher.isRunning()) {
            launcher.stopFlywheelMotor();
        } else {
            launcher.startFlywheelMotor();
        }
//        launcher.setFlywheelRpm(3000);
    }

    public void outputTelemetry(Telemetry telemetry) {
        telemetry.addData("limitSwitch" + deviceNumber, limitSwitch.getState());
        telemetry.addLine(turretServo.log());
    }
}
