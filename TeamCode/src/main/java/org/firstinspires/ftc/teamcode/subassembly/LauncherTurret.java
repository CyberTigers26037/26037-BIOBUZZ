package org.firstinspires.ftc.teamcode.subassembly;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.servos.RTPAxon;

public class LauncherTurret {
    // Device Config Constants
    private static final double SERVO_DEG_PER_TURRET_DEG = 2.75;
    private boolean isError = false;

    private DigitalChannel limitSwitch;
    private boolean initialized;
    private RTPAxon turretServo;
    private ArtifactLauncher launcher;
    private String deviceName;

    public LauncherTurret(HardwareMap hardwareMap, String devName) {
        deviceName = devName;
        limitSwitch = hardwareMap.get(DigitalChannel.class, "turretSwitch" + deviceName);
        limitSwitch.setMode(DigitalChannel.Mode.INPUT);
        CRServo servo = hardwareMap.get(CRServo.class, "turretServo" + deviceName);
        AnalogInput encoder = hardwareMap.get(AnalogInput.class, "turretEncoder" + deviceName);

        turretServo = new RTPAxon(servo, encoder, 0, false, devName);
        turretServo.setMaxPower(1.0);
        turretServo.setPidCoeffs(0.007, 0.00041, 0.0002);
        launcher = new ArtifactLauncher(hardwareMap,"flywheelMotor" + deviceName);
        init();
    }
    public void init(){
        if (limitSwitch.getState()) {
            isError = true;
        }
    }

    public void loop() {
        if (isError){ return;} //refused to oporate if an error
        if (!initialized) {
            turretServo.setPower(-0.1);
            turretServo.trackAngle();

            if (limitSwitch.getState()) {
                turretServo.setPower(0);
                turretServo.resetHomeAngle();
                initialized = true;
            }
            return;
        }
        turretServo.update();
    }

    public boolean isInitialized() {
        return initialized;
    }

    public synchronized void rehome() {
        initialized = false;
        turretServo.setPower(0);
    }
    public void setTurretAngle(double angle) {
        if (!initialized) return;

        turretServo.setTargetRotation(angle * SERVO_DEG_PER_TURRET_DEG);
    }
    public void setLauncherRpm (int rpm){
        launcher.adjustFlywheelRpm(rpm);
    }
    public int getLauncherRpm(){
        return launcher.getFlywheelRpm();
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
        if (isError){
            telemetry.setDisplayFormat(Telemetry.DisplayFormat.HTML);
            telemetry.addData("WARNING TURRET " + deviceName, "<h1><font color ='red'>TURRET LIMIT SWITCH PRESSED</font></h1>");
        }
        telemetry.addData("limitSwitch" + deviceName, limitSwitch.getState());
        telemetry.addData("isInitialized" + deviceName, isInitialized());
        telemetry.addData("turretRpm" + deviceName, launcher.getFlywheelRpm());
        telemetry.addData("actualTurretRpm" + deviceName, launcher.getActualFlywheelRpm());
        telemetry.addLine(turretServo.log());
    }
}
