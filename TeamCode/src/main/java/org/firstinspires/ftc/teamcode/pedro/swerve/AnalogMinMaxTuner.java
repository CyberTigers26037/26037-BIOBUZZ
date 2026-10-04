package org.firstinspires.ftc.teamcode.pedro.swerve;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;

import org.firstinspires.ftc.teamcode.pedro.Constants;

import java.util.List;

/**
 * Tuning OpMode to get the min and max encoder values for swerve pods
 * @author Kabir Goyal
 * @author Havish Sripada
 */
@SuppressWarnings("unused")
@TeleOp
public class AnalogMinMaxTuner extends OpMode {
    //populate the below with your names for the servos and encoders
    public String[] encoderNames = {
            Constants.leftFront.servoEncoderName.get(),
            Constants.rightFront.servoEncoderName.get(),
            Constants.leftBack.servoEncoderName.get(),
            Constants.rightBack.servoEncoderName.get()
    };
    public String[] servoNames = {
            Constants.leftFront.servoName.get(),
            Constants.rightFront.servoName.get(),
            Constants.leftBack.servoName.get(),
            Constants.rightBack.servoName.get()
    };
    public AnalogInput[] encoders = new AnalogInput[encoderNames.length];
    public CRServo[] servos = new CRServo[servoNames.length];
    public double[] minVoltages = new double[encoderNames.length];
    public double[] maxVoltages = new double[encoderNames.length];

    public List<LynxModule> lynxModules; //js to improve loop times a bit yk

    @Override
    public void init_loop() {
        telemetry.addLine("Press START. Then, Spin each pod slowly for 4 to 5 full rotations.\n" +
                "The OpMode will keep track of the min and max voltages seen so far and print them to telemetry.");
        telemetry.update();
    }

    @Override
    public void init() {
        lynxModules = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : lynxModules) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        for (int i = 0; i < encoders.length; i++)  {
            encoders[i] = hardwareMap.get(AnalogInput.class, encoderNames[i]);
            minVoltages[i] = 5; //bigger value than should ever be read
        }
        // ANP, 10/3/2026
        // Axon servos (and maybe others) do not provide encoder feedback unless the
        // servo is powered. So, we need to set power to 0 on all the servos to get
        // encoder feedback.
        for (int i = 0; i < servos.length; i++)  {
            servos[i] = hardwareMap.get(CRServo.class, servoNames[i]);
            servos[i].setPower(0);
        }
    }

    /**
     * This runs the OpMode, updating the Follower as well as printing out the debug statements to
     * the Telemetry, as well as the FTC Dashboard.
     */
    @Override
    public void loop() {
        for (LynxModule hub : lynxModules) {
            hub.clearBulkCache();
        }

        telemetry.addLine("Spin each pod slowly for 4 to 5 full rotations.\n" +
                "The OpMode will keep track of the min and max voltages seen so far and print them to telemetry.\n\n");

        for (int i = 0; i < encoders.length; i++) {
            double currentVoltage = encoders[i].getVoltage();
            minVoltages[i] = Math.min(minVoltages[i], currentVoltage);
            maxVoltages[i] = Math.max(maxVoltages[i], currentVoltage);
            telemetry.addData(encoderNames[i] + " current value:", currentVoltage);
            telemetry.addData(encoderNames[i] + " min value:", minVoltages[i]);
            telemetry.addData(encoderNames[i] + " max value:", maxVoltages[i]);
            telemetry.addLine("");
        }

        telemetry.update();
    }
}