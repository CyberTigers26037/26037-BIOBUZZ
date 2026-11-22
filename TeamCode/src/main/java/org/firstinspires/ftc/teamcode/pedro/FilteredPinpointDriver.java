package org.firstinspires.ftc.teamcode.pedro;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchSimple;
import com.qualcomm.robotcore.hardware.configuration.annotations.DeviceProperties;
import com.qualcomm.robotcore.hardware.configuration.annotations.I2cDeviceType;

@I2cDeviceType
@DeviceProperties(
        name = "Filtered goBILDA Pinpoint",
        xmlTag = "goBILDA_Pinpoint",
        description = "Custom bulk-read filtered Pinpoint driver"
)
@SuppressWarnings("unused")
public class FilteredPinpointDriver extends GoBildaPinpointDriver {
    private static long updateCount;
    private static long updateTimeNs;

    public FilteredPinpointDriver(I2cDeviceSynchSimple deviceClient, boolean deviceClientIsOwned) {
        super(deviceClient, deviceClientIsOwned);

        // Set the bulk read scope to only include what Pedro Pathing PinpointLocalizer uses...
        setBulkReadScope(
                Register.X_POSITION,
                Register.Y_POSITION,
                Register.H_ORIENTATION,
                Register.X_VELOCITY,
                Register.Y_VELOCITY,
                Register.H_VELOCITY
        );
    }

    @Override
    public void update() {
        // this method override is solely for timing measurements...
        long startTime = System.nanoTime();
        super.update();
        updateTimeNs += System.nanoTime() - startTime;
        updateCount++;
    }

    public static double getAverageUpdateTimeMilliseconds() {
        return (double) updateTimeNs / updateCount * 1e-6;
    }
}
