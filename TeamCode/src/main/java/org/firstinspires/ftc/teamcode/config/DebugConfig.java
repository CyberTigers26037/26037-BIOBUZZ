package org.firstinspires.ftc.teamcode.config;

public class DebugConfig{
    private static boolean debugEnabled = true;


    public static boolean isDebugEnabled() {
        return debugEnabled;
    }

    public static void setDebugEnabled(boolean debugEnabled) {
        DebugConfig.debugEnabled = debugEnabled;
    }
}
