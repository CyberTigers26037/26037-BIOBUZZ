package org.firstinspires.ftc.teamcode.config;

public class DebugConfig {
    private static boolean debugEnabled;


    public static boolean isDebugEnabled() {
        return debugEnabled;
    }

    public static void setDebugEnabled(boolean debugEnabled) {
        DebugConfig.debugEnabled = debugEnabled;
    }
}
