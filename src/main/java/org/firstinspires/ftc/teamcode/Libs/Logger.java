package org.firstinspires.ftc.teamcode.Libs;

/**
 * Created by caseyzandbergen on 11/15/16.
 */

public class Logger {
    private DataLogger ltDl;               // Linear Time Data Logger
    private int csReadCount = 0;
    private long csTimeStamp;        // In microseconds
    private int clear, red, green, blue;
    private int dsReadCount = 0;
    private long dsTimeStamp;        // In microseconds
    private int distance;
    private long pingTime;
    private double robotX, robotY;

    public void init() {

    }
}
