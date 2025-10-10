package org.firstinspires.ftc.teamcode.opmodes;

import android.util.Log;


import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;


public class AutoPath {

    //Inherited data objects
    protected HardwareMap hardwareMap;
    public Telemetry telemetry;
    protected RobotBase robotBase;
    public Pose2d startPose;

    public enum PathMethod {
        AutoPathEx,
        AutoPathFtcLib,
        AutoPathRoadRunner
    }
    public PathMethod pathMethod;

    public String targetLabel;

    public enum StartPosition {
        NONE,
        LEFT,
        RIGHT,
        RED_LEFT,
        RED_RIGHT,
        BLUE_LEFT,
        BLUE_RIGHT
    }
    //public StartPosition startPosition = StartPosition.NONE;

    public enum PathName {
        NONE,
        RED_LEFT_SPIKE, RED_LEFT_ONE, RED_LEFT_TWO, RED_LEFT_THREE, RED_LEFT_NONE,
        RED_RIGHT_SPIKE, RED_RIGHT_ONE, RED_RIGHT_TWO, RED_RIGHT_THREE, RED_RIGHT_NONE,
        BLUE_LEFT_SPIKE, BLUE_LEFT_ONE, BLUE_LEFT_TWO, BLUE_LEFT_THREE, BLUE_LEFT_NONE,
        BLUE_RIGHT_SPIKE, BLUE_RIGHT_ONE, BLUE_RIGHT_TWO, BLUE_RIGHT_THREE, BLUE_RIGHT_NONE
    }
    public PathName pathName = PathName.NONE;
    public boolean enableDriveToRead = false;

    public AutoPath(HardwareMap hardwareMap, RobotBase opMode) {
        this.hardwareMap = hardwareMap;
        this.robotBase = opMode;
    }

    public void getAutoPaths(PathName pathName) {
        Log.v("AutoPath", ":getAutoPaths(PathName pathName) not implemented");
    }
}
