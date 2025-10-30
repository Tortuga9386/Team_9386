package org.firstinspires.ftc.teamcode.opmodes;

import android.util.Log;

import com.qualcomm.hardware.bosch.BNO055IMU;

import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.subsystems.*;

import java.util.List;
import java.util.Locale;

/**
 * This is NOT an opmode.
 *
 * This class initiates all the specific hardware for a robot.
 * The various subsystems classes are located in the subsystem folder.
 * This code is used by all the other opmodes.
 *
 */
@Disabled
public class RobotBase
{
    //Define constants that should NOT be adjusted by manual calibration
    protected boolean           INITIALIZE_IMU      = true;
    protected boolean           INITIALIZE_DRIVE    = true;

    //Make subsystems available to all class extensions
    public Drive drive;
    //public Lift lift;
    //public SensorHuskyLens sensorHuskyLens;
    private final HardwareMap hMap;
    private final OpMode contextOpMode;

    /* Constructor */
    public RobotBase(HardwareMap hardwareMap, OpMode opMode) {
        this.hMap = hardwareMap;
        this.contextOpMode = opMode;

        drive = new Drive(hMap, contextOpMode);
        }
    }