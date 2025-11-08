package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.opmodes.RobotBase;
import org.firstinspires.ftc.teamcode.opmodes.*;

public class Auto_paths {

    //Inherited data objects
    protected HardwareMap hardwareMap;
    public Telemetry telemetry;
    protected RobotBase robotBase;

    //Motor object definitions
    SparkFunOTOS myOtos;
    SparkFunOTOS myOtos2;
    public DcMotor leftFrontMotor;
    public DcMotor rightFrontMotor;
    public DcMotor leftRearMotor;
    public DcMotor rightRearMotor;


    //mods
    private final ElapsedTime time = new ElapsedTime();

    public Auto_paths(HardwareMap hardwareMap, RobotBase opMode) {
        this.hardwareMap = hardwareMap;
        this.robotBase = opMode;
        this.telemetry = robotBase.telemetry;

        initHardware();
    }

    public void initHardware() {



        myOtos = hardwareMap.get(SparkFunOTOS.class, "LeftOtos");
        myOtos2 = hardwareMap.get(SparkFunOTOS.class, "RightOtos");

        try {
            leftFrontMotor = hardwareMap.get(DcMotorEx.class, "FL");
            leftFrontMotor.setDirection(DcMotor.Direction.FORWARD);
        } catch (Exception e) {
//            Log.v("Drive", ":leftFrontMotor init failed");
        }

        try {
            leftRearMotor = hardwareMap.get(DcMotorEx.class, "BL");
            leftRearMotor.setDirection(DcMotor.Direction.FORWARD);
        } catch (Exception e){
//            Log.v("Drive", ":leftRearMotor init failed");
        }

        try {
            rightFrontMotor = hardwareMap.get(DcMotorEx.class, "FR");
            rightFrontMotor.setDirection(DcMotor.Direction.REVERSE);
        } catch (Exception e){
//            Log.v("Drive", ":rightFrontMotor init failed");
        }

        try {
            rightRearMotor = hardwareMap.get(DcMotorEx.class, "BR");
            rightRearMotor.setDirection(DcMotor.Direction.REVERSE);
        } catch (Exception e){
//            Log.v("Drive", ":rightRearMotor init failed");
        }
        leftFrontMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRearMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRearMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        myOtos.setLinearUnit(DistanceUnit.INCH);
        myOtos2.setLinearUnit(DistanceUnit.INCH);

        myOtos.setAngularUnit(AngleUnit.DEGREES);
        myOtos2.setAngularUnit(AngleUnit.DEGREES);

        configureOtos();


    }


    public void customWait (long seconds){
        try {
            // Use Thread.sleep() to pause the execution for 1000 milliseconds (1 second).
            Thread.sleep(seconds * 1000);
        } catch (InterruptedException e) {
            // Important to handle the interruption gracefully
            Thread.currentThread().interrupt();
        }
    }

    private void configureOtos() {
        telemetry.addLine("Configuring OTOS...");
        telemetry.update();


        myOtos.setLinearUnit(DistanceUnit.INCH);
        myOtos2.setLinearUnit(DistanceUnit.INCH);

        myOtos.setAngularUnit(AngleUnit.DEGREES);
        myOtos2.setAngularUnit(AngleUnit.DEGREES);


        SparkFunOTOS.Pose2D offset = new SparkFunOTOS.Pose2D(-7.65827, 3.50149, 0);
        myOtos.setOffset(offset);

        SparkFunOTOS.Pose2D offset2 = new SparkFunOTOS.Pose2D(7.61476, 3.50149, 0);
        myOtos2.setOffset(offset2);

        myOtos.setLinearScalar(1.0);
        myOtos.setAngularScalar(1.0);

        myOtos2.setLinearScalar(1.0);
        myOtos2.setAngularScalar(1.0);

        myOtos.calibrateImu();

        myOtos2.calibrateImu();

        myOtos.resetTracking();

        myOtos2.resetTracking();

        SparkFunOTOS.Pose2D currentPosition = new SparkFunOTOS.Pose2D(0, 0, 0);
        myOtos.setPosition(currentPosition);

        SparkFunOTOS.Pose2D currentPosition2 = new SparkFunOTOS.Pose2D(0, 0, 0);
        myOtos.setPosition(currentPosition2);
    }

    public void moveToPos (double userH, double userX, double userY) {


        double headingAverage = (((myOtos.getPosition().h + myOtos2.getPosition().h)/2));

        double xAverage = (((myOtos.getPosition().x + myOtos2.getPosition().x)/2));

        double yAverage = (((myOtos.getPosition().y + myOtos2.getPosition().y)/2));

        double forward  = ((-yAverage + userY)/4);

        double strafe =  ((xAverage + -userX)/4);

        double twist =   ((headingAverage + userH)/10);


        telemetry.addData("forward", forward);
        telemetry.addData("strafe", strafe);
        telemetry.addData("twist", twist);


        double[] speeds = {
                (forward + strafe + twist),
                (forward - strafe - twist),
                (forward - strafe + twist),
                (forward + strafe - twist)
        };

        // Loop through all values in the speeds[] array and find the greatest
        // *magnitude*.  Not the greatest velocity.
        double max = Math.abs(speeds[0]);
        for(int i = 0; i < speeds.length; i++) {
            if ( max < Math.abs(speeds[i]) ) max = Math.abs(speeds[i]);
        }

        // If and only if the maximum is outside of the range we want it to be,
        // normalize all the other speeds based on the given speed value.
        if (max > 1) {
            for (int i = 0; i < speeds.length; i++) speeds[i] /= max;
        }

        // apply the calculated values to the motors.
        leftFrontMotor.setPower(speeds[0]);
        rightFrontMotor.setPower(speeds[1]);
        leftRearMotor.setPower(speeds[2]);
        rightRearMotor.setPower(speeds[3]);
    }


    public boolean isBusy() {
        return true;
    }

    public void stop() {
        leftFrontMotor.setPower(0);
        leftRearMotor.setPower(0);
        rightFrontMotor.setPower(0);
        rightRearMotor.setPower(0);
    }

    public void reset() {
        initHardware();
    }
}
