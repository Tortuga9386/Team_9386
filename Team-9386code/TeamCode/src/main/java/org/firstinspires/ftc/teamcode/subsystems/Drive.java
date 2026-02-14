package org.firstinspires.ftc.teamcode.subsystems;

import android.util.Log;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.opmodes.RobotBase;

public class Drive {
    SparkFunOTOS leftOtos;
    SparkFunOTOS rightOtos;
    //Inherited data objects
    protected HardwareMap hardwareMap;
    public Telemetry telemetry;
    protected RobotBase robotBase;

    //Motor object definitions
    public DcMotor leftFrontMotor;
    public DcMotor rightFrontMotor;
    public DcMotor leftRearMotor;
    public DcMotor rightRearMotor;

    public Limelight3A limelight3A;

    //mods
    public final ElapsedTime time = new ElapsedTime();


    public double limeLightOffset;

    public Drive(HardwareMap hardwareMap, RobotBase opMode) {
        this.hardwareMap = hardwareMap;
        this.robotBase = opMode;
        this.telemetry = robotBase.telemetry;

        initHardware();
    }

    protected void initHardware() {
        try {
            leftFrontMotor = hardwareMap.get(DcMotorEx.class, "FL");
            leftFrontMotor.setDirection(DcMotor.Direction.REVERSE);
        } catch (Exception e) {
//            Log.v("Drive", ":leftFrontMotor init failed");
        }

        try {
            leftRearMotor = hardwareMap.get(DcMotorEx.class, "BL");
            leftRearMotor.setDirection(DcMotor.Direction.REVERSE);
        } catch (Exception e){
//            Log.v("Drive", ":leftRearMotor init failed");
        }

        try {
            rightFrontMotor = hardwareMap.get(DcMotorEx.class, "FR");
        } catch (Exception e){
//            Log.v("Drive", ":rightFrontMotor init failed");
        }

        try {
            rightRearMotor = hardwareMap.get(DcMotorEx.class, "BR");
        } catch (Exception e){
//            Log.v("Drive", ":rightRearMotor init failed");
        }
        leftFrontMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRearMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRearMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftOtos = hardwareMap.get(SparkFunOTOS.class, "leftOtos");
        rightOtos = hardwareMap.get(SparkFunOTOS.class, "rightOtos");

        leftOtos.setLinearUnit(DistanceUnit.INCH);
        rightOtos.setLinearUnit(DistanceUnit.INCH);

        leftOtos.setAngularUnit(AngleUnit.DEGREES);
        rightOtos.setAngularUnit(AngleUnit.DEGREES);

        SparkFunOTOS.Pose2D offsetRight = new SparkFunOTOS.Pose2D(-7.65827, 3.58023, 0);
        SparkFunOTOS.Pose2D offsetLeft = new SparkFunOTOS.Pose2D(7.61476, 3.58023, 0);

        leftOtos.setOffset(offsetLeft);
        rightOtos.setOffset(offsetRight);

        leftOtos.calibrateImu();
        rightOtos.calibrateImu();

        leftOtos.resetTracking();
        rightOtos.resetTracking();

        //Limelight
        limelight3A = hardwareMap.get(Limelight3A.class, "limelight");
        limelight3A.getLatestResult();
        limelight3A.setPollRateHz(100);
        limelight3A.start();
    }

    public void moveToPos (double y, double x, double r, double FGain, double SGain, double RGain, boolean limeLight) {

        //averages
        double avForward = (leftOtos.getPosition().y + rightOtos.getPosition().y) / 2.0;
        double avStrafe = (leftOtos.getPosition().x + rightOtos.getPosition().x) / 2.0;
        double avTwist = (leftOtos.getPosition().h + rightOtos.getPosition().h) / 2.0;

        //degrees to radians
        double convertedTwist = Math.toRadians(avTwist + 90);

        //calculations
        double posForward = (((-avForward + y) * Math.sin(convertedTwist))  + ((-avStrafe + x) * Math.cos(convertedTwist)));
        double posStrafe = (((-avStrafe + x) * Math.sin(convertedTwist))  - ((-avForward + y) * Math.cos(convertedTwist)));
        double posTwist = (-avTwist + r);

        //telemetry
        telemetry.addData("f", posForward);
        telemetry.addData("s", posStrafe);
        telemetry.addData("t", posTwist);
        telemetry.addData("t2", avTwist);
        telemetry.addData("tc", convertedTwist);

        telemetry.update();

        //run motors
        if (!limeLight) {
            runMotors(-posForward * FGain, -posStrafe * SGain, posTwist / RGain);
        }
        if (limeLight){
            runMotors(-posForward * FGain, -posStrafe * SGain, (-limelight3A.getLatestResult().getTx() + limeLightOffset)/27.25);
        }
    }

    public void backShotRed (){
        if (time.seconds() > 0 && time.seconds() < 0.75){
            moveToPos(-4,-4,-21,0.125,0.125,22.5, false);
        }
        if (time.seconds() > 0.75 && time.seconds() < 3.9 ){
            moveToPos(-4,-4,0,0.125,0.125,50, true);
        }
        if (time.seconds() > 3.9 && time.seconds() < 5.75){
            moveToPos(-22,-18,90,0.25,0.25,22.5,false);
        }
        if (time.seconds() > 5.75 && time.seconds() < 8.5){
            moveToPos(-22,-55,90,0.02,0.25,22.5,false);
        }

        if (time.seconds() > 8.5 && time.seconds() < 12){
            moveToPos(-4,-4,-20.5,0.25,0.25,22.5,false);
        }

        if (time.seconds() > 12 && time.seconds() < 15.15){
            moveToPos(-4,-4,-0,0.125,0.125,50,true);
        }
        if (time.seconds() > 15.15 && time.seconds() < 18){
            moveToPos(-18,-18,130,0.25,0.25,22.5,false);
        }

    }

    public void backShotRed2 (){
        if (time.seconds() > 0 && time.seconds() < 0.75){
            moveToPos(-4,-4,-21,0.125,0.125,22.5, false);
        }
        if (time.seconds() > 0.75 && time.seconds() < 3.9 ){
            moveToPos(-4,-4,0,0.125,0.125,50, true);
        }
        if (time.seconds() > 3.9 && time.seconds() < 5.75){
            moveToPos(-22,-18,90,0.25,0.25,22.5,false);
        }
        if (time.seconds() > 5.75 && time.seconds() < 8.5){
            moveToPos(-22,-55,90,0.02,0.25,22.5,false);
        }

        if (time.seconds() > 8.5 && time.seconds() < 12){
            moveToPos(-4,-4,-20.5,0.25,0.25,22.5,false);
        }

        if (time.seconds() > 12 && time.seconds() < 15.15){
            moveToPos(-4,-4,-0,0.125,0.125,50,true);
        }
        if (time.seconds() > 15.15 && time.seconds() < 18){
            moveToPos(-18,-18,130,0.25,0.25,22.5,false);
        }

    }


    public void backShotBlue (){
        if (time.seconds() > 0 && time.seconds() < 0.75){
            moveToPos(-4,4,21,0.125,0.125,22.5, false);
        }
        if (time.seconds() > 0.75 && time.seconds() < 3.9 ){
            moveToPos(-4,4,0,0.125,0.125,50, true);
        }
        if (time.seconds() > 3.9 && time.seconds() < 5.75){
            moveToPos(-22,18,-90,0.25,0.25,22.5,false);
        }
        if (time.seconds() > 5.75 && time.seconds() < 8.5){
            moveToPos(-22,55,-90,0.02,0.25,22.5,false);
        }

        if (time.seconds() > 8.5 && time.seconds() < 12){
            moveToPos(-4,4,20.5,0.25,0.25,22.5,false);
        }

        if (time.seconds() > 12 && time.seconds() < 15.15){
            moveToPos(-4,4,0,0.125,0.125,50,true);
        }
        if (time.seconds() > 15.15 && time.seconds() < 18){
            moveToPos(-18,18,-130,0.25,0.25,22.5,false);
        }
    }

    public void backShotBlue2 () {
        if (time.seconds() > 0 && time.seconds() < 0.75){
            moveToPos(-4,4,21,0.125,0.125,22.5, false);
        }
        if (time.seconds() > 0.75 && time.seconds() < 3.9 ){
            moveToPos(-4,4,0,0.125,0.125,50, true);
        }
        if (time.seconds() > 3.9 && time.seconds() < 5.75){
            moveToPos(-22,18,-90,0.25,0.25,22.5,false);
        }
        if (time.seconds() > 5.75 && time.seconds() < 8.5){
            moveToPos(-22,55,-90,0.02,0.25,22.5,false);
        }

        if (time.seconds() > 8.5 && time.seconds() < 12){
            moveToPos(-4,4,20.5,0.25,0.25,22.5,false);
        }

        if (time.seconds() > 12 && time.seconds() < 15.15){
            moveToPos(-4,4,-0,0.125,0.125,50,true);
        }
        if (time.seconds() > 15.15 && time.seconds() < 18){
            moveToPos(-18,18,-130,0.25,0.25,22.5,false);
        }
    }

    public void driveFromGamepad(Gamepad gamepad, double limeLightR) {

        double forwardRM;
        double strafeRM;
        double twistRM;
        double limeLightT;


        forwardRM = gamepad.left_stick_y;
        strafeRM = -gamepad.left_stick_x;
        twistRM = -gamepad.right_stick_x;

        limeLightT = twistRM - limeLightR;

        runMotors(forwardRM, strafeRM, limeLightT);
    }


    public void runMotors (double forward, double strafe, double twist) {


        double[] speeds = {
                (forward + strafe + twist),
                (forward - strafe - twist),
                (forward - strafe + twist),
                (forward + strafe - twist)
        };
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
