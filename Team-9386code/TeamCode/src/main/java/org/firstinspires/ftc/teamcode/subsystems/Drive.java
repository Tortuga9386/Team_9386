package org.firstinspires.ftc.teamcode.subsystems;

import android.util.Log;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
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

    //mods
    private final ElapsedTime time = new ElapsedTime();

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

        leftOtos = hardwareMap.get(SparkFunOTOS.class, "LeftOtos");
        rightOtos = hardwareMap.get(SparkFunOTOS.class, "RightOtos");

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
    }

    public void moveToPos (double y, double x, double r,Gamepad gamepad1) {

        //averages
        double avForward = (leftOtos.getPosition().y + rightOtos.getPosition().y) / 2.0;
        double avStrafe = (leftOtos.getPosition().x + rightOtos.getPosition().x) / 2.0;
        double avTwist = (leftOtos.getPosition().h + rightOtos.getPosition().h) / 2.0;

        //degrees to radians
        double convertedTwist = Math.toRadians(avTwist);

        //calculations
        double posForward = (((-avForward + y) * Math.sin(convertedTwist)) /* + ((avStrafe + x) * Math.cos(otosTwist))*/);
        double posStrafe = (((-avStrafe + x) * Math.cos(convertedTwist)) /* + ((avStrafe + x) * Math.sin(otosTwist))*/);
        double posTwist = (-avTwist + r);

        //telemetry
        telemetry.addData("f", posForward);
        telemetry.addData("s", posStrafe);
        telemetry.addData("t", posTwist);
        telemetry.addData("t2", avTwist);
        telemetry.addData("tc", convertedTwist);

        //run motors
        runMotors(0, 0, gamepad1.right_stick_x);
    }

    public void driveFromGamepad(Gamepad gamepad) {

        double forward = 0;
        double strafe = 0;
        double twist = 0;

        forward = gamepad.left_stick_y;
        strafe = gamepad.left_stick_x;
        twist = -gamepad.right_stick_x;


        runMotors(forward, strafe, twist);
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
