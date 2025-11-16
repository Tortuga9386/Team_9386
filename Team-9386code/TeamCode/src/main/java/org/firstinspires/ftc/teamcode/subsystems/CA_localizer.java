package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.opmodes.RobotBase;

//public class CA_localizer {
    protected HardwareMap hardwareMap;
    protected Telemetry telemetry;
    protected RobotBase robotBase;
    public Otos otos;

    public CA_localizer(HardwareMap hardwareMap, RobotBase opMode) {
        this.hardwareMap = hardwareMap;
        this.robotBase = opMode;
        this.telemetry = robotBase.telemetry;

        initHardware();
    }

    public CA_localizer(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
    }

    protected void initHardware() {
        otos  = new Otos();
    }

    public class Otos {

        SparkFunOTOS leftOtos;
        SparkFunOTOS rightOtos;

        public Otos() { //HardwareMap hardwareMap, RobotBase opMode
            initHardware();
        }





        protected void initHardware() {
            leftOtos = hardwareMap.get(SparkFunOTOS.class,"LeftOtos");
            rightOtos = hardwareMap.get(SparkFunOTOS.class,"RightOtos");

            SparkFunOTOS.Pose2D pos = leftOtos.getPosition();
            SparkFunOTOS.Pose2D pos2 = rightOtos.getPosition();

            // myOtos.setLinearUnit(DistanceUnit.METER);
            leftOtos.setLinearUnit(DistanceUnit.INCH);
            rightOtos.setLinearUnit(DistanceUnit.INCH);
            // myOtos.setAngularUnit(AnguleUnit.RADIANS);
            leftOtos.setAngularUnit(AngleUnit.DEGREES);
            rightOtos.setAngularUnit(AngleUnit.DEGREES);

            SparkFunOTOS.Pose2D offset = new SparkFunOTOS.Pose2D(-7.65827, 3.50149, 0);
            leftOtos.setOffset(offset);

            SparkFunOTOS.Pose2D offset2 = new SparkFunOTOS.Pose2D(7.61476, 3.50149, 0);
            rightOtos.setOffset(offset2);

            leftOtos.setLinearScalar(1.0);
            rightOtos.setAngularScalar(1.0);

            leftOtos.setLinearScalar(1.0);
            rightOtos.setAngularScalar(1.0);

            leftOtos.calibrateImu();

            rightOtos.calibrateImu();

            leftOtos.resetTracking();
            rightOtos.resetTracking();

            SparkFunOTOS.Pose2D currentPosition = new SparkFunOTOS.Pose2D(0, 0, 0);
            leftOtos.setPosition(currentPosition);

            SparkFunOTOS.Pose2D currentPosition2 = new SparkFunOTOS.Pose2D(0, 0, 0);
            rightOtos.setPosition(currentPosition2);
            }

        public void doOtosStuff() {


        }





        }

        }

