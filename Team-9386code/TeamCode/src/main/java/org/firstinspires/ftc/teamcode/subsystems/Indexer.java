package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.opmodes.RobotBase;

public class Indexer {
    protected HardwareMap hardwareMap;
    protected Telemetry telemetry;
    protected RobotBase robotBase;
    public IndexerSystem indexerSystem;

    public Indexer(HardwareMap hardwareMap, RobotBase opMode) {
        this.hardwareMap = hardwareMap;
        this.robotBase = opMode;
        this.telemetry = robotBase.telemetry;

        initHardware();
    }

    public Indexer(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
    }

    protected void initHardware() {
        indexerSystem = new IndexerSystem();

    }

    public class IndexerSystem {
        public DcMotor indexerMotor;
        public Servo rightLifter;
        public Servo leftLifter;
        public ColorSensor rightColorSensor;
        public ColorSensor leftColorSensor;

        public IndexerSystem() { //HardwareMap hardwareMap, RobotBase opMode
            initHardware();
        }


        protected void initHardware() {
            indexerMotor = hardwareMap.get(DcMotor.class, "TriggerRoller");
            indexerMotor.setDirection(DcMotor.Direction.FORWARD);
            indexerMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            leftColorSensor = hardwareMap.get(ColorSensor.class, "leftColor");

            rightColorSensor = hardwareMap.get(ColorSensor.class, "rightColor");

            leftLifter = hardwareMap.get(Servo.class, "leftLifter");

            rightLifter = hardwareMap.get(Servo.class, "rightLifter");
            rightLifter.setDirection(Servo.Direction.REVERSE);

            goToTarget(0,0.875,0.875);
        }

        public void goToTarget(double indexerPower, double leftLifterHeight, double rightLifterHeight) {
                indexerMotor.setPower(indexerPower);
                leftLifter.setPosition(leftLifterHeight);
                rightLifter.setPosition(rightLifterHeight);
            }
        }

    }

