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
        public DcMotor leftLifterMotor;
        public DcMotor rightLifterMotor;

        public ColorSensor leftColorSensor;
        public ColorSensor rightColorSensor;

        public TouchSensor leftMagSensor;
        public TouchSensor rightMagSensor;


        private final ElapsedTime sequenceTimer = new ElapsedTime();

        public IndexerSystem() { //HardwareMap hardwareMap, RobotBase opMode
            initHardware();
        }

        protected void initHardware() {
        leftLifterMotor = hardwareMap.get(DcMotor.class, "leftLifter");
        rightLifterMotor = hardwareMap.get(DcMotor.class, "rightLifter");

        leftLifterMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightLifterMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftColorSensor = hardwareMap.get(ColorSensor.class, "leftColorSensor");
        rightColorSensor = hardwareMap.get(ColorSensor.class, "rightColorSensor");

        leftMagSensor = hardwareMap.get(TouchSensor.class,"leftMagSensor");
        rightMagSensor = hardwareMap.get(TouchSensor.class,"rightMagSensor");

        }

        public void goToTarget(double leftLifterSpeed, double rightLifterSpeed, int leftLifterHeight, int rightLifterHeight, boolean lifterMode) {
                if (lifterMode){
                leftLifterMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                rightLifterMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

                leftLifterMotor.setPower(leftLifterSpeed);
                rightLifterMotor.setPower(rightLifterSpeed);
                }
                if (!lifterMode){
                    leftLifterMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    rightLifterMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

                    leftLifterMotor.setTargetPosition(leftLifterHeight);
                    rightLifterMotor.setTargetPosition(rightLifterHeight);
                }
            }
        }

    }

