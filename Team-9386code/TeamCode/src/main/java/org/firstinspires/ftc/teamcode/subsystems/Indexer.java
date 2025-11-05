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


        private final ElapsedTime sequenceTimer = new ElapsedTime();

        public IndexerSystem() { //HardwareMap hardwareMap, RobotBase opMode
            initHardware();
        }

        public double indexerPower = 0;
        public boolean triggerRollerForward = false;

        public double shooterSelection = 0;
        public double gamepadSelection = 0;
        public double lifterPos;


        protected void initHardware() {
            indexerMotor = hardwareMap.get(DcMotor.class, "TriggerRoller");
            indexerMotor.setDirection(DcMotor.Direction.FORWARD);
            indexerMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            indexerMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            leftColorSensor = hardwareMap.get(ColorSensor.class, "leftColor");
            rightColorSensor = hardwareMap.get(ColorSensor.class, "rightColor");

            leftLifter = hardwareMap.get(Servo.class, "leftLifter");
            rightLifter = hardwareMap.get(Servo.class, "rightLifter");
            rightLifter.setDirection(Servo.Direction.REVERSE);


        }

        public void colorSensorStuff(){

        }
        public void doIndexerStuff(Gamepad gamepad2) {
            goToSpeedTriggerTarget(indexerPower, lifterPos);


            //Lifters

            if (gamepad2.a){
                lifterPos = 0;
            }

            if (!gamepad2.a){
                lifterPos = 0.7;
            }

            if (gamepad2.right_trigger > 0.25 || gamepad2.right_bumper){
                triggerRollerForward = true;
            }

            if (gamepad2.right_trigger < 0.25 && !gamepad2.right_bumper){
                triggerRollerForward = false;
            }

// motor/servo control
            if (triggerRollerForward) {
                indexerPower = 1;
            }

            if (!triggerRollerForward) {
                indexerPower = 0;
            }

        }

        public void goToSpeedTriggerTarget(double indexerPower, double lifterPos) {
                indexerMotor.setPower(indexerPower);
                leftLifter.setPosition(lifterPos);
                rightLifter.setPosition(lifterPos);
            }
        }

    }

