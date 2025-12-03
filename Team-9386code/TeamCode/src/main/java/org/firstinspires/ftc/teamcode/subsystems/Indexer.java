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
import org.firstinspires.ftc.teamcode.opmodes.*;

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

        public void resetSequenceTimer(){
            sequenceTimer.reset();
        }

        public IndexerSystem() { //HardwareMap hardwareMap, RobotBase opMode
            initHardware();
        }

        public double indexerPower = 0;
        public boolean triggerRollerForward = false;

        public double shooterSelection = 0;
        public double gamepadSelection = 0;
        public double lifterPos = 0.1;


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
        public void doIndexerStuff(Gamepad gamepad2 , Gamepad gamepad1) {
            goToSpeedTriggerTarget(indexerPower, lifterPos);


            //Lifters

            if (gamepad2.a || gamepad1.left_trigger > 0.25){
                lifterPos = 0.65;
            }

            if (!gamepad2.a && gamepad1.left_trigger < 0.25){
                lifterPos = 0.875;
            }

            if (gamepad2.right_trigger > 0.25 || gamepad2.right_bumper || gamepad1.right_trigger > 0.25){
                triggerRollerForward = true;
            }

            if (gamepad2.right_trigger < 0.25 && !gamepad2.right_bumper && gamepad1.right_trigger < 0.25){
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

        public void doIndexerStuffAuto(){

            if (sequenceTimer.seconds() < 17){
                indexerPower = 1;
            }

            if ((sequenceTimer.seconds() > 6 && sequenceTimer.seconds() < 8.5) || (sequenceTimer.seconds() > 13 && sequenceTimer.seconds() < 17)  ){
                lifterPos = 0;
            }
            else if ((sequenceTimer.seconds() < 6 || sequenceTimer.seconds() > 8.5) || (sequenceTimer.seconds() < 13 || sequenceTimer.seconds() > 17)){
                lifterPos = 0.115;
            }
            else if (sequenceTimer.seconds() > 17){
                indexerPower = 0;
            }
            goToSpeedTriggerTarget(indexerPower,lifterPos);
        }

        public void goToSpeedTriggerTarget(double indexerPower, double lifterPos) {
                indexerMotor.setPower(indexerPower);
                leftLifter.setPosition(lifterPos);
                rightLifter.setPosition(lifterPos);
            }
        }

    }

