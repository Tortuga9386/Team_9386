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


        public final ElapsedTime indexerTimer = new ElapsedTime();

        public IndexerSystem() { //HardwareMap hardwareMap, RobotBase opMode
            initHardware();
        }

        public double indexerPower = 0;
        public boolean triggerRollerForward = false;

        public double leftLifterHeight = 0.875;
        public double rightLifterHeight = 0.875;
        public double intakePower;

        // Shooter selector
        public boolean leftChamberReady = false;
        public boolean rightChamberReady = false;
        public boolean intakeChamberReady = false;

        //Sequences
        public boolean rightChamberSequence = false;
        public boolean leftChamberSequence = false;
        public boolean intakeChamberSequence = false;

        public double shooterSelection = 0;
        public double gamepadSelection = 0;

        public boolean leftLifterUp = false;
        public boolean rightLifterUp = false;



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

            //select left chamber
            if (gamepad2.x) {
                leftChamberReady = true;
                intakeChamberReady = false;
                rightChamberReady = false;
                gamepadSelection = 1;
            }

            //select right chamber
            if (gamepad2.b) {
                leftChamberReady = false;
                intakeChamberReady = false;
                rightChamberReady = true;
                gamepadSelection = 2;
            }

            //select intake chamber
            if (gamepad2.y) {
                leftChamberReady = false;
                intakeChamberReady = true;
                rightChamberReady = false;
                gamepadSelection = 3;
            }

//            if (gamepad2.x){
//                shooterSelection = 0;
//                gamepadSelection = 0;
//            }

            //confirm selection
            if (gamepad2.right_bumper /*&& apriltagLock*/ || gamepad2.right_bumper && gamepad2.y) {

                //confirming left chamber
                if (leftChamberReady) {
                    leftChamberSequence = true;
                    rightChamberSequence = false;
                    intakeChamberSequence = false;
                }

                //confirming right chamber
                if (rightChamberReady) {
                    leftChamberSequence = false;
                    rightChamberSequence = true;
                    intakeChamberSequence = false;
                }

                //confirming intake chamber
                if (intakeChamberReady) {
                    leftChamberSequence = false;
                    rightChamberSequence = false;
                    intakeChamberSequence = true;
                }


            }

            if (leftChamberSequence) {
                shooterSelection = 1;
                leftChamberSequence = false;
            }

            if (rightChamberSequence) {
                shooterSelection = 2;
                rightChamberSequence = false;
            }

            if (intakeChamberSequence) {
                shooterSelection = 3;
                intakeChamberSequence = false;
            }

            if (gamepad2.right_trigger > 0.25 || gamepad2.right_bumper) {
                triggerRollerForward = true;
            }
            if (gamepad2.right_trigger < 0.25 && !gamepad2.right_bumper) {
                triggerRollerForward = false;
            }
            if (gamepad2.right_bumper) {
                intakePower = -1;
            }

            if (gamepad2.left_bumper) {
                intakePower = 1;
            }

            if (!gamepad2.right_bumper && !gamepad2.left_bumper) {
                intakePower = 0;
            }
            if (gamepad2.a){
                leftLifterHeight = 0.62;
            }
            if (gamepad2.x){
                rightLifterHeight = 0.65;
            }
            if (!gamepad2.a && !gamepad2.x){
                leftLifterHeight = 0.875;
                rightLifterHeight = 0.875;
            }
            if (triggerRollerForward) {
                indexerPower = 1;
            }


            if (!triggerRollerForward) {
                indexerPower = 0;
            }

            goToTarget(indexerPower);
        }
//Auto control

        public void intakeAutoLONG() {

            if (indexerTimer.seconds() > 9.5) {
                intakePower = -1;
            }
            if (indexerTimer.seconds() < 9.5 || indexerTimer.seconds() > 15) {
                intakePower = 0;
            }
            if (indexerTimer.seconds() < 20) {
                indexerPower = 1;
            }
            if (indexerTimer.seconds() > 20){
                indexerPower = 0;
            }
            if (indexerTimer.seconds() > 5 || indexerTimer.seconds() > 11) {
                leftLifterUp = true;
            }
            if (indexerTimer.seconds() > 6.25 || indexerTimer.seconds() > 11) {
                rightLifterUp = true;
            }
            if (indexerTimer.seconds() < 5 || indexerTimer.seconds() > 8 && indexerTimer.seconds() < 11) {
                leftLifterUp = false;
            }

            if (indexerTimer.seconds() < 6.25 || indexerTimer.seconds() > 9.25 && indexerTimer.seconds() < 11) {
                rightLifterUp = false;
            }





// motor/servo control
            if (leftLifterUp){
                leftLifterHeight = 0.62;
            }

            if (rightLifterUp){
                rightLifterHeight = 0.65;
            }

            if (!leftLifterUp){
                leftLifterHeight = 0.875;
            }

            if (!rightLifterUp) {
                rightLifterHeight = 0.875;
            }

            goToTarget(indexerPower);
        }

        public void goToTarget(double indexerPower) {
                indexerMotor.setPower(indexerPower);
                leftLifter.setPosition(leftLifterHeight);
                rightLifter.setPosition(rightLifterHeight);
                robotBase.intake.intakeRoller.intakeMotor.setPower(intakePower);
            }
        }

    }

