package org.firstinspires.ftc.teamcode.subsystems;

import android.util.Log;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.opmodes.RobotBase;

public class Subsystem {

    // The original classes are placed as static nested classes to be accessible
    // while keeping the structure close to the original.

    public static class Drive {

        //Inherited data objects
        protected HardwareMap hardwareMap;
        public Telemetry telemetry;
        protected RobotBase robotBase;

        //Motor object definitions
        public DcMotor leftFrontMotor;
        public DcMotor rightFrontMotor;
        public DcMotor leftRearMotor;
        public DcMotor rightRearMotor;

        //Turtle Mode
        //public boolean turtleMode = false;
        public double  turtleFactor = 1;

        //teokstjeosrjoeirjoianowaieoiwajewa
        //    public double twist = 0;
        //    public double gobacktotheshadow;
        //    public boolean backlock;
        //    public double timelimit = 0;
        //    public boolean turnlock = false;
        //    public boolean helpwheredoigo;

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
            if (leftFrontMotor != null) leftFrontMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            if (leftRearMotor != null) leftRearMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            if (rightFrontMotor != null) rightFrontMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            if (rightRearMotor != null) rightRearMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        }

        public void driveFromGamepad(Gamepad gamepad) {

            double deadZone = 0.001;

            double forward  = 0;
            if (Math.abs(gamepad.left_stick_y) > deadZone) {
                forward = gamepad.left_stick_y * turtleFactor;
            }
            //        Log.v("Drive", "left_stick_y:"+gamepad.left_stick_y);
            //        Log.v("Drive", "forward:"+forward);

            double strafe = 0;
            if (Math.abs(gamepad.left_stick_x) > deadZone) {
                strafe = gamepad.left_stick_x * turtleFactor;
            }
            double twist = 0;
            if (Math.abs(gamepad.right_stick_x) > deadZone) {
                twist = -(gamepad.right_stick_x * 1);
            }
            //        Log.v("Drive", "left_stick_x:"+gamepad.left_stick_x);
            //        Log.v("Drive", "strafe:"+strafe);



            //easier for driver stuff
            //        if (gamepad.right_stick_button) {
            //            backlock = true;
            //            gobacktotheshadow = System.currentTimeMillis()+200;
            //            helpwheredoigo = true;
            //        }
            //        if (gamepad.left_stick_button) {
            //            backlock = true;
            //            gobacktotheshadow = System.currentTimeMillis()+200;
            //            helpwheredoigo = false;
            //        }
            //        if (backlock == true) {
            //            if (System.currentTimeMillis() < gobacktotheshadow) {
            //                forward = 0.75; //backing up
            //            }
            //            else {
            //                backlock = false;
            //                turnlock = true;
            //                timelimit = System.currentTimeMillis()+400; //turn amount
            //            }
            //        }
            //        if (turnlock == true) {
            //            if (System.currentTimeMillis() < timelimit) {
            //                if (helpwheredoigo == true) {
            //                    twist = -0.75;
            //                }
            //                if (helpwheredoigo == false) {
            //                    twist = 0.75;
            //                }
            //            }
            //            else {
            //                turnlock = false;
            //            }
            //        }
            //        if (backlock == false) {
            //            if (turnlock == false) {
            //                twist = 0;
            //                if (Math.abs(gamepad.right_stick_x) > deadZone) {
            //                    twist = -((gamepad.right_stick_x * 0.75) * turtleFactor);
            //                }
            //            }
            //        }



            //        Log.v("Drive", "right_stick_x:"+gamepad.right_stick_x);
            //        Log.v("Drive", "twist:"+twist);

            // You may need to multiply some of these by -1 to invert direction of
            // the motor.  This is not an issue with the calculations themselves.
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
            if (leftFrontMotor != null) leftFrontMotor.setPower(speeds[0]);
            if (rightFrontMotor != null) rightFrontMotor.setPower(speeds[1]);
            if (leftRearMotor != null) leftRearMotor.setPower(speeds[2]);
            if (rightRearMotor != null) rightRearMotor.setPower(speeds[3]);
        }

        //    public void turtleToggle() {
        //        if (turtleMode) {
        //            turtleMode = false;
        //            turtleFactor = 1;
        //        } else {
        //            turtleMode = true;
        //            turtleFactor = 0.5;
        //        }
        //    }

        public boolean isBusy() {
            return true;
        }

        public void stop() {
            if (leftFrontMotor != null) leftFrontMotor.setPower(0);
            if (leftRearMotor != null) leftRearMotor.setPower(0);
            if (rightFrontMotor != null) rightFrontMotor.setPower(0);
            if (rightRearMotor != null) rightRearMotor.setPower(0);
        }

        public void reset() {
            initHardware();
        }
    }

    public static class Indexer {
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
            initHardware(); // Added initHardware call for this constructor
        }

        protected void initHardware() {
            indexerSystem = new IndexerSystem(hardwareMap); // Passed hardwareMap to inner class
        }

        // FIX: Added 'static' keyword here
        public static class IndexerSystem {
            public DcMotor indexerMotor;
            public Servo rightLifter;
            public Servo leftLifter;
            public ColorSensor rightColorSensor;
            public ColorSensor leftColorSensor;

            private final ElapsedTime sequenceTimer = new ElapsedTime();

            // Need to pass HardwareMap to init hardware in the inner class
            public IndexerSystem(HardwareMap hardwareMap) {
                initHardware(hardwareMap);
            }

            public double indexerPower = 0;
            public boolean triggerRollerForward = false;

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



            protected void initHardware(HardwareMap hardwareMap) {
                try {
                    indexerMotor = hardwareMap.get(DcMotor.class, "TriggerRoller");
                    indexerMotor.setDirection(DcMotor.Direction.FORWARD);
                    indexerMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                    indexerMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                } catch (Exception e) {}

                try {
                    leftColorSensor = hardwareMap.get(ColorSensor.class, "leftColor");
                } catch (Exception e) {}

                try {
                    rightColorSensor = hardwareMap.get(ColorSensor.class, "rightColor");
                } catch (Exception e) {}

                try {
                    leftLifter = hardwareMap.get(Servo.class, "leftLifter");
                } catch (Exception e) {}

                try {
                    rightLifter = hardwareMap.get(Servo.class, "rightLifter");
                    rightLifter.setDirection(Servo.Direction.REVERSE);
                } catch (Exception e) {}
            }

            public void colorSensorStuff(){
                // implementation not provided in original
            }
            public void doIndexerStuff(Gamepad gamepad2) {
                goToTarget(indexerPower);

                //select left chamber
                if (gamepad2.x){
                    leftChamberReady = true;
                    intakeChamberReady = false;
                    rightChamberReady = false;
                    gamepadSelection = 1;
                }

                //select right chamber
                if (gamepad2.b){
                    leftChamberReady = false;
                    intakeChamberReady = false;
                    rightChamberReady = true;
                    gamepadSelection = 2;
                }

                //select intake chamber
                if (gamepad2.y){
                    leftChamberReady = false;
                    intakeChamberReady = true;
                    rightChamberReady = false;
                    gamepadSelection = 3;
                }

                if (gamepad2.x){
                    shooterSelection = 0;
                    gamepadSelection = 0;
                }

                //confirm selection
                if (gamepad2.right_bumper /*&& apriltagLock*/ || gamepad2.right_bumper && gamepad2.y){

                    //confirming left chamber
                    if (leftChamberReady){
                        leftChamberSequence = true;
                        rightChamberSequence = false;
                        intakeChamberSequence = false;
                    }

                    //confirming right chamber
                    if (rightChamberReady){
                        leftChamberSequence = false;
                        rightChamberSequence = true;
                        intakeChamberSequence = false;
                    }

                    //confirming intake chamber
                    if (intakeChamberReady){
                        leftChamberSequence = false;
                        rightChamberSequence = false;
                        intakeChamberSequence = true;
                    }


                }

                if (leftChamberSequence){
                    shooterSelection = 1;
                    leftChamberSequence = false;
                }

                if (rightChamberSequence){
                    shooterSelection = 2;
                    rightChamberSequence = false;
                }

                if (intakeChamberSequence){
                    shooterSelection = 3;

                    intakeChamberSequence = false;
                }

                if (gamepad2.right_trigger > 0.25){
                    triggerRollerForward = true;
                }
                if (gamepad2.right_trigger < 0.25){
                    triggerRollerForward = false;
                }

                // motor/servo control
                if (triggerRollerForward) {
                    indexerPower = 1;
                }

                else {
                    indexerPower = 0;
                }

            }

            public void goToTarget(double indexerPower) {
                if (indexerMotor != null) indexerMotor.setPower(indexerPower);
            }
        }


    }

    public static class Shooter {
        protected HardwareMap hardwareMap;
        protected Telemetry telemetry;
        protected RobotBase robotBase;
        public ShooterMotor shooterMotor;

        public Shooter(HardwareMap hardwareMap, RobotBase opMode) {
            this.hardwareMap = hardwareMap;
            this.robotBase = opMode;
            this.telemetry = robotBase.telemetry;

            initHardware();
        }

        public Shooter(HardwareMap hardwareMap) {
            this.hardwareMap = hardwareMap;
            initHardware(); // Added initHardware call for this constructor
        }

        protected void initHardware() {
            shooterMotor  = new ShooterMotor(hardwareMap); // Passed hardwareMap to inner class
        }

        // FIX: Added 'static' keyword here
        public static class ShooterMotor {

            private CRServo helperWheel;
            public DcMotor shooterMotor;
            public Servo shooterHood;

            public ShooterMotor(HardwareMap hardwareMap) {
                initHardware(hardwareMap);
            }

            public double targetSpeed = 0;
            public double servoTargetSpeed = 0;
            public boolean shooterForward = false;
            public double hoodAngle = 1;



            protected void initHardware(HardwareMap hardwareMap) {
                try {
                    shooterMotor = hardwareMap.get(DcMotor.class, "shooter");
                    shooterMotor.setDirection(DcMotor.Direction.REVERSE);
                    shooterMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                } catch (Exception e) {}

                try {
                    helperWheel = hardwareMap.get(CRServo.class, "helperWheel");
                    helperWheel.setDirection(CRServo.Direction.REVERSE);
                } catch (Exception e) {}

                try {
                    shooterHood = hardwareMap.get(Servo.class, "shooterHood");
                    shooterHood.setPosition(hoodAngle);
                } catch (Exception e) {}
            }

            public void doShooterStuff(Gamepad gamepad2) {
                goToTargetSpeed(targetSpeed);

                if (gamepad2.right_trigger > 0.25 || shooterForward) {
                    targetSpeed = 0.95;
                    servoTargetSpeed = 1;
                } else {
                    targetSpeed = 0;
                    servoTargetSpeed = 0;
                }

                if (gamepad2.dpad_up) {
                    hoodAngle = hoodAngle - 0.0001;
                }

                if (gamepad2.dpad_down) {
                    hoodAngle = hoodAngle + 0.0001;
                }

                if (gamepad2.a){
                    hoodAngle = 1;
                }

                if (gamepad2.b){
                    hoodAngle = 0.7327;
                }


            }

            public void goToTargetSpeed(double targetSpeed) {
                if (shooterMotor != null) shooterMotor.setPower(targetSpeed);
                if (helperWheel != null) helperWheel.setPower(servoTargetSpeed);
                if (shooterHood != null) shooterHood.setPosition(hoodAngle);
            }
        }
    }

    public static class Turret {
        protected HardwareMap hardwareMap;
        protected Telemetry telemetry;
        protected RobotBase robotBase;
        public TurretMotor turretMotor;
        // Limelight is initialized in TurretMotor class, not here
        // public Limelight3A limelight;

        public Turret(HardwareMap hardwareMap, RobotBase opMode) {
            this.hardwareMap = hardwareMap;
            this.robotBase = opMode;
            this.telemetry = robotBase.telemetry;

            initHardware();
        }

        public Turret(HardwareMap hardwareMap) {
            this.hardwareMap = hardwareMap;
            initHardware(); // Added initHardware call for this constructor
        }

        protected void initHardware() {
            turretMotor = new TurretMotor(hardwareMap); // Passed hardwareMap to inner class
        }

        // FIX: Added 'static' keyword here
        public static class TurretMotor {


            public DcMotor turretMotor;
            public Limelight3A limelight; // Limelight instance moved here

            public TurretMotor(HardwareMap hardwareMap) {
                initHardware(hardwareMap);
            }

            public double targetSpeedl = 0;
            public double targetSpeed = 0;
            public double servoTargetSpeed = 0;
            public boolean shooterForward = false;
            public double hoodAngle = 1;


            protected void initHardware(HardwareMap hardwareMap) {
                try {
                    turretMotor = hardwareMap.get(DcMotor.class, "Turret");
                    turretMotor.setDirection(DcMotor.Direction.FORWARD);
                    turretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                } catch (Exception e) {}

                try {
                    limelight = hardwareMap.get(Limelight3A.class, "limelight");
                    limelight.setPollRateHz(100); // This sets how often we ask Limelight for data (100 times per second)
                    limelight.start(); // This tells Limelight to start looking!
                    limelight.pipelineSwitch(0);
                } catch (Exception e) {}
            }

            public void doTurretStuff(Gamepad gamepad2) {

                double tx = 0;
                if (limelight != null && limelight.getLatestResult() != null) {
                    tx = limelight.getLatestResult().getTx();
                }

                targetSpeed = ((tx / 27.25)*1);

                if (gamepad2.right_trigger > 0.25) {
                    if (targetSpeed < 0.125) {
                        targetSpeed = targetSpeed - 0.025;
                    }
                    if (targetSpeed > 0.125) {
                        targetSpeed = targetSpeed + 0.025;
                    }
                }
                else {
                    targetSpeed = 0;
                }


                goToTargetSpeed(targetSpeed);
            }
            public void goToTargetSpeed ( double targetSpeed) {
                if (turretMotor != null) turretMotor.setPower(targetSpeed);
            }


        }
    }

    public static class Intake {
        protected HardwareMap hardwareMap;
        protected Telemetry telemetry;
        protected RobotBase robotBase;
        public IntakeRoller intakeRoller;

        public Intake(HardwareMap hardwareMap, RobotBase opMode) {
            this.hardwareMap = hardwareMap;
            this.robotBase = opMode;
            this.telemetry = robotBase.telemetry;

            initHardware();
        }

        public Intake(HardwareMap hardwareMap) {
            this.hardwareMap = hardwareMap;
            initHardware(); // Added initHardware call for this constructor
        }

        protected void initHardware() {
            intakeRoller = new IntakeRoller(hardwareMap); // Passed hardwareMap to inner class
        }

        // FIX: Added 'static' keyword here
        public static class IntakeRoller {

            public DcMotor intakeMotor;;

            public IntakeRoller(HardwareMap hardwareMap) {
                initHardware(hardwareMap);
            }

            public boolean intakeForward = false;
            public boolean intakeBackwards = false;
            public double intakePower = 0;


            protected void initHardware(HardwareMap hardwareMap) {
                try {
                    intakeMotor = hardwareMap.get(DcMotor.class, "IntakeRollers");
                    intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                    intakeMotor.setDirection(DcMotor.Direction.FORWARD);
                } catch (Exception e) {}
            }

            public void doIntakeStuff(Gamepad gamepad2) {
                goToTarget(intakePower);

                //            if (intakeForward || gamepad2.right_bumper){
                //                intakePower = -1;
                //            }

                if (intakeBackwards || gamepad2.left_bumper){
                    intakePower = 1;
                }

                else {
                    intakePower = 0;
                }

            }

            public void goToTarget(double intakePower) {
                if (intakeMotor != null) intakeMotor.setPower(intakePower);
            }
        }
    }
}