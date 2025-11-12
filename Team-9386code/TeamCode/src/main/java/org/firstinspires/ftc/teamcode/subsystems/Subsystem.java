package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.Locale;

/**
 * This class initiates all the specific hardware for a robot.
 * All subsystem logic is nested inside this OpMode-extending class.
 * This class serves as the base for all OpModes.
 */
@Disabled
public class Subsystem extends OpMode
{
    // =========================================================================
    // MAIN ROBOT BASE LOGIC (Formerly RobotBase.java)
    // =========================================================================

    // Define constants that should NOT be adjusted by manual calibration
    protected boolean           INITIALIZE_IMU      = true;
    protected boolean           INITIALIZE_DRIVE    = true;

    // Make subsystems available to all class extensions
    public Drive drive;
    public Shooter shooter;
    public Indexer indexer;
    public Intake intake;
    public Turret turret;
    public Auto_paths auto_Paths;

    /* Constructor */
    public Subsystem(){ }

    /* Initialize standard Hardware interfaces */
    @Override
    public void init() {

        //Initialize subsystems - passing 'this' (the OpMode context)
        drive = new Drive(hardwareMap, this);
        shooter = new Shooter(hardwareMap, this);
        intake = new Intake(hardwareMap, this);
        indexer = new Indexer(hardwareMap, this);
        turret = new Turret(hardwareMap, this);
        auto_Paths = new Auto_paths(hardwareMap, this);
    }

    @Override
    public void init_loop() {
    }

    @Override
    public void start() {
        telemetry.clearAll();
    }

    public void loop() {
        //Do nothing, use these classes in the opModes
    }

    /*
     * Code to run when the op mode is first disabled goes here
     */
    @Override
    public void stop() {

    }

    String formatDegrees(double degrees){
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }

    // =========================================================================
    // DRIVE SUBSYSTEM (Formerly Drive.java)
    // =========================================================================

    public class Drive {

        //Inherited data objects
        protected HardwareMap hardwareMap;
        public Telemetry telemetry;
        // Changed RobotBase to OpMode to match the new parent class
        protected OpMode robotBase;

        //Motor object definitions
        public DcMotor leftFrontMotor;
        public DcMotor rightFrontMotor;
        public DcMotor leftRearMotor;
        public DcMotor rightRearMotor;

        //Turtle Mode
        public double  turtleFactor = 1;

        //mods
        private final ElapsedTime time = new ElapsedTime();

        public Drive(HardwareMap hardwareMap, OpMode opMode) {
            this.hardwareMap = hardwareMap;
            this.robotBase = opMode;
            this.telemetry = opMode.telemetry;
            initHardware();
        }

        protected void initHardware() {
            try {
                leftFrontMotor = hardwareMap.get(DcMotorEx.class, "FL");
                leftFrontMotor.setDirection(DcMotor.Direction.REVERSE);
            } catch (Exception e) {
                // Log.v("Drive", ":leftFrontMotor init failed");
            }

            try {
                leftRearMotor = hardwareMap.get(DcMotorEx.class, "BL");
                leftRearMotor.setDirection(DcMotor.Direction.REVERSE);
            } catch (Exception e){
                // Log.v("Drive", ":leftRearMotor init failed");
            }

            try {
                rightFrontMotor = hardwareMap.get(DcMotorEx.class, "FR");
            } catch (Exception e){
                // Log.v("Drive", ":rightFrontMotor init failed");
            }

            try {
                rightRearMotor = hardwareMap.get(DcMotorEx.class, "BR");
            } catch (Exception e){
                // Log.v("Drive", ":rightRearMotor init failed");
            }
            leftFrontMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            leftRearMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            rightFrontMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            rightRearMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        }

        public void driveFromGamepad(Gamepad gamepad) {

            double deadZone = 0.001;

            double forward  = 0;
            if (Math.abs(gamepad.left_stick_y) > deadZone) {
                forward = -gamepad.left_stick_y;
            }

            double strafe = 0;
            if (Math.abs(gamepad.left_stick_x) > deadZone) {
                strafe = -gamepad.left_stick_x;
            }
            double twist = 0;
            if (Math.abs(gamepad.right_stick_x) > deadZone) {
                twist = gamepad.right_stick_x;
            }

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

// ---

    // =========================================================================
    // INTAKE SUBSYSTEM (Formerly Intake.java)
    // =========================================================================

    public class Intake {
        protected HardwareMap hardwareMap;
        protected Telemetry telemetry;
        // Changed RobotBase to OpMode to match the new parent class
        protected OpMode robotBase;
        public IntakeRoller intakeRoller;

        public Intake(HardwareMap hardwareMap, OpMode opMode) {
            this.hardwareMap = hardwareMap;
            this.robotBase = opMode;
            this.telemetry = opMode.telemetry;
            initHardware();
        }

        public Intake(HardwareMap hardwareMap) {
            this.hardwareMap = hardwareMap;
        }

        protected void initHardware() {
            intakeRoller = new IntakeRoller();
        }

        public class IntakeRoller {

            private final ElapsedTime sequenceTimer = new ElapsedTime();

            public void resetSequenceTimer(){
                sequenceTimer.reset();
            }

            public DcMotor intakeMotor;

            public IntakeRoller() { //HardwareMap hardwareMap, RobotBase opMode
                initHardware();
            }

            public boolean intakeForward = false;
            public boolean intakeBackwards = false;
            public double intakePower = 0;

            protected void initHardware() {
                intakeMotor = hardwareMap.get(DcMotor.class, "IntakeRollers");
                intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                intakeMotor.setDirection(DcMotor.Direction.FORWARD);
            }

            public void doIntakeStuff(Gamepad gamepad2) {
                goToTarget(intakePower);

                if (gamepad2.right_bumper){
                    intakePower = -1;
                }

                else if (gamepad2.left_bumper){
                    intakePower = 1;
                }

                else if (!gamepad2.left_bumper && !gamepad2.right_bumper){
                    intakePower = 0;
                }
            }

            public void doIntakeStuffAuto (){
                if (sequenceTimer.seconds() > 12 && sequenceTimer.seconds() < 17){
                    intakePower = -1;
                }
                else {
                    intakePower = 0;
                }
                goToTarget(intakePower);
            }

            public void goToTarget(double intakePower) {
                intakeMotor.setPower(intakePower);
            }
        }
    }

// ---

    // =========================================================================
    // INDEXER SUBSYSTEM (Formerly Indexer.java)
    // =========================================================================

    public class Indexer {
        protected HardwareMap hardwareMap;
        protected Telemetry telemetry;
        // Changed RobotBase to OpMode to match the new parent class
        protected OpMode robotBase;
        public IndexerSystem indexerSystem;

        public Indexer(HardwareMap hardwareMap, OpMode opMode) {
            this.hardwareMap = hardwareMap;
            this.robotBase = opMode;
            this.telemetry = opMode.telemetry;
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
                    lifterPos = 0;
                }

                if (!gamepad2.a && gamepad1.left_trigger < 0.25){
                    lifterPos = 0.1;
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

// ---

    // =========================================================================
    // SHOOTER SUBSYSTEM (Formerly Shooter.java)
    // =========================================================================

    public class Shooter {
        protected HardwareMap hardwareMap;
        protected Telemetry telemetry;
        // Changed RobotBase to OpMode to match the new parent class
        protected OpMode robotBase;
        public ShooterMotor shooterMotor;

        public Shooter(HardwareMap hardwareMap, OpMode opMode) {
            this.hardwareMap = hardwareMap;
            this.robotBase = opMode;
            this.telemetry = opMode.telemetry;
            initHardware();
        }

        public Shooter(HardwareMap hardwareMap) {
            this.hardwareMap = hardwareMap;
        }

        protected void initHardware() {
            shooterMotor  = new ShooterMotor();
        }

        public class ShooterMotor {

            private CRServo helperWheel;
            public DcMotor shooterMotor;
            public Servo shooterHood;

            private final ElapsedTime sequenceTimer = new ElapsedTime();

            public void resetSequenceTimer(){
                sequenceTimer.reset();
            }

            public ShooterMotor() { //HardwareMap hardwareMap, RobotBase opMode
                initHardware();
            }

            public double targetSpeed = 0;
            public double servoTargetSpeed = 0;
            public boolean shooterForward = false;
            public boolean shooterForwardAuto1 = false;
            public double hoodAngle = 1;

            protected void initHardware() {
                shooterMotor = hardwareMap.get(DcMotor.class, "shooter");
                shooterMotor.setDirection(DcMotor.Direction.REVERSE);
                shooterMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                helperWheel = hardwareMap.get(CRServo.class, "helperWheel");
                helperWheel.setDirection(CRServo.Direction.REVERSE);
                shooterHood = hardwareMap.get(Servo.class, "shooterHood");
                shooterHood.setPosition(hoodAngle);
            }

            public void doShooterStuff(Gamepad gamepad2 , Gamepad gamepad1) {

                if (gamepad2.right_trigger > 0.25 || shooterForward || gamepad1.right_trigger > 0.25) {
                    targetSpeed = 0.85;
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

                if (gamepad2.x){
                    hoodAngle = 1;
                }

                if (gamepad2.b){
                    hoodAngle = 0.7327;
                }

                if (gamepad2.y){
                    hoodAngle = 0.5462;
                }
                goToTargetSpeed(targetSpeed);
            }

            public void autoTestRev (){

                if (sequenceTimer.seconds() > 15 ){
                    targetSpeed = 0;
                    hoodAngle = 1;
                    servoTargetSpeed = 0;
                }

                else {
                    targetSpeed = 0.8;
                    hoodAngle = 0.5462;
                    servoTargetSpeed = 1;
                }
                goToTargetSpeed(targetSpeed);
            }

            public void autoTestRev2 (){

                if (sequenceTimer.seconds() > 18 ){
                    targetSpeed = 0;
                    hoodAngle = 1;
                    servoTargetSpeed = 0;
                }

                else {
                    targetSpeed = 0.825;
                    hoodAngle = 0.7327;
                    servoTargetSpeed = 1;
                }
                goToTargetSpeed(targetSpeed);
            }

            public void goToTargetSpeed(double targetSpeed) {
                shooterMotor.setPower(targetSpeed);
                helperWheel.setPower(servoTargetSpeed);
                shooterHood.setPosition(hoodAngle);
            }
        }
    }

// ---

    // =========================================================================
    // TURRET SUBSYSTEM (Formerly Turret.java)
    // =========================================================================

    public class Turret {
        protected HardwareMap hardwareMap;
        protected Telemetry telemetry;
        // Changed RobotBase to OpMode to match the new parent class
        protected OpMode robotBase;
        public TurretMotor turretMotor;
        public Limelight3A limelight;

        public Turret(HardwareMap hardwareMap, OpMode opMode) {
            this.hardwareMap = hardwareMap;
            this.robotBase = opMode;
            this.telemetry = opMode.telemetry;
            initHardware();
        }

        public Turret(HardwareMap hardwareMap) {
            this.hardwareMap = hardwareMap;
        }

        protected void initHardware() {
            turretMotor = new TurretMotor();
        }

        public class TurretMotor {


            public DcMotor turretMotor;

            public TurretMotor() { //HardwareMap hardwareMap, RobotBase opMode
                initHardware();
            }

            public double targetSpeedb = 0;
            public double targetSpeedr = 0;
            public double targetSpeedRA = 0;
            public double targetSpeedBA = 0;
            public double targetSpeed = 0;
            public double servoTargetSpeed = 0;
            public boolean shooterForward = false;
            public double hoodAngle = 1;
            int id;


            protected void initHardware() {
                turretMotor = hardwareMap.get(DcMotor.class, "Turret");
                turretMotor.setDirection(DcMotor.Direction.FORWARD);
                turretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

                limelight = hardwareMap.get(Limelight3A.class, "limelight");
                limelight.setPollRateHz(100); // This sets how often we ask Limelight for data (100 times per second)
                limelight.start(); // This tells Limelight to start looking!
            }

            public void doTurretRStuff(Gamepad gamepad2 , Gamepad gamepad1) {
                limelight.pipelineSwitch(0);
                // Removed redundant Limelight initialization

                targetSpeedr = ((limelight.getLatestResult().getTx() / 27.25)*1);
                if (gamepad2.right_trigger > 0.25 || gamepad1.right_trigger > 0.25 || gamepad1.left_stick_button) {
                    if (targetSpeedr < 0.125) {
                        targetSpeedr = targetSpeedr - 0.025;
                    }
                    if (targetSpeedr > 0.125) {
                        targetSpeedr = targetSpeedr + 0.025;
                    }
                }
                else {
                    targetSpeedr = 0;
                }


                goToTargetSpeed(targetSpeedr);
            }
            public void goToTargetSpeed ( double targetSpeed) {
                turretMotor.setPower(targetSpeedr);
            }


            public void doTurretBStuff(Gamepad gamepad2, Gamepad gamepad1) {
                limelight.pipelineSwitch(1);
                // Removed redundant Limelight initialization

                targetSpeedb = ((limelight.getLatestResult().getTx() / 27.25)*1);
                if (gamepad2.right_trigger > 0.25 || gamepad1.right_trigger > 0.25 || gamepad1.left_stick_button) {
                    if (targetSpeedb < 0.125) {
                        targetSpeedb = targetSpeedb - 0.025;
                    }
                    if (targetSpeedb > 0.125) {
                        targetSpeedb = targetSpeedb + 0.025;
                    }
                }
                else {
                    targetSpeedb = 0;
                }


                goToTargetSpeed2(targetSpeedb);
            }
            public void goToTargetSpeed2 ( double targetSpeed) {
                turretMotor.setPower(targetSpeedb);
            }

            public void TargetSpeedRA() {
                limelight.pipelineSwitch(0);
                // Removed redundant Limelight initialization

                targetSpeedRA = ((limelight.getLatestResult().getTx() / 27.25)*1);

                if (targetSpeedRA < 0.125) {
                    targetSpeedRA = targetSpeedRA - 0.025;
                }
                if (targetSpeedRA > 0.125) {
                    targetSpeedRA = targetSpeedRA + 0.025;
                }


                goToTargetSpeedRA(targetSpeedRA);
            }
            public void goToTargetSpeedRA ( double targetSpeedRA) {
                turretMotor.setPower(targetSpeedRA);
            }

            public void TargetSpeedBA() {
                limelight.pipelineSwitch(1);
                // Removed redundant Limelight initialization


                targetSpeedBA = ((limelight.getLatestResult().getTx() / 27.25)*1);

                if (targetSpeedBA < 0.125) {
                    targetSpeedBA = targetSpeedBA - 0.025;
                }
                if (targetSpeedBA > 0.125) {
                    targetSpeedBA = targetSpeedBA + 0.025;
                }


                goToTargetSpeedBA(targetSpeedBA);
            }
            public void goToTargetSpeedBA ( double targetSpeedBA) {
                turretMotor.setPower(targetSpeedBA);
            }


        }
    }

// ---

    // =========================================================================
    // AUTO PATHS SUBSYSTEM (Formerly Auto_paths.java)
    // =========================================================================

    public class Auto_paths {

        //Inherited data objects
        protected HardwareMap hardwareMap;
        public Telemetry telemetry;
        // Changed RobotBase to OpMode to match the new parent class
        protected OpMode robotBase;

        //Motor object definitions
        SparkFunOTOS myOtos;
        SparkFunOTOS myOtos2;
        public DcMotor leftFrontMotor;
        public DcMotor rightFrontMotor;
        public DcMotor leftRearMotor;
        public DcMotor rightRearMotor;


        //mods
        private final ElapsedTime time = new ElapsedTime();

        public Auto_paths(HardwareMap hardwareMap, OpMode opMode) {
            this.hardwareMap = hardwareMap;
            this.robotBase = opMode;
            this.telemetry = opMode.telemetry;
            initHardware();
        }

        public void initHardware() {


            myOtos = hardwareMap.get(SparkFunOTOS.class, "LeftOtos");
            myOtos2 = hardwareMap.get(SparkFunOTOS.class, "RightOtos");

            try {
                leftFrontMotor = hardwareMap.get(DcMotorEx.class, "FL");
                leftFrontMotor.setDirection(DcMotor.Direction.FORWARD);
            } catch (Exception e) {
                // Log.v("Drive", ":leftFrontMotor init failed");
            }

            try {
                leftRearMotor = hardwareMap.get(DcMotorEx.class, "BL");
                leftRearMotor.setDirection(DcMotor.Direction.FORWARD);
            } catch (Exception e){
                // Log.v("Drive", ":leftRearMotor init failed");
            }

            try {
                rightFrontMotor = hardwareMap.get(DcMotorEx.class, "FR");
                rightFrontMotor.setDirection(DcMotor.Direction.REVERSE);
            } catch (Exception e){
                // Log.v("Drive", ":rightFrontMotor init failed");
            }

            try {
                rightRearMotor = hardwareMap.get(DcMotorEx.class, "BR");
                rightRearMotor.setDirection(DcMotor.Direction.REVERSE);
            } catch (Exception e){
                // Log.v("Drive", ":rightRearMotor init failed");
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

            double forwardSpeed = forward;

            double strafeSpeed = strafe;

            double twistSpeed = twist;

            leftFrontMotor.setPower(forwardSpeed + strafeSpeed + twistSpeed);
            rightFrontMotor.setPower(forwardSpeed - strafeSpeed - twistSpeed);
            leftRearMotor.setPower((forwardSpeed - strafeSpeed + twistSpeed));
            rightRearMotor.setPower((forwardSpeed + strafeSpeed - twistSpeed));
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
}