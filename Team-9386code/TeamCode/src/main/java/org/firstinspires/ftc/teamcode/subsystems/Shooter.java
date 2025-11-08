package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.Indexer;
import org.firstinspires.ftc.teamcode.opmodes.*;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.opmodes.RobotBase;

public class Shooter {
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

