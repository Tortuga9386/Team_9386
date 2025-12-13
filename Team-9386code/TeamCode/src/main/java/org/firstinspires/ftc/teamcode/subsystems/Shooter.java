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

        public ShooterMotor() { //HardwareMap hardwareMap, RobotBase opMode
            initHardware();
        }

        protected void initHardware() {
            shooterMotor = hardwareMap.get(DcMotor.class, "shooter");
            shooterMotor.setDirection(DcMotor.Direction.REVERSE);
            shooterMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            helperWheel = hardwareMap.get(CRServo.class, "helperWheel");
            helperWheel.setDirection(CRServo.Direction.REVERSE);

            shooterHood = hardwareMap.get(Servo.class, "shooterHood");
            shooterHood.setPosition(1);

            goToTarget(0,0,1);
            }



        public void goToTarget(double shooterPower, double sushiRollerSpeed, double hoodAngle) {
            shooterMotor.setPower(shooterPower);
            helperWheel.setPower(sushiRollerSpeed);
            shooterHood.setPosition(hoodAngle);
        }



        }

        }

