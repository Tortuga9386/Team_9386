package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
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

        public DcMotorEx shooterMotor;

        public ShooterMotor() { //HardwareMap hardwareMap, RobotBase opMode
            initHardware();
        }

        public double hoodAngle = 1;

        protected void initHardware() {
            shooterMotor = hardwareMap.get(DcMotorEx.class, "shooterRoller");
            shooterMotor.setDirection(DcMotor.Direction.REVERSE);
            shooterMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            shooterMotor.setVelocityPIDFCoefficients(60,0,0,13);
            }

        public void goToTargetSpeed(double targetSpeed) {
            shooterMotor.setVelocity(targetSpeed);
        }



        }

        }

