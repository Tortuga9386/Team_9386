package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.opmodes.RobotBase;

public class Shooter {
    protected HardwareMap hardwareMap;
    protected Telemetry telemetry;
    protected RobotBase robotBase;
    public ShooterMotor shooterMotor;
//    public LiftSlide liftSlide;
//    public IntakeSlide intakeSlide;
//    public IntakeClaw intakeClaw;
//    public IntakeLinkage intakelinkage;
//    public Climber climber;
//    public Roller roller;
//    public Tilter tilter;

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

        public DcMotor shooterMotor;

        public ShooterMotor() { //HardwareMap hardwareMap, RobotBase opMode
            initHardware();
        }

        public double targetSpeed = 0;
        ;


        protected void initHardware() {
            shooterMotor = hardwareMap.get(DcMotor.class, "shooter");
            shooterMotor.setDirection(DcMotor.Direction.REVERSE);
            shooterMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        public void doShooterStuff(Gamepad gamepad2) {

            if (gamepad2.y) {
                targetSpeed = 0.75;
            }
            if (gamepad2.y) {
                targetSpeed = 0.75;
            }
            else {
                targetSpeed = 0;      }


        }

        }
    }
