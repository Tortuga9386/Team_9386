package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.opmodes.RobotBase;

public class Intake {
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
    }

    protected void initHardware() {
        intakeRoller = new IntakeRoller();
    }

    public class IntakeRoller {

        public DcMotor intakeMotor;;

        public IntakeRoller() { //HardwareMap hardwareMap, RobotBase opMode
            initHardware();
        }

        //public double intakePower = 0;

        protected void initHardware() {
            intakeMotor = hardwareMap.get(DcMotor.class, "IntakeRollers");
            intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            intakeMotor.setDirection(DcMotor.Direction.FORWARD);
        }

//        public void doIntakeStuff() {
//            goToTarget(intakePower);
//        }
//
//        public void goToTarget(double intakePower) {
//            intakeMotor.setPower(intakePower);
//        }
    }
}

