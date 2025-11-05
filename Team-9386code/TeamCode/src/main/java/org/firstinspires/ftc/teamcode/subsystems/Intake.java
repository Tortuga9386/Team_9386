package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import org.firstinspires.ftc.teamcode.subsystems.Indexer;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.opmodes.RobotBase;
import org.firstinspires.ftc.teamcode.subsystems.Drive;

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

            else {
                intakePower = 0;
            }

        }

        public void goToTarget(double intakePower) {
            intakeMotor.setPower(intakePower);
        }
    }
}

