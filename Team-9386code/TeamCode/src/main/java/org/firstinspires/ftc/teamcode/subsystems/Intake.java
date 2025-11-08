package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

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

        private final ElapsedTime sequenceTimer = new ElapsedTime();

        public void resetSequenceTimer(){
            sequenceTimer.reset();
        }

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

