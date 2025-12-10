package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.opmodes.RobotBase;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Turret {
    protected HardwareMap hardwareMap;
    protected Telemetry telemetry;
    protected RobotBase robotBase;
    public TurretMotor turretMotor;
    public Limelight3A limelight;

    public Turret(HardwareMap hardwareMap, RobotBase opMode) {
        this.hardwareMap = hardwareMap;
        this.robotBase = opMode;
        this.telemetry = robotBase.telemetry;

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

        public final ElapsedTime turretTimer = new ElapsedTime();

        public double targetSpeedR = 0;
        public double targetSpeedB = 0;
        public double targetSpeedRa = 0;
        public double targetSpeedBa = 0;



        protected void initHardware() {
            turretMotor = hardwareMap.get(DcMotor.class, "Turret");
            turretMotor.setDirection(DcMotor.Direction.FORWARD);
            turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            turretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            limelight = hardwareMap.get(Limelight3A.class, "limelight");
            limelight.setPollRateHz(100); // This sets how often we ask Limelight for data (100 times per second)
            limelight.start(); // This tells Limelight to start looking!
        }

        public void doTurretStuffRT(Gamepad gamepad1) {
            limelight.pipelineSwitch(0);

            targetSpeedR = ((limelight.getLatestResult().getTx() / 27.25)*1);
            if (gamepad1.left_trigger > 0.25) {
                if (targetSpeedR < 0.125) {
                    targetSpeedR = targetSpeedR - 0.025;
                }
                if (targetSpeedR > 0.125) {
                    targetSpeedR = targetSpeedR + 0.025;
                }
            }
            else {
                targetSpeedR = 0;
            }


            goToTargetSpeedRT(targetSpeedR);
        }
            public void goToTargetSpeedRT ( double targetSpeedR) {
                turretMotor.setPower(targetSpeedR);
            }

            public void doTurretStuffBT(Gamepad gamepad1) {
            limelight.pipelineSwitch(1);

            targetSpeedB = ((limelight.getLatestResult().getTx() / 27.25)*1);
            if (gamepad1.left_trigger > 0.25) {
                if (targetSpeedB < 0.125) {
                    targetSpeedB = targetSpeedB - 0.025;
                }
                if (targetSpeedB > 0.125) {
                    targetSpeedB = targetSpeedB + 0.025;
                }
            }
            else {
                targetSpeedB = 0;
            }


            goToTargetSpeedBT(targetSpeedB);
        }
            public void goToTargetSpeedBT ( double targetSpeedB) {
                turretMotor.setPower(targetSpeedB);
            }




            public void doTurretStuffRA() {
            limelight.pipelineSwitch(0);

            targetSpeedRa = (((limelight.getLatestResult().getTx() - 2) / 27.25)*0.9);
            if (turretTimer.seconds() > 0) {
                if (targetSpeedRa < 0.125) {
                    targetSpeedRa = targetSpeedRa - 0.025;
                }
                if (targetSpeedRa > 0.125) {
                    targetSpeedRa = targetSpeedRa + 0.025;
                }
            }
            if (turretTimer.seconds() > 17) {
                targetSpeedRa = 0;
            }


            goToTargetSpeedRa(targetSpeedRa);
        }

            public void goToTargetSpeedRa ( double targetSpeedRa) {
                turretMotor.setPower(targetSpeedRa);
            }

            public void doTurretStuffBA() {
            limelight.pipelineSwitch(1);

                targetSpeedBa = (((limelight.getLatestResult().getTx() + 2) / 27.25)*0.9);
            if (turretTimer.seconds() > 0) {
                if (targetSpeedBa < 0.125) {
                    targetSpeedBa = targetSpeedBa - 0.025;
                }
                if (targetSpeedBa > 0.125) {
                    targetSpeedBa = targetSpeedBa + 0.025;
                }
            }
            if (turretTimer.seconds() > 17) {
                targetSpeedBa = 0;
            }


            goToTargetSpeedBa(targetSpeedBa);
        }
            public void goToTargetSpeedBa ( double targetSpeedBa) {
                turretMotor.setPower(targetSpeedBa);
            }


        }
    }


