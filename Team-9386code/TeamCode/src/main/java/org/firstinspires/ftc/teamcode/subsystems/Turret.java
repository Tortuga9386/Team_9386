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
import com.qualcomm.hardware.limelightvision.LLResultTypes.*;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import java.util.List;

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

        public double targetSpeedl = 0;
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
            limelight.pipelineSwitch(0);
        }

        public void doTurretStuff(Gamepad gamepad2) {

            Limelight3A limelight = hardwareMap.get(Limelight3A.class, "limelight");
// limelight.start(); // Ensure polling is started

            LLResult result = limelight.getLatestResult();

            if (result != null && result.isValid()) {
                // Get the list of all detected AprilTags
                List<FiducialResult> fiducials = result.getFiducialResults();

                for (FiducialResult fiducial : fiducials) {
                    // **This is the key method to get the ID**
                    int aprilTagId = fiducial.getFiducialId();

                    // Example: Display the ID and other data
                    telemetry.addData("Detected AprilTag ID", aprilTagId);

                    // You can also get pose data (distance, rotation, etc.) here
                    // double distance = fiducial.getRobotPoseTargetSpace().getY();
                }
            }




            targetSpeed = ((limelight.getLatestResult().getTx() / 27.25)*1);
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
            turretMotor.setPower(targetSpeed);
        }


    }
}


