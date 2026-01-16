package org.firstinspires.ftc.teamcode.subsystems;

import static android.graphics.Color.BLUE;
import static android.graphics.Color.RED;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.opmodes.RobotBase;

public class Control_center {
    protected HardwareMap hardwareMap;
    protected Telemetry telemetry;
    protected RobotBase robotBase;
    public Control_center1 control_center1;

    public Control_center(HardwareMap hardwareMap, RobotBase opMode) {
        this.hardwareMap = hardwareMap;
        this.robotBase = opMode;
        this.telemetry = robotBase.telemetry;

        initHardware();
    }

    public Control_center(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
    }

    protected void initHardware() {
        control_center1 = new Control_center1();

    }

    public class Control_center1 {

        public double TPS ; //far:4000 close 3550
        public double motorTPS ;
        public void teleop(Gamepad gamepad1, Gamepad gamepad2, boolean allianceRedIsTrue){
            //Virtual_turret
            double rawLimelightX;
            double rotationX;

            rawLimelightX = -robotBase.drive.limelight3A.getLatestResult().getTx();

            if (gamepad2.right_trigger > 0.1 || gamepad1.left_trigger > 0.1){
                rotationX = rawLimelightX / 27.25;
            }
            else {
                rotationX = 0;
            }

            if (allianceRedIsTrue) {
                robotBase.drive.limelight3A.pipelineSwitch(0);
            }

            if (!allianceRedIsTrue){
                robotBase.drive.limelight3A.pipelineSwitch(1);
            }

            //Indexer
            if (gamepad2.a) {
                robotBase.indexer.indexerSystem.leftLifterMotor.setTargetPosition(0);
            }if (gamepad2.b) {
                robotBase.indexer.indexerSystem.rightLifterMotor.setTargetPosition(0);
            }
            if  (!gamepad2.b && !gamepad2.a) {
                robotBase.indexer.indexerSystem.leftLifterMotor.setTargetPosition(-690);
                robotBase.indexer.indexerSystem.rightLifterMotor.setTargetPosition(-690);
            }

            //Shooter


            if (gamepad2.dpad_up || gamepad1.dpad_up){
                TPS = (4000 * 28) /60;
            }

            if (gamepad2.dpad_down || gamepad1.dpad_down){
                TPS = (3550 * 28) /60;
            }

            if (gamepad2.right_trigger < 0.1){
                motorTPS = 0;
            }

            if (gamepad2.right_trigger > 0.1 ) {
                motorTPS = TPS;
            }


            robotBase.shooter.shooterMotor.goToTargetSpeed(motorTPS);

            //Drive_base
            robotBase.drive.driveFromGamepad(gamepad1, -rotationX);


        }

        public class Auto_op {
            }
        }
    }

