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
import org.firstinspires.ftc.robotcore.internal.usb.UsbSerialNumber;
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
        public    ElapsedTime   runtime = new ElapsedTime();

        public double TPS ; //far:4000 close 3550
        public double motorTPS ;

        private double snapShotRuntime;

        public boolean lifterTimer1 = false;
        public boolean lifterTimer2 = false;
        private boolean lifterTimer3 = false;
        private boolean lifterTimer4 = false;
        private boolean lifterTimer5 = false;
        private boolean lifterTimer6 = false;

        private boolean rollerTimer1 = false;
        private boolean rollerTimer2 = false;

        private boolean runRoller = false;

        private boolean runTimer = false;



        public boolean triggerCheck;
        public void teleop(Gamepad gamepad1, Gamepad gamepad2, boolean allianceRedIsTrue){
            //Virtual_turret
            double rawLimelightX;
            double rotationX;

            rawLimelightX = -robotBase.drive.limelight3A.getLatestResult().getTx();

            if (gamepad2.right_trigger > 0.1 || gamepad1.right_trigger > 0.1){
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



            if (gamepad1.left_trigger > 0.1){
                triggerCheck = true;

                snapShotRuntime = runtime.seconds();
            }




            if (triggerCheck){

                if (runtime.seconds() > (snapShotRuntime + 0)){
                    lifterTimer1 = true;
                }
                if (runtime.seconds() > (snapShotRuntime + 0.5)){
                    lifterTimer2 = true;
                }

                if (runtime.seconds() > (snapShotRuntime + 0.75)){
                    lifterTimer3 = true;
                }
                if (runtime.seconds() > (snapShotRuntime + 1.25)){
                    lifterTimer4 = true;
                }

                if (runtime.seconds() > (snapShotRuntime + 1.25)){
                    rollerTimer1 = true;
                }
                if (runtime.seconds() > (snapShotRuntime + 3.25)){
                    rollerTimer2 = true;
                }

                if (runtime.seconds() > (snapShotRuntime + 3.25)){
                    lifterTimer5 = true;
                }
                if (runtime.seconds() > (snapShotRuntime + 4)){
                    lifterTimer6 = true;
                }

                if (runtime.seconds() > (snapShotRuntime + 4.25)){
                    runTimer = true;
                }

                if(lifterTimer1 && !lifterTimer2){
                    robotBase.indexer.indexerSystem.leftLifterMotor.setTargetPosition(0);
                }
                if (lifterTimer2){
                    robotBase.indexer.indexerSystem.leftLifterMotor.setTargetPosition(-490);
                }

                if(lifterTimer3 && !lifterTimer4){
                    robotBase.indexer.indexerSystem.rightLifterMotor.setTargetPosition(0);
                }
                if (lifterTimer4){
                    robotBase.indexer.indexerSystem.rightLifterMotor.setTargetPosition(-490);
                }

                if(rollerTimer1 && !rollerTimer2){
                    runRoller = true;
                }
                if (rollerTimer2){
                    runRoller = false;
                }
                if(lifterTimer5 && !lifterTimer6){
                    robotBase.indexer.indexerSystem.rightLifterMotor.setTargetPosition(0);
                    robotBase.indexer.indexerSystem.leftLifterMotor.setTargetPosition(0);
                }
                if (lifterTimer6){
                    robotBase.indexer.indexerSystem.rightLifterMotor.setTargetPosition(-662);
                    robotBase.indexer.indexerSystem.leftLifterMotor.setTargetPosition(-662);
                }
                if (runTimer){
                    lifterTimer1 = false;
                    lifterTimer2 = false;
                    lifterTimer3 = false;
                    lifterTimer4 = false;
                    lifterTimer5 = false;
                    lifterTimer6 = false;
                    rollerTimer1 = false;
                    rollerTimer2 = false;

                    triggerCheck = false;
                    runTimer = false;
                }

            }
            if (!triggerCheck){
                robotBase.indexer.indexerSystem.leftLifterMotor.setTargetPosition(-662);
                robotBase.indexer.indexerSystem.rightLifterMotor.setTargetPosition(-662);
            }
//            if (gamepad2.a) {
//                robotBase.indexer.indexerSystem.leftLifterMotor.setTargetPosition(0);
//            }if (gamepad2.b) {
//                robotBase.indexer.indexerSystem.rightLifterMotor.setTargetPosition(0);
//            }
//            if  (!gamepad2.b && !gamepad2.a) {
//                robotBase.indexer.indexerSystem.leftLifterMotor.setTargetPosition(-690);
//                robotBase.indexer.indexerSystem.rightLifterMotor.setTargetPosition(-690);
//            }

            //Shooter


            if (gamepad2.dpad_up || gamepad1.dpad_up){
                TPS = (4000 * 28) /60;
            }

            if (gamepad2.dpad_down || gamepad1.dpad_down){
                TPS = (3550 * 28) /60;
            }

            if (gamepad2.right_trigger < 0.1 && gamepad1.right_trigger < 0.1){
                motorTPS = 0;
            }

            if (gamepad2.right_trigger > 0.1 || gamepad1.right_trigger > 0.1) {
                motorTPS = TPS;
            }


            robotBase.shooter.shooterMotor.goToTargetSpeed(motorTPS);
            //Intake
            if (gamepad2.right_bumper || gamepad1.right_bumper || runRoller){
                robotBase.intake.intakeRoller.goToTarget(1);
            }

            if (gamepad2.left_bumper || gamepad1.left_bumper){
                robotBase.intake.intakeRoller.goToTarget(-1);
            }

            if ((!gamepad2.right_bumper && !gamepad1.right_bumper) && (!gamepad2.left_bumper && !gamepad1.left_bumper) && !runRoller){
                robotBase.intake.intakeRoller.goToTarget(0);
            }

            //Drive_base
            robotBase.drive.driveFromGamepad(gamepad1, -rotationX);


        }

        public class Auto_op {
            }
        }
    }

