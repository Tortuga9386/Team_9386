package org.firstinspires.ftc.teamcode.subsystems;

import static android.graphics.Color.BLUE;
import static android.graphics.Color.RED;

import static org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit.INCH;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
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
        public ElapsedTime runtime = new ElapsedTime();

        public double TPS; //far:4000 close 3550
        public double motorTPS;

        private double snapShotRuntime;
        private double snapShotRuntimeIntakeOut;

        public boolean lifterTimer1 = false;
        public boolean lifterTimer2 = false;
        private boolean lifterTimer3 = false;
        private boolean lifterTimer4 = false;
        private boolean lifterTimer5 = false;
        private boolean lifterTimer6 = false;
        private boolean lifterTimer7 = false;
        private boolean lifterTimer8 = false;

        private boolean rollerTimer1 = false;
        private boolean rollerTimer2 = false;

        private boolean runRoller = false;

        private boolean runTimer = false;

        public boolean slightLift = false;

        public double leftLifterPos;
        public double rightLifterPos;


        public boolean triggerCheck;



        public void teleop(Gamepad gamepad1, Gamepad gamepad2, boolean allianceRedIsTrue) {
            //Virtual_turret
            double rawLimelightX;
            double rotationX;

            rawLimelightX = -robotBase.drive.limelight3A.getLatestResult().getTx();

            if (gamepad2.right_trigger > 0.1 || gamepad1.right_trigger > 0.1) {
                rotationX = rawLimelightX / 27.25;
            } else {
                rotationX = 0;
            }

            if (allianceRedIsTrue) {
                robotBase.drive.limelight3A.pipelineSwitch(0);
            }

            if (!allianceRedIsTrue) {
                robotBase.drive.limelight3A.pipelineSwitch(1);
            }

            //Indexer


            if (gamepad1.left_trigger > 0.1 || gamepad2.left_trigger > 0.85) {
                triggerCheck = true;

                snapShotRuntime = runtime.seconds();
            }


            if (triggerCheck) {

                if (runtime.seconds() > (snapShotRuntime + 0)) {
                    lifterTimer1 = true;
                }
                if (runtime.seconds() > (snapShotRuntime + 0.225)) {
                    lifterTimer2 = true;
                }

                if (runtime.seconds() > (snapShotRuntime + 0.45)) {
                    lifterTimer3 = true;
                }
                if (runtime.seconds() > (snapShotRuntime + 0.75)) {//0.675
                    lifterTimer4 = true;
                }

                if (runtime.seconds() > (snapShotRuntime + 0.925)) {
                    rollerTimer1 = true;
                }
                if (runtime.seconds() > (snapShotRuntime + 1.675)) {
                    rollerTimer2 = true;
                }

                if (runtime.seconds() > (snapShotRuntime + 1.675)) {
                    lifterTimer5 = true;
                }
                if (runtime.seconds() > (snapShotRuntime + 1.9)) {
                    lifterTimer6 = true;
                }
                if (runtime.seconds() > (snapShotRuntime + 2.125)) {
                    lifterTimer7 = true;
                }
                if (runtime.seconds() > (snapShotRuntime + 2.35)) {
                    lifterTimer8 = true;
                }

                if (runtime.seconds() > (snapShotRuntime + 2.4)) {
                    runTimer = true;
                }

                if (lifterTimer1 && !lifterTimer2) {
                    leftLifterPos = 3;
                }
                if (lifterTimer2) {
                    leftLifterPos = 2;
                }

                if (lifterTimer3 && !lifterTimer4) {
                    rightLifterPos = 3;
                }
                if (lifterTimer4) {
                    rightLifterPos = 2;
                }

                if (rollerTimer1 && !rollerTimer2) {
                    runRoller = true;
                }
                if (rollerTimer2) {
                    runRoller = false;
                }
                if (lifterTimer5 && !lifterTimer6) {
                    leftLifterPos = 3;
                }
                if (lifterTimer6) {
                    leftLifterPos = 1;
                }
                if (lifterTimer7 && !lifterTimer8) {
                    rightLifterPos = 3;
                }
                if (lifterTimer8) {
                    rightLifterPos = 1;
                }
                if (runTimer) {
                    lifterTimer1 = false;
                    lifterTimer2 = false;
                    lifterTimer3 = false;
                    lifterTimer4 = false;
                    lifterTimer5 = false;
                    lifterTimer6 = false;
                    rollerTimer1 = false;
                    rollerTimer2 = false;
                    lifterTimer7 = false;
                    lifterTimer8 = false;

                    triggerCheck = false;
                    runTimer = false;
                }

            }

            if (gamepad2.a || gamepad1.a){
                leftLifterPos = 3;
                rightLifterPos = 3;
            }

            if (!triggerCheck && (!gamepad1.a && !gamepad2.a) && (!gamepad1.right_bumper && !gamepad2.right_bumper)){
                leftLifterPos = 1;
                rightLifterPos = 1;
            }

            if (gamepad1.right_bumper || gamepad2.right_bumper) {
                if ((robotBase.intake.intakeRoller.leftDistance.getDistance(DistanceUnit.MM) < 75 && robotBase.intake.intakeRoller.rightDistance.getDistance(DistanceUnit.MM) < 75)) {
                    leftLifterPos = 1.5;
                    rightLifterPos = 1.5;
                }

                if ((robotBase.intake.intakeRoller.leftDistance.getDistance(DistanceUnit.MM) > 75 && robotBase.intake.intakeRoller.rightDistance.getDistance(DistanceUnit.MM) < 75)){
                    leftLifterPos = 1;
                    rightLifterPos = 1;
                }
            }



            //Shooter


            if (gamepad2.dpad_up || gamepad1.dpad_up) {
                TPS = (4000 * 28) / 60;
            }

            if (gamepad2.dpad_down || gamepad1.dpad_down) {
                TPS = (3550 * 28) / 60;
            }

            if (gamepad1.rightBumperWasReleased() || gamepad2.rightBumperWasReleased()){
                snapShotRuntimeIntakeOut = runtime.seconds() + 0.2;
            }






            robotBase.shooter.shooterMotor.goToTargetSpeed(TPS);


            if (runRoller){
                robotBase.intake.intakeRoller.goToTarget(1);
            }
            if (((gamepad2.right_bumper || gamepad1.right_bumper) && !triggerCheck)) {
                robotBase.intake.intakeRoller.goToTarget(1);

            }

            if (gamepad2.left_bumper || gamepad1.left_bumper || (snapShotRuntimeIntakeOut > runtime.seconds() && (!gamepad1.right_bumper && !gamepad2.right_bumper))) {
                robotBase.intake.intakeRoller.goToTarget(-1);
            }

            if ((!gamepad2.right_bumper && !gamepad1.right_bumper) && (!gamepad2.left_bumper && !gamepad1.left_bumper) && !runRoller && (snapShotRuntimeIntakeOut < runtime.seconds())) {
            robotBase.intake.intakeRoller.goToTarget(0);
            }

            //Drive_base
            robotBase.drive.driveFromGamepad(gamepad1, -rotationX);


        }

        public void Auto_op(boolean alianceRedIsTrue) {

            if (alianceRedIsTrue){
                robotBase.drive.limelight3A.pipelineSwitch(0);
            }

            if (!alianceRedIsTrue){
                robotBase.drive.limelight3A.pipelineSwitch(1);
            }
            if (runtime.seconds() > 1.95 && runtime.seconds() < 2.7){
                robotBase.intake.intakeRoller.goToTarget(1);
            }
            if (runtime.seconds() > 2.7 && runtime.seconds() < 3 ){
                robotBase.intake.intakeRoller.goToTarget(0);
            }

            if (runtime.seconds() > 5 && runtime.seconds() < 9){
                robotBase.intake.intakeRoller.goToTarget(1);
            }
            if (runtime.seconds() > 9 && runtime.seconds() < 9.6 ){
                robotBase.intake.intakeRoller.goToTarget(0);
            }

            if (runtime.seconds() > 9.6 && runtime.seconds() < 9.75){
                robotBase.intake.intakeRoller.goToTarget(-1);
            }
            if (runtime.seconds() > 9.75 && runtime.seconds() < 10.1 ){
                robotBase.intake.intakeRoller.goToTarget(0);
            }

            if (runtime.seconds() < 5.75){
                robotBase.shooter.shooterMotor.goToTargetSpeed((3950 * 28) / 60);
            }


            if (runtime.seconds() > 0.75 && runtime.seconds() < 1.05){
                leftLifterPos = 3;
            }
            if (runtime.seconds() > 1.05 && runtime.seconds() < 1.35){
                leftLifterPos = 2;
            }

            if (runtime.seconds() > 1.35 && runtime.seconds() < 1.65){
                rightLifterPos = 3;
            }
            if (runtime.seconds() > 1.65 && runtime.seconds() < 1.95){
                rightLifterPos = 2;
            }
            if (runtime.seconds() > 2.7 && runtime.seconds() < 3){
                rightLifterPos = 3;
            }
            if (runtime.seconds() > 3 && runtime.seconds() < 3.3){
                rightLifterPos = 1;
            }
            if (runtime.seconds() > 3.3 && runtime.seconds()  < 3.6){
                leftLifterPos = 3;
            }
            if (runtime.seconds() > 3.6 && runtime.seconds() < 3.9){
                leftLifterPos = 1;
            }






            if (runtime.seconds() > 12 && runtime.seconds() < 12.3){
                leftLifterPos = 3;
            }
            if (runtime.seconds() > 12.3 && runtime.seconds() < 12.6){
                leftLifterPos = 2;
            }

            if (runtime.seconds() > 12.6 && runtime.seconds() < 12.9){
                rightLifterPos = 3;
            }
            if (runtime.seconds() > 12.9 && runtime.seconds() < 13.2){
                rightLifterPos = 2;
            }
            if (runtime.seconds() > 13.95 && runtime.seconds() < 14.25){
                rightLifterPos = 3;
            }
            if (runtime.seconds() > 14.25 && runtime.seconds() < 14.55){
                rightLifterPos = 1;
            }
            if (runtime.seconds() > 14.55 && runtime.seconds()  < 14.85){
                leftLifterPos = 3;
            }
            if (runtime.seconds() > 14.85 && runtime.seconds() < 15.15){
                leftLifterPos = 1;
            }
            if (runtime.seconds() > 18 && runtime.seconds() < 18.5){
                leftLifterPos = 3;
                rightLifterPos = 3;
            }

            if (runtime.seconds() > 13.2 && runtime.seconds() < 13.95){
                robotBase.intake.intakeRoller.goToTarget(1);
            }
            if (runtime.seconds() > 13.95 && runtime.seconds() < 14.1 ){
                robotBase.intake.intakeRoller.goToTarget(0);
            }

            if (runtime.seconds() > 10 && runtime.seconds() < 20){
                robotBase.shooter.shooterMotor.goToTargetSpeed((3950 * 28) / 60);
            }

            if (runtime .seconds() > 20 && runtime.seconds() < 21){
                robotBase.shooter.shooterMotor.goToTargetSpeed(0);
            }





        }

        public void Auto_op2(boolean alianceRedIsTrue) {

            if (alianceRedIsTrue){
                robotBase.drive.limelight3A.pipelineSwitch(0);
            }

            if (!alianceRedIsTrue){
                robotBase.drive.limelight3A.pipelineSwitch(1);
            }
            if (runtime.seconds() > 1.95 && runtime.seconds() < 2.7){
                robotBase.intake.intakeRoller.goToTarget(1);
            }
            if (runtime.seconds() > 2.7 && runtime.seconds() < 3 ){
                robotBase.intake.intakeRoller.goToTarget(0);
            }

            if (runtime.seconds() > 5 && runtime.seconds() < 9){
                robotBase.intake.intakeRoller.goToTarget(1);
            }
            if (runtime.seconds() > 9 && runtime.seconds() < 9.6 ){
                robotBase.intake.intakeRoller.goToTarget(0);
            }

            if (runtime.seconds() > 9.6 && runtime.seconds() < 9.75){
                robotBase.intake.intakeRoller.goToTarget(-1);
            }
            if (runtime.seconds() > 9.75 && runtime.seconds() < 10.1 ){
                robotBase.intake.intakeRoller.goToTarget(0);
            }

            if (runtime.seconds() < 5.75){
                robotBase.shooter.shooterMotor.goToTargetSpeed((3950 * 28) / 60);
            }


            if (runtime.seconds() > 0.75 && runtime.seconds() < 1.05){
                leftLifterPos = 3;
            }
            if (runtime.seconds() > 1.05 && runtime.seconds() < 1.35){
                leftLifterPos = 2;
            }

            if (runtime.seconds() > 1.35 && runtime.seconds() < 1.65){
                rightLifterPos = 3;
            }
            if (runtime.seconds() > 1.65 && runtime.seconds() < 1.95){
                rightLifterPos = 2;
            }
            if (runtime.seconds() > 2.7 && runtime.seconds() < 3){
                rightLifterPos = 3;
            }
            if (runtime.seconds() > 3 && runtime.seconds() < 3.3){
                rightLifterPos = 1;
            }
            if (runtime.seconds() > 3.3 && runtime.seconds()  < 3.6){
                leftLifterPos = 3;
            }
            if (runtime.seconds() > 3.6 && runtime.seconds() < 3.9){
                leftLifterPos = 1;
            }






            if (runtime.seconds() > 12 && runtime.seconds() < 12.3){
                leftLifterPos = 3;
            }
            if (runtime.seconds() > 12.3 && runtime.seconds() < 12.6){
                leftLifterPos = 2;
            }

            if (runtime.seconds() > 12.6 && runtime.seconds() < 12.9){
                rightLifterPos = 3;
            }
            if (runtime.seconds() > 12.9 && runtime.seconds() < 13.2){
                rightLifterPos = 2;
            }
            if (runtime.seconds() > 13.95 && runtime.seconds() < 14.25){
                rightLifterPos = 3;
            }
            if (runtime.seconds() > 14.25 && runtime.seconds() < 14.55){
                rightLifterPos = 1;
            }
            if (runtime.seconds() > 14.55 && runtime.seconds()  < 14.85){
                leftLifterPos = 3;
            }
            if (runtime.seconds() > 14.85 && runtime.seconds() < 15.15){
                leftLifterPos = 1;
            }
            if (runtime.seconds() > 18 && runtime.seconds() < 18.5){
                leftLifterPos = 3;
                rightLifterPos = 3;
            }

            if (runtime.seconds() > 13.2 && runtime.seconds() < 13.95){
                robotBase.intake.intakeRoller.goToTarget(1);
            }
            if (runtime.seconds() > 13.95 && runtime.seconds() < 14.1 ){
                robotBase.intake.intakeRoller.goToTarget(0);
            }

            if (runtime.seconds() > 10 && runtime.seconds() < 20){
                robotBase.shooter.shooterMotor.goToTargetSpeed((3950 * 28) / 60);
            }

            if (runtime .seconds() > 20 && runtime.seconds() < 21){
                robotBase.shooter.shooterMotor.goToTargetSpeed(0);
            }





        }
        public void runLifters () {
                if (leftLifterPos == 1) {
            robotBase.indexer.indexerSystem.leftLifterMotor.setTargetPosition(-662);
        }
                if (leftLifterPos == 1.5) {
            robotBase.indexer.indexerSystem.leftLifterMotor.setTargetPosition(-525);
        }
                if (leftLifterPos == 2) {
            robotBase.indexer.indexerSystem.leftLifterMotor.setTargetPosition(-490);
        }
                if (leftLifterPos == 3) {
            robotBase.indexer.indexerSystem.leftLifterMotor.setTargetPosition(0);
        }

                if (rightLifterPos == 1) {
            robotBase.indexer.indexerSystem.rightLifterMotor.setTargetPosition(-662);
        }
                if (rightLifterPos == 1.5) {
            robotBase.indexer.indexerSystem.rightLifterMotor.setTargetPosition(-525);
        }
                if (rightLifterPos == 2) {
            robotBase.indexer.indexerSystem.rightLifterMotor.setTargetPosition(-490);
        }
                if (rightLifterPos == 3) {
            robotBase.indexer.indexerSystem.rightLifterMotor.setTargetPosition(0);
            }
        }
    }
}

