package org.firstinspires.ftc.teamcode.subsystems;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.opmodes.RobotBase;
import org.firstinspires.ftc.teamcode.subsystems.*;

public class CONTROL_CENTER {
    protected RobotBase robotBase;
    private ElapsedTime controlCenterRuntime = new ElapsedTime();

    public void initControlCenter (){
        robotBase.turret.initHardware();
        robotBase.intake.initHardware();
        robotBase.indexer.initHardware();
        robotBase.shooter.initHardware();
        robotBase.drive.initHardware();
    }

    public void startControlCenter(){
        controlCenterRuntime.reset();
    }

    //TeleOp Stuff
    public void TurretTeleOp (Gamepad gamepad, double tagNumber){
        if (tagNumber == 24){
            robotBase.turret.limelight.pipelineSwitch(1);//change to actual number
        }
        if (tagNumber == 20){
            robotBase.turret.limelight.pipelineSwitch(0);//change to actual number
        }
        double limelightX = robotBase.turret.limelight.getLatestResult().getTx();

        double limelightPID = limelightX;// add pid

        if (gamepad.left_trigger > 0.125){
            robotBase.turret.turretMotor.goToTarget(limelightPID);
        }
        if (gamepad.left_trigger < 0.125){
            robotBase.turret.turretMotor.goToTarget(0);
        }
    }

    public void intakeAndIndexerTeleop (Gamepad gamepad){
        if (gamepad.left_bumper){
            robotBase.intake.intakeRoller.gotoTarget(-1);
        }
        if (gamepad.right_bumper){
            robotBase.intake.intakeRoller.gotoTarget(1);
            robotBase.indexer.indexerSystem.goToTarget(1,0.875,0.875);
        }
        if (gamepad.x && !gamepad.a){
            robotBase.indexer.indexerSystem.goToTarget(1,0.62,0);
        }
        if (gamepad.a && !gamepad.x){
            robotBase.indexer.indexerSystem.goToTarget(1,0,0.65);
        }
        if (gamepad.x && gamepad.a){
            robotBase.indexer.indexerSystem.goToTarget(1,0.62,0.65);
        }
        if (!gamepad.left_bumper && !gamepad.right_bumper){
            robotBase.intake.intakeRoller.gotoTarget(0);
        }
        if ((!gamepad.a && !gamepad.x) ){
            robotBase.indexer.indexerSystem.goToTarget(0,0.875,0.875);
        }

    }
    public void shooterTeleop (Gamepad gamepad){

        double calculatedHoodAngle = 1;
        if (gamepad.right_trigger > 0.125){
            robotBase.shooter.shooterMotor.goToTarget(1, 1,calculatedHoodAngle);
        }
        if (gamepad.right_trigger < 0.125){
            robotBase.shooter.shooterMotor.goToTarget(0,0,calculatedHoodAngle);
        }
    }


    //Auto Stuff
    public void TurretLongAutoOp (double tagNumber){
        if (tagNumber == 24){
            robotBase.turret.limelight.pipelineSwitch(1);//change to actual number
        }
        if (tagNumber == 20){
            robotBase.turret.limelight.pipelineSwitch(0);//change to actual number
        }
        double limelightX = robotBase.turret.limelight.getLatestResult().getTx();

        double limelightPID = limelightX;// add pid

        robotBase.turret.turretMotor.goToTarget(limelightPID);

    }

    public void driveLongAutoOp (){
        if (controlCenterRuntime.seconds() < 10) {
            robotBase.drive.moveToPos(0, 0, 0);
        }
        if (controlCenterRuntime.seconds() > 10){
            robotBase.drive.moveToPos(10,0,0);
        }
    }

    public void shooterLongShotAutoOp(){
            robotBase.shooter.shooterMotor.goToTarget(1, 1, 0.55);
        }

    public void indexerAndIntakeLongAutoOp (){
        if (controlCenterRuntime.seconds() < 5){
            robotBase.indexer.indexerSystem.goToTarget(1, 0.62,0.7);
        }
        if (controlCenterRuntime.seconds() > 5 && controlCenterRuntime.seconds() < 10){
            robotBase.indexer.indexerSystem.goToTarget(1, 0.875, 0.65);
        }
        if (controlCenterRuntime.seconds() > 10 && controlCenterRuntime.seconds() < 12){
            robotBase.indexer.indexerSystem.goToTarget(1,0.875,0.875);
        }
        if (controlCenterRuntime.seconds() > 12 && controlCenterRuntime.seconds() <15){
            robotBase.intake.intakeRoller.gotoTarget(1);
        }
        if (controlCenterRuntime.seconds() > 15 && controlCenterRuntime.seconds() < 17) {
            robotBase.indexer.indexerSystem.goToTarget(1, 0.62, 0.65);
            robotBase.intake.intakeRoller.gotoTarget(0);
        }

    }




}
