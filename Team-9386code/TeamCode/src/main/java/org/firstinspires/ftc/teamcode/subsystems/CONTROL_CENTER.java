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
    public void doTurretStuffTeleOp (Gamepad gamepad, double tagNumber){
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
        if (controlCenterRuntime.seconds() < 10) {
            robotBase.shooter.shooterMotor.goToTarget(1, 1, 0.55);
        }
        if (controlCenterRuntime.seconds() > 10){
            robotBase.shooter.shooterMotor.goToTarget(0,0,1);
        }
    }

    public void indexerAndIntakeAndShooterAutoOp (){

    }




}
