package org.firstinspires.ftc.teamcode.opmodes;


import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;


@TeleOp(name="***TeleOp_RED***", group="teleop")
public class MainTeleOpRed extends RobotBase
{

    private   ElapsedTime   runtime = new ElapsedTime();

    public MainTeleOpRed() {}

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {



        super.INITIALIZE_DRIVE  = true;

        super.init();

        //Set initial positions
        telemetry.addData("Status", "init complete");
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {

    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        runtime.reset();
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        drive_loop();
        shooter_loop();
        indexer_loop();
        telemetry_loop();
        turret_loop();
    }

    protected void indexer_loop(){
        indexer.indexerSystem.doIndexerStuff(gamepad2);
        indexer.indexerSystem.colorSensorStuff();
    }



    protected void shooter_loop(){
        shooter.shooterMotor.doShooterStuff(gamepad2);
    }

    protected void drive_loop() {
        drive.driveFromGamepad(gamepad1);
    }

    protected void turret_loop(){
        control_center.TurretTeleOp(gamepad1, 24);
    }



    protected void imu_loop() {

    }

    protected void telemetry_loop() {
        telemetry.addData("Hood Angle", shooter.shooterMotor.hoodAngle);
        telemetry.addData("selection for seq", indexer.indexerSystem.shooterSelection);
        telemetry.addData("selection for selector", indexer.indexerSystem.gamepadSelection);
        telemetry.addData("x", turret.limelight.getLatestResult().getTx());
        telemetry.addData("area", turret.limelight.getLatestResult().getTa());
        telemetry.addData("y", turret.limelight.getLatestResult().getTy());
//        telemetry.addData("turretSpeed", turret.turretMotor.targetSpeed);
        telemetry.update();
    }

}
