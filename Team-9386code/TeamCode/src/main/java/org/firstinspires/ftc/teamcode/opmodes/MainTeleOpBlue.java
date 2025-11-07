package org.firstinspires.ftc.teamcode.opmodes;


import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;


@TeleOp(name="***TeleOp_blue***", group="teleop")
public class MainTeleOpBlue extends RobotBase
{

    private   ElapsedTime   runtime = new ElapsedTime();

    public MainTeleOpBlue() {}

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
        intake_loop();
        indexer_loop();
        telemetry_loop();
        turret_loop();
        //lift_loop();
        //husky_loop();
    }

    protected void indexer_loop(){
        indexer.indexerSystem.doIndexerStuff(gamepad2);
        indexer.indexerSystem.colorSensorStuff();
    }

    protected void intake_loop(){
        intake.intakeRoller.doIntakeStuff(gamepad2);
    }

    protected void shooter_loop(){
        shooter.shooterMotor.doShooterStuff(gamepad2);
    }

    protected void turret_loop(){
        turret.turretMotor.doTurretBStuff(gamepad2);
    }

    protected void drive_loop() {
        drive.driveFromGamepad(gamepad1);
    }

    protected void imu_loop() {

    }

    protected void telemetry_loop() {
        telemetry.addData("Hood Angle", shooter.shooterMotor.hoodAngle);
        telemetry.addData("selection for seq", indexer.indexerSystem.shooterSelection);
        telemetry.addData("selection for selector", indexer.indexerSystem.gamepadSelection);
        telemetry.addData("lifterHeight", indexer.indexerSystem.lifterPos);
        telemetry.update();
    }

}
