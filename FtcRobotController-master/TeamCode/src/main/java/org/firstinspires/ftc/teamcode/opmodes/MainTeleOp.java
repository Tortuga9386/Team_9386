package org.firstinspires.ftc.teamcode.opmodes;


import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.subsystems.Lift;


@TeleOp(name="***TeleOp***", group="teleop")
public class MainTeleOp extends RobotBase
{

    private   ElapsedTime   runtime = new ElapsedTime();

    public MainTeleOp() {}

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
        lift_loop();
        //husky_loop();
    }


    protected void lift_loop() {
        lift.liftSlide.doSlideStuff(gamepad2);
        lift.intakeSlide.doIntakeSlideStuff(gamepad2);
        lift.intakeClaw.doIntakeClawStuff(gamepad2);
        lift.intakelinkage.doIntakeLinkageStuff(gamepad2);
        lift.climber.doClimberStuff(gamepad1);
        lift.roller.doIntakeRollerStuff(gamepad2);
        lift.tilter.doIntakeTilterStuff(gamepad2);
//        telemetry.addData("encoder", lift.liftSlide.liftMotor.getCurrentPosition());
//        telemetry.addData("encoderslide", lift.intakeSlide.intakeliftMotor.getCurrentPosition());
//        telemetry.addData("linkage pos",lift.intakelinkage.intakeLinkage.getPosition());
//        telemetry.addData("climber pos1",lift.climber.climberMotor1.getCurrentPosition());
//        telemetry.addData("Distance_MM's",lift.roller.intakeSensor.getDistance(DistanceUnit.MM));
    }

    protected void drive_loop() {


        //slidePosition
        drive.turtleFactor = ((1 - 0.5 * lift.liftSlide.liftMotor.getCurrentPosition() / 5000) * 1);
        drive.driveFromGamepad(gamepad1);
    }

//    protected void husky_loop() {
//        sensorHuskyLens.setAlgorithm("COLOR");
//        sensorHuskyLens.getColorBlocks();
//    }

    protected void imu_loop() {

    }

    protected void telemetry_loop() {

    }

}
