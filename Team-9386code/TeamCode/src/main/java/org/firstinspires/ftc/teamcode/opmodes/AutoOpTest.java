package org.firstinspires.ftc.teamcode.opmodes;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.Drive;


@Autonomous(name="TEST", group="Autonomous")
public class AutoOpTest extends RobotBase
{

    private   ElapsedTime   runtime = new ElapsedTime();

    public AutoOpTest() {}

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
        auto_op_loop();
        telemetry_loop();
    }
    protected void auto_op_loop(){
        control_center.control_center1.Auto_op();
    }




    protected void drive_loop() {
        drive.moveToPos(0,36,90,gamepad1);
    }




    protected void telemetry_loop() {
        telemetry.addData("MOTOR RF", drive.rightFrontMotor.getPower());
        telemetry.addData("MOTOR LF", drive.leftFrontMotor.getPower());
        telemetry.addData("MOTOR RR", drive.rightRearMotor.getPower());
        telemetry.addData("MOTOR LR", drive.leftRearMotor.getPower());

        telemetry.update();
    }

}
