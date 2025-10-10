package org.firstinspires.ftc.teamcode.Teleop;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
@Disabled
@TeleOp(name="opMode", group="Linear OpMode")
public class opMode extends LinearOpMode {
    public void runOpMode() {
        DcMotor DcMotorA = hardwareMap.get(DcMotor.class, "Drive1");
        DcMotor DcMotorB = hardwareMap.get(DcMotor.class, "Drive2");
        DcMotor DcMotorC = hardwareMap.get(DcMotor.class, "Drive3");
        DcMotor DcMotorD = hardwareMap.get(DcMotor.class, "Drive4");


        waitForStart();
        while (opModeIsActive()){
            if (gamepad1.left_stick_y != 0){
                DcMotorA.setPower(-gamepad1.left_stick_y);
                DcMotorB.setPower(-gamepad1.left_stick_y);
                DcMotorC.setPower(-gamepad1.left_stick_y);
                DcMotorD.setPower(-gamepad1.left_stick_y);
            }
            if (gamepad1.left_stick_y == 0) {
                DcMotorA.setPower(0);
                DcMotorB.setPower(0);
                DcMotorC.setPower(0);
                DcMotorD.setPower(0);
            }
            if (gamepad1.right_stick_x != 0){
                DcMotorA.setPower(gamepad1.right_stick_x);
                DcMotorB.setPower(gamepad1.right_stick_x);
                DcMotorC.setPower(-gamepad1.right_stick_x);
                DcMotorD.setPower(-gamepad1.right_stick_x);
            }
            else {
                DcMotorA.setPower(0);
                DcMotorB.setPower(0);
                DcMotorC.setPower(0);
                DcMotorD.setPower(0);
            }

        }
    }
}
