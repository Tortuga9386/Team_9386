package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.TankDrive;
import org.firstinspires.ftc.teamcode.tuning.TuningOpModes;

@Disabled
@Autonomous(name="Test Roadrunner Sequence 2", group = "auto")
public final class RoadRunnerTest2 extends LinearOpMode {

    double NORTH = Math.PI;
    double EAST = 2 * Math.PI;
    double SOUTH = 3 * Math.PI / 2;
    double WEST = Math.PI / 2;

    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d beginPose = new Pose2d(18, -65, WEST);
        if (TuningOpModes.DRIVE_CLASS.equals(MecanumDrive.class)) {
            MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);

            waitForStart();

            Actions.runBlocking(
                drive.actionBuilder(beginPose)
                        .splineTo(new Vector2d(0, -30.5), WEST)
                        .build());
        } else {
            throw new RuntimeException();
        }
    }
}
