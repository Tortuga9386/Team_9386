package org.firstinspires.ftc.teamcode.opmodes;

import android.util.Log;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.MecanumDrive;

@Disabled
@Autonomous(name="Test Roadrunner Sequence", group = "auto")
public class TestRoadRunnerSequence extends RobotBase {

    double NORTH = Math.PI;
    double EAST = 2 * Math.PI;
    double SOUTH = 3 * Math.PI / 2;
    double WEST = Math.PI / 2;
    //public static final Class<?> DRIVE_CLASS = MecanumDrive.class;
    private int iterationCounter = 0;

    public TestRoadRunnerSequence() {
    }

    @Override
    public void init() {
        super.init();
    }


    public void loop() {

        iterationCounter++;
        if (iterationCounter < 2) {
            Log.i("Test RR Seq", "--- Below execution count (" + iterationCounter + ") ---");
            Pose2d beginPose = new Pose2d(18, -65, WEST);
            MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);

            Actions.runBlocking(
                    drive.actionBuilder(beginPose)
                            .splineTo(new Vector2d(0, -30.5), WEST)
                            .build());

//            TrajectoryActionBuilder path1 = drive.actionBuilder(beginPose)
//                            .splineTo(new Vector2d(0, -30.5), WEST);
//
//            Action trajectoryActionChosen;
//            trajectoryActionChosen = path1.build();
//
//            Actions.runBlocking(
//                    new SequentialAction(
//                            trajectoryActionChosen
//                    )
//            );

        } else if (iterationCounter <= 500) {
            Log.i("Test RR Seq", "--- Exceeded execution count ("+iterationCounter+") ---");
        }
    }

}
