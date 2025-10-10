package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MecanumDrive;
@Disabled
@Config
@Autonomous(name = "Tuning", group = "Autonomous")
public class TuningProgram extends LinearOpMode {

    double SOUTH = Math.PI;
    double EAST = 3 * Math.PI / 2;
    double NORTH = 0;
    double NORTH_EAST = 7 *Math.PI /4;
    double WEST = Math.PI / 2;

    @Override
    public void runOpMode() {
        Pose2d initialPose = new Pose2d(0, -0, WEST);
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
        ActionLib.RobotLift             lift            = new ActionLib.RobotLift(hardwareMap);
        ActionLib.RobotIntake      intakeSlide     = new ActionLib.RobotIntake(hardwareMap);
//        ActionLib.RobotIntakeRotator    intakeRotator   = new ActionLib.RobotIntakeRotator(hardwareMap);
        ActionLib.RobotIntakeClaw       intakeClaw      = new ActionLib.RobotIntakeClaw(hardwareMap);

        //Init robot position
        intakeClaw.actionClawClose();


        initialPose = new Pose2d(0, 0, WEST);// driving to the first sample

        TrajectoryActionBuilder rightPathToSub = drive.actionBuilder(initialPose)
                .strafeToConstantHeading(new Vector2d(24,24));

        Action trajectoryActionToSub = rightPathToSub.build();


        initialPose = new Pose2d(24, 24, WEST);// driving to the first sample

        TrajectoryActionBuilder rightPathToSubBack = drive.actionBuilder(initialPose)
                .strafeToConstantHeading(new Vector2d(0,0));

        Action trajectoryActionToSubBack = rightPathToSubBack.build();

        TrajectoryActionBuilder wait = drive.actionBuilder(initialPose)
                .waitSeconds(1);

        Action trajectoryActionwait = wait.build();
        telemetry.addData("Status", "> INIT");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        Actions.runBlocking(
            new SequentialAction(
                trajectoryActionToSub,
                trajectoryActionToSubBack,
                trajectoryActionwait
            )
        );
    }
}
