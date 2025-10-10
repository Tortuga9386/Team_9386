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
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MecanumDrive;

@Config
@Autonomous(name = "AUTO_RIGHT_4_SPEC", group = "Autonomous")
public class AutoRightLinearHighPointHang4 extends LinearOpMode {

    double SOUTH = Math.PI;
    double EAST = 3 * Math.PI / 2;
    double NORTH = 0;
    double NORTH_EAST = 7 *Math.PI /4;
    double NORTH_WEST = Math.PI/4;
    double WEST = Math.PI / 2;
    double SOUTHEAST = 1.25 * Math.PI;


    @Override
    public void runOpMode() {
        Pose2d initialPose = new Pose2d(6.25, -65, WEST);
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
        ActionLib.RobotLift             lift            = new ActionLib.RobotLift(hardwareMap);
        ActionLib.RobotIntake      intakeSlide     = new ActionLib.RobotIntake(hardwareMap);
//        ActionLib.RobotIntakeRotator    intakeRotator   = new ActionLib.RobotIntakeRotator(hardwareMap);
        ActionLib.RobotIntakeClaw       intakeClaw      = new ActionLib.RobotIntakeClaw(hardwareMap);
        ActionLib.RobotIntakeTilter     intakeTilter    =new ActionLib.RobotIntakeTilter(hardwareMap);
        ActionLib.RobotIntakeLinkage     intakelinkage   =new ActionLib.RobotIntakeLinkage(hardwareMap);

        //Init robot position
        intakeClaw.clawClose();
        intakeSlide.actionIntakeUp();
        intakeTilter.tilterDown();
        intakelinkage.linkageIn();

        TrajectoryActionBuilder rightPathToSub = drive.actionBuilder(initialPose)
                .strafeToConstantHeading(new Vector2d(-4, -31.5)); //spline out to the sub and scoring first spec
        Action trajectoryActionToSub = rightPathToSub.build();

        initialPose = new Pose2d(-4, -31.5, WEST);
        TrajectoryActionBuilder backUpFromSub = drive.actionBuilder(initialPose)
                .strafeToLinearHeading(new Vector2d(24, -48),NORTH_WEST)//backup from sub
                .strafeToLinearHeading(new Vector2d(33,-31), NORTH)
                .strafeToLinearHeading(new Vector2d(42, -20), NORTH_EAST)
                .strafeToLinearHeading(new Vector2d(48, -36), EAST) //pushing first sample
                .strafeToConstantHeading(new Vector2d(46, -65.5));//strafe to first sample
        Action trajectoryActionbackUpFromSub = backUpFromSub.build();


        initialPose = new Pose2d(46, -65.5, EAST);
        TrajectoryActionBuilder pathToSub2 = drive.actionBuilder(initialPose)
                .strafeToConstantHeading(new Vector2d(46,-58))
                .strafeToLinearHeading(new Vector2d(0, -29.4), WEST); //pushing to first sample
        Action trajectoryActionpathToSub2 = pathToSub2.build();

        initialPose = new Pose2d(0, -29.4, WEST);
        TrajectoryActionBuilder backUpFromSub2 = drive.actionBuilder(initialPose)
                .strafeToConstantHeading(new Vector2d(2,-32))
                .strafeToLinearHeading(new Vector2d(24, -42),NORTH_WEST)//backup from sub
                .strafeToLinearHeading(new Vector2d(40,-31), NORTH)
                .strafeToLinearHeading(new Vector2d(56, -20), NORTH_EAST)
                .strafeToLinearHeading(new Vector2d(48, -36), EAST) //pushing to first sample
                .strafeToConstantHeading(new Vector2d(46, -66.5));//strafe to first sample
        Action trajectoryActionbackUpFromSub2 = backUpFromSub2.build();

        initialPose = new Pose2d(46, -66.5, EAST);
        TrajectoryActionBuilder pathToSub3 = drive.actionBuilder(initialPose)
                .strafeToConstantHeading(new Vector2d(46,-58))
                .strafeToLinearHeading(new Vector2d(2, -40), 1.570796)
                .strafeToConstantHeading(new Vector2d(2,-29.5));
        Action trajectoryActionpathToSub3 = pathToSub3.build();


        initialPose = new Pose2d(2, -29.5, WEST);
        TrajectoryActionBuilder backUpFromSub3 = drive.actionBuilder(initialPose)
                .strafeToConstantHeading(new Vector2d(2,-32))//back up from sub
                //.strafeToLinearHeading(new Vector2d(24, -42),NORTH)
                //.strafeToLinearHeading(new Vector2d(45.5,-30), NORTH_EAST)
                //.strafeToLinearHeading(new Vector2d(67, -18), EAST)
                .strafeToLinearHeading(new Vector2d(42,-67),4.747296);
        Action trajectoryActionbackUpFromSub3 = backUpFromSub3.build();

        initialPose = new Pose2d(42, -67, 4.747296);
        TrajectoryActionBuilder pathToSub4 = drive.actionBuilder(initialPose)
                .strafeToConstantHeading(new Vector2d(46,-58))
                .strafeToLinearHeading(new Vector2d(2, -40), 1.570796)
                .strafeToConstantHeading(new Vector2d(2,-29.5));
        Action trajectoryActionpathToSub4 = pathToSub4.build();

        initialPose = new Pose2d(2, -29.5, WEST);
        TrajectoryActionBuilder backUpFromSub4 = drive.actionBuilder(initialPose)
                .strafeToConstantHeading(new Vector2d(2,-34))//back up from sub
                .strafeToConstantHeading(new Vector2d(42,-67));
        Action trajectoryActionbackUpFromSub4 = backUpFromSub4.build();


// wait section claw down
        initialPose = new Pose2d(6.25, -65, WEST);
        TrajectoryActionBuilder pathWait1 = drive.actionBuilder(initialPose)
                .waitSeconds(0.25);
        Action trajectoryActionpathpathWait1 = pathWait1.build();

        initialPose = new Pose2d(6.25, -65, WEST);
        TrajectoryActionBuilder pathWait3 = drive.actionBuilder(initialPose)
                .waitSeconds(0.1);
        Action trajectoryActionpathpathWait3 = pathWait3.build();

        initialPose = new Pose2d(6.25, -65, WEST);
        TrajectoryActionBuilder pathWait4 = drive.actionBuilder(initialPose)
                .waitSeconds(0.1);
        Action trajectoryActionpathpathWait4 = pathWait4.build();

        initialPose = new Pose2d(6.25, -65, WEST);
        TrajectoryActionBuilder pathWait5 = drive.actionBuilder(initialPose)
                .waitSeconds(0.1);
        Action trajectoryActionpathpathWait5 = pathWait5.build();


        initialPose = new Pose2d(2, -29.9, WEST);
        TrajectoryActionBuilder pathWait2 = drive.actionBuilder(initialPose)
                .waitSeconds(30);
        Action trajectoryActionpathpathWait2 = pathWait2.build();

// wait section claw up


        telemetry.addData("Status", "> INIT");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        Actions.runBlocking(
            new SequentialAction(
                    new ParallelAction(
                            intakeSlide.actionIntakeUp(),
                            intakelinkage.actionLinkageIn(),
                            lift.actionLiftSpecimen(),
                            intakeTilter.actionTilterUp(),
                            new SequentialAction(trajectoryActionpathpathWait1,trajectoryActionToSub)),
                    lift.actionliftScore(),
                    intakeClaw.actionClawOpen(),
                    new ParallelAction(trajectoryActionbackUpFromSub,lift.actionClawGrabSpec(),intakeSlide.actionIntakeUp()),
                    intakeClaw.actionClawClose(),
                    trajectoryActionpathpathWait4,
                    new ParallelAction(intakeSlide.actionIntakeUp(),trajectoryActionpathToSub2,lift.actionLiftSpecimen()),
                    lift.actionliftScore(),
                    intakeClaw.actionClawOpen(),
                    new ParallelAction(trajectoryActionbackUpFromSub2,lift.actionClawGrabSpec(),intakeSlide.actionIntakeUp()),
                    intakeClaw.actionClawClose(),
                    trajectoryActionpathpathWait3,
                    new ParallelAction(intakeSlide.actionIntakeUp(),trajectoryActionpathToSub3,lift.actionLiftSpecimen()),
                    lift.actionliftScore(),
                    intakeClaw.actionClawOpen(),
                    new ParallelAction(trajectoryActionbackUpFromSub3,lift.actionClawGrabSpec(),intakeSlide.actionIntakeUp()),
                    intakeClaw.actionClawClose(),
                    trajectoryActionpathpathWait5,
                    new ParallelAction(trajectoryActionpathToSub4,intakeSlide.actionIntakeUp(),lift.actionLiftSpecimen()),
                    lift.actionliftScore(),
                    intakeClaw.actionClawOpen(),
                    new ParallelAction(trajectoryActionbackUpFromSub4,lift.actionLiftDown()),
                    trajectoryActionpathpathWait2


            )
        );
    }
}
