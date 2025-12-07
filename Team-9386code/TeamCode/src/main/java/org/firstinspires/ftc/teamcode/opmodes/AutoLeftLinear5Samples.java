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
@Autonomous(name = "AUTO_RIGHT_3_SAMPLE", group = "Autonomous")
public class AutoLeftLinear5Samples extends LinearOpMode {

    double SOUTH = Math.PI;
    double EAST = 3 * Math.PI / 2;
    double NORTH = 0;
    double NORTH_EAST = 7 *Math.PI /4;
    double NORTH_WEST = Math.PI/4;
    double WEST = Math.PI / 2;
    double SOUTHEAST = 1.25 * Math.PI;


    @Override
    public void runOpMode() {
        Pose2d initialPose = new Pose2d(-41, -65, WEST);
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
        ActionLib.RobotLift             lift            = new ActionLib.RobotLift(hardwareMap);
        ActionLib.RobotIntake      intakeSlide     = new ActionLib.RobotIntake(hardwareMap);
//        ActionLib.RobotIntakeRotator    intakeRotator   = new ActionLib.RobotIntakeRotator(hardwareMap);
        ActionLib.RobotIntakeClaw       intakeClaw      = new ActionLib.RobotIntakeClaw(hardwareMap);
        ActionLib.RobotIntakeTilter     intakeTilter    =new ActionLib.RobotIntakeTilter(hardwareMap);
        ActionLib.RobotIntakeRoller     intakeRoller   =new ActionLib.RobotIntakeRoller(hardwareMap);
        ActionLib.RobotIntakeLinkage     intakelinkage   =new ActionLib.RobotIntakeLinkage(hardwareMap);
        ActionLib.RobotIntakefinger     intakefinger   =new ActionLib.RobotIntakefinger(hardwareMap);


        //Init robot position
        intakeClaw.clawClose();
        intakeSlide.actionIntakeUp();
        intakeTilter.tilterDown();
        intakelinkage.linkageIn();
        intakefinger.fingerIn();

        TrajectoryActionBuilder leftPathToBasket = drive.actionBuilder(initialPose)
                .strafeToLinearHeading(new Vector2d(-56, -58),SOUTHEAST);
        Action trajectoryActionToBasket = leftPathToBasket.build();

        initialPose = new Pose2d(-56, -58, SOUTHEAST);
        TrajectoryActionBuilder leftPathToSample1 = drive.actionBuilder(initialPose)
                .strafeToLinearHeading(new Vector2d(-50, -40),WEST);
        Action trajectoryActionToSample1 = leftPathToSample1.build();

        initialPose = new Pose2d(-50, -40, WEST);
        TrajectoryActionBuilder leftPathToGrabSample1 = drive.actionBuilder(initialPose)
                .strafeToConstantHeading(new Vector2d(-50, -30));
        Action trajectoryActionToGrabSample1 = leftPathToGrabSample1.build();

        initialPose = new Pose2d(-50, -30, WEST);
        TrajectoryActionBuilder leftPathToBasket2 = drive.actionBuilder(initialPose)
                .strafeToLinearHeading(new Vector2d(-56, -58),SOUTHEAST);
        Action trajectoryActionToBasket2 = leftPathToBasket2.build();

        initialPose = new Pose2d(-56, -58, SOUTHEAST);
        TrajectoryActionBuilder leftPathToSample2 = drive.actionBuilder(initialPose)
                .strafeToLinearHeading(new Vector2d(-60, -40),WEST);
        Action trajectoryActionToSample2 = leftPathToSample2.build();

        initialPose = new Pose2d(-60, -40, WEST);
        TrajectoryActionBuilder leftPathToGrabSample2 = drive.actionBuilder(initialPose)
                .strafeToConstantHeading(new Vector2d(-60, -30));
        Action trajectoryActionToGrabSample2 = leftPathToGrabSample2.build();

        initialPose = new Pose2d(-60, -30, WEST);
        TrajectoryActionBuilder leftPathToBasket3 = drive.actionBuilder(initialPose)
                .strafeToLinearHeading(new Vector2d(-55.5, -57.5),SOUTHEAST);
        Action trajectoryActionToBasket3 = leftPathToBasket3.build();

        initialPose = new Pose2d(-55.5, -57.5, SOUTHEAST);
        TrajectoryActionBuilder leftPathToSample3 = drive.actionBuilder(initialPose)
                .strafeToLinearHeading(new Vector2d(-63, -50),1.919862);
        Action trajectoryActionToSample3 = leftPathToSample3.build();

        initialPose = new Pose2d(-63, -50, 1.919862);
        TrajectoryActionBuilder leftPathToGrabSample3 = drive.actionBuilder(initialPose)
                .strafeToLinearHeading(new Vector2d(-63, -42),1.919862);
        Action trajectoryActionToGrabSample3 = leftPathToGrabSample3.build();

        initialPose = new Pose2d(-63, -42, 1.919862);
        TrajectoryActionBuilder leftPathToBasket4 = drive.actionBuilder(initialPose)
                .strafeToLinearHeading(new Vector2d(-55, -58),4.014257);
        Action trajectoryActionToBasket4 = leftPathToBasket4.build();

        initialPose = new Pose2d(-55, -58, 4.014257);
        TrajectoryActionBuilder leftPathToPark = drive.actionBuilder(initialPose)
                .strafeToLinearHeading(new Vector2d(-36, -8),SOUTH)
                .strafeToConstantHeading(new Vector2d(-28, -8));
        Action trajectoryActionToPark = leftPathToPark.build();



// wait section claw down
        initialPose = new Pose2d(6.25, -65, WEST);
        TrajectoryActionBuilder pathWait1 = drive.actionBuilder(initialPose)
                .waitSeconds(0.5);
        Action trajectoryActionpathWait1 = pathWait1.build();

        initialPose = new Pose2d(6.25, -65, WEST);
        TrajectoryActionBuilder pathWait3 = drive.actionBuilder(initialPose)
                .waitSeconds(0.5);
        Action trajectoryActionpathWait2 = pathWait3.build();

        initialPose = new Pose2d(6.25, -65, WEST);
        TrajectoryActionBuilder pathWait4 = drive.actionBuilder(initialPose)
                .waitSeconds(0.5);
        Action trajectoryActionpathWait3 = pathWait4.build();


        initialPose = new Pose2d(2, -29.9, WEST);
        TrajectoryActionBuilder pathWait2 = drive.actionBuilder(initialPose)
                .waitSeconds(0.5);
        Action trajectoryActionpathWait4 = pathWait2.build();

        initialPose = new Pose2d(2, -29.9, WEST);
        TrajectoryActionBuilder pathWait5 = drive.actionBuilder(initialPose)
                .waitSeconds(0.5);
        Action trajectoryActionpathWait5 = pathWait5.build();

        initialPose = new Pose2d(2, -29.9, WEST);
        TrajectoryActionBuilder pathWait6 = drive.actionBuilder(initialPose)
                .waitSeconds(0.5);
        Action trajectoryActionpathWait6 = pathWait6.build();

        initialPose = new Pose2d(2, -29.9, WEST);
        TrajectoryActionBuilder pathWait7 = drive.actionBuilder(initialPose)
                .waitSeconds(1);
        Action trajectoryActionpathWait7 = pathWait7.build();

// wait section claw up


        telemetry.addData("Status", "> INIT");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        Actions.runBlocking(
                new SequentialAction(
                        new ParallelAction(trajectoryActionToBasket,lift.actionLiftToBasket(),intakeClaw.actionClawClose(),intakeTilter.actionTilterUp()),
                        intakeClaw.actionClawOpen(),
                        trajectoryActionpathWait1,
                        new ParallelAction(lift.actionLiftDown(),intakeSlide.actionIntakeUp(),trajectoryActionToSample1),
                        new ParallelAction(intakeSlide.actionIntakeDown(),intakeTilter.actionTilterUp()),
                        trajectoryActionToGrabSample1,
                        intakeClaw.actionClawClose(),
                        trajectoryActionpathWait2,
                        new ParallelAction(intakeSlide.actionIntakeUp(),trajectoryActionToBasket2,lift.actionLiftToBasket()),
                        intakeClaw.actionClawOpen(),
                        trajectoryActionpathWait3,
                        new ParallelAction(lift.actionLiftDown(),intakeSlide.actionIntakeUp(),trajectoryActionToSample2),
                        new ParallelAction(intakeSlide.actionIntakeDown(),intakeTilter.actionTilterUp()),
                        trajectoryActionToGrabSample2,
                        intakeClaw.actionClawClose(),
                        trajectoryActionpathWait4,
                        new ParallelAction(intakeSlide.actionIntakeUp(),trajectoryActionToBasket3,lift.actionLiftToBasket()),
                        intakeClaw.actionClawOpen(),
                        trajectoryActionpathWait5,
                        new ParallelAction(trajectoryActionToSample3,lift.actionClawGrab(),intakeTilter.actionTilterDown(),intakeRoller.actionRollerIn()),
                        intakeSlide.actionIntakeDown(),
                        trajectoryActionToGrabSample3,
                        trajectoryActionpathWait6,
                        new ParallelAction(intakeRoller.actionRollerStop(),intakeSlide.actionIntakeUp()),
                        new ParallelAction(lift.actionLiftToBasketRoller(),intakeTilter.actionTilterUp(),trajectoryActionToBasket4),
                        intakeRoller.actionRollerIn(),
                        trajectoryActionpathWait7,
                        new ParallelAction(trajectoryActionToPark,lift.actionLiftDown(),intakeRoller.actionRollerStop(),intakefinger.actionfingerOut())


                )
            );
        }
    }
