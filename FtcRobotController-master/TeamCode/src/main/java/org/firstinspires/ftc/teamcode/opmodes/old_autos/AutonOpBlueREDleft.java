//package org.firstinspires.ftc.teamcode.opmodes;
//
//import com.acmerobotics.roadrunner.Pose2d;
//import com.acmerobotics.roadrunner.Vector2d;
//import com.acmerobotics.roadrunner.ftc.Actions;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.Disabled;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//
//import org.firstinspires.ftc.teamcode.MecanumDrive;
//import org.firstinspires.ftc.teamcode.tuning.TuningOpModes;
//@Disabled
//@Autonomous(name="1) LEFT SPECIMEN", group = "auto")
//public final class AutonOpBlueREDleft extends LinearOpMode {
//
//    double SOUTH = Math.PI;
//    double EAST = 3 * Math.PI / 2;
//    double NORTH = 0;
//    double WEST = Math.PI / 2;
//
//    @Override
//    public void runOpMode() throws InterruptedException {
//        Pose2d beginPose = new Pose2d(-29.5, -65, WEST);
//        if (TuningOpModes.DRIVE_CLASS.equals(MecanumDrive.class)) {
//            MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);
//
//            waitForStart();
//
//            Actions.runBlocking(
//                    drive.actionBuilder(beginPose)
//                            .splineToConstantHeading(new Vector2d(-4, -29), WEST) //spline out to the sub
//                            .waitSeconds(2)
//                            .lineToYConstantHeading(-35)//back up from the sub
//                            .splineToConstantHeading(new Vector2d(-47,-44 ), WEST)
//                            .turnTo(EAST)
//                            .waitSeconds(0.01)
//                            .splineToConstantHeading(new Vector2d(-63,-54), EAST)// baskettttt
//                            .waitSeconds(0.01)
//                            .strafeToConstantHeading(new Vector2d(-60,-54))
//                            .turnTo(WEST)
//                            .splineToConstantHeading(new Vector2d(-56,-44),WEST)
//                            .turnTo(EAST)
//                            .waitSeconds(0.01)
//                            .splineToConstantHeading(new Vector2d(-63,-54), EAST)// baskettttt
//                            .waitSeconds(0.01)
//                            .strafeToConstantHeading(new Vector2d(-58, -54))
//                            .turnTo(NORTH)
//                            .splineToConstantHeading(new Vector2d(-24,-6),NORTH)//level 1 ascent
//                            .build());
//        } else {
//            throw new RuntimeException();
//        }
//    }
//}
