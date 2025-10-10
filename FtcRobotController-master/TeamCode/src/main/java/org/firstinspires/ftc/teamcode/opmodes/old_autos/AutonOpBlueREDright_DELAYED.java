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
//@Autonomous(name="2) RIGHT DELAYED", group = "auto")
//public final class AutonOpBlueREDright_DELAYED extends LinearOpMode {
//
//    double SOUTH = Math.PI;
//    double EAST = 3 * Math.PI / 2;
//    double NORTH = 0;
//    double WEST = Math.PI / 2;
//
//    @Override
//    public void runOpMode() throws InterruptedException {
//        Pose2d beginPose = new Pose2d(16.5, -65, WEST);
//        if (TuningOpModes.DRIVE_CLASS.equals(MecanumDrive.class)) {
//            MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);
//
//            waitForStart();
//
//            Actions.runBlocking(
//                    drive.actionBuilder(beginPose)
//                            .waitSeconds(4.5)
//                            .splineToConstantHeading(new Vector2d(4, -29), WEST) //spline out to the sub
//                            .waitSeconds(2) //back up from the sub
//                            .lineToYConstantHeading(-35)//back up from the sub
//                            .splineToConstantHeading(new Vector2d(36,-24 ), WEST)
//                            .turnTo(NORTH)
//                            .lineToXConstantHeading(46)//captures first block
//                            .turnTo(EAST)
//                            .splineToConstantHeading(new Vector2d(48, -60), EAST) // pushes the the first block
//                            .lineToY(-12)
//                            .strafeToConstantHeading(new Vector2d(54, -12)) //strafe aiming for the second block
//                            .waitSeconds(0.01)
//                            .strafeToConstantHeading(new Vector2d(54,-60)) //pushes the second block into the zone
//                            .lineToY(-12)
//                            .strafeToConstantHeading(new Vector2d(62, -12)) //strafe aiming for the third block
//                            .waitSeconds(0.01)
//                            .turnTo(EAST)
//                            .strafeToConstantHeading(new Vector2d(62,-60)) //pushes the third block into the zone
//                            //.strafeToConstantHeading(new Vector2d(66,-12))
//                            .build());
//        } else {
//            throw new RuntimeException();
//        }
//    }
//}
