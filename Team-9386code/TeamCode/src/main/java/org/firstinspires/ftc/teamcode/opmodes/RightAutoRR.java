package org.firstinspires.ftc.teamcode.opmodes;

import androidx.annotation.NonNull;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.teamcode.MecanumDrive;

import com.qualcomm.robotcore.hardware.HardwareMap;

@Config
@Disabled
@Autonomous(name = "RIGHT_AUTO_RR", group = "Autonomous")
public class RightAutoRR extends LinearOpMode {

    double SOUTH = Math.PI;
    double EAST = 3 * Math.PI / 2;
    double NORTH = 0;
    double WEST = Math.PI / 2;

    public static class RobotLift {
        private DcMotor liftlift;

        public RobotLift(HardwareMap hardwareMap) {
            liftlift = hardwareMap.get(DcMotor.class, "lift");
            liftlift.setDirection(DcMotor.Direction.FORWARD);
            liftlift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            liftlift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        public class LiftUp implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    liftlift.setPower(0.5);
                    initialized = true;
                }

                double pos = liftlift.getCurrentPosition();
                packet.put("liftPos", pos);
                if (pos < 3000.0) {
                    return true;
                } else {
                    liftlift.setPower(0);
                    return false;
                }
            }
        }
        public Action liftUp() {
            return new LiftUp();
        }

        public class LiftDown implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    liftlift.setPower(-0.5);
                    initialized = true;
                }

                double pos = liftlift.getCurrentPosition();
                packet.put("liftPos", pos);
                if (pos > 50.0) {
                    return true;
                } else {
                    liftlift.setPower(0);
                    return false;
                }
            }
        }
        public Action liftDown(){
            return new LiftDown();
        }
    }

    @Override
    public void runOpMode() {
        Pose2d initialPose = new Pose2d(16.5, -65, WEST);
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
        RobotLift lift = new RobotLift(hardwareMap);

        // vision here that outputs position
        int visionOutputPosition = 1;

        TrajectoryActionBuilder rightPathToSub = drive.actionBuilder(initialPose)
                .splineToConstantHeading(new Vector2d(4, -29), WEST); //spline out to the sub

        TrajectoryActionBuilder rightPathDropSamples = drive.actionBuilder(initialPose)
                .splineToConstantHeading(new Vector2d(24,-48), WEST)
                .waitSeconds(0.01)
                .splineToConstantHeading(new Vector2d(36,-24 ), WEST)
                .turnTo(NORTH) //turn 90 degrease
                .lineToXConstantHeading(46)//captures first block
                .turnTo(EAST)
                .splineToConstantHeading(new Vector2d(48, -60), EAST) // pushes the the first block
                .lineToY(-12)
                .strafeToConstantHeading(new Vector2d(54, -12)) //strafe aiming for the second block
                .waitSeconds(0.01)
                .strafeToConstantHeading(new Vector2d(54,-60)) //pushes the second block into the zone
                .lineToY(-12)
                .strafeToConstantHeading(new Vector2d(62, -12)) //strafe aiming for the third block
                .waitSeconds(0.01)
                .turnTo(EAST)
                .strafeToConstantHeading(new Vector2d(62,-60)); //pushes the third block into the zone

        while (!isStopRequested() && !opModeIsActive()) {
            int position = visionOutputPosition;
            telemetry.addData("Position during Init", position);
            telemetry.update();
        }

        int startPosition = visionOutputPosition;
        telemetry.addData("Starting Position", startPosition);
        telemetry.update();
        waitForStart();

        if (isStopRequested()) return;

        Action trajectoryActionToSub;
        Action trajectoryActionDropSamples;
        //PUT SWITCH CASE HERE
        trajectoryActionToSub = rightPathToSub.build();
        trajectoryActionDropSamples = rightPathDropSamples.build();

        Actions.runBlocking(
                new SequentialAction(
                        trajectoryActionToSub,
                        lift.liftUp(),
                        lift.liftDown(),
                        trajectoryActionDropSamples
                )
        );
    }
}
