package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.opmodes.RobotBase;

import java.util.List;

public class DriveRoadRunnerMec extends Drive{

    protected MecanumDrive mecanumDrive;

    public DriveRoadRunnerMec(HardwareMap hardwareMap, RobotBase opMode) {
        super(hardwareMap,opMode);
        initHardware();
    }

    protected void initHardware() {
        super.initHardware();
        //mecanumDrive = new MecanumDrive(hardwareMap, Pose2d pose2D);
    }

    public boolean isBusy() {
        return mecanumDrive.leftBack.isBusy();
    }

    public void setPoseEstimate(Pose2d pose2D) {
        //mecanumDrive.setPoseEstimate(pose2D);
    }
    //public void setPoseEstimate() { setPoseEstimate(new Pose2d()); }

//    public void followTrajectory(Trajectory trajectory) {
//        mecanumDrive.followTrajectory(trajectory);
//    }
//    public void followTrajectoryAsync(Trajectory trajectory) {
//        mecanumDrive.followTrajectoryAsync(trajectory);
//    }
//    public boolean testEncoders() {
//        //List<Double> wheelPositions = mecanumDrive.getWheelPositions();
//        boolean allOdometersPass = true;
////        if (mecanumDrive.leftRear.getCurrentPosition() == 0.0) {
////            telemetry.addData("ODOMETER [FRONT]", "NO READING");
////            allOdometersPass = false;
////        }
////        if (mecanumDrive.rightFront.getCurrentPosition() == 0.0) {
////            telemetry.addData("ODOMETER [RIGHT]", "NO READING");
////            allOdometersPass = false;
////        }
////        if (mecanumDrive.leftFront.getCurrentPosition() == 0.0) {
////            telemetry.addData("ODOMETER [LEFT]", "NO READING");
////            allOdometersPass = false;
////        }
//
//        if (allOdometersPass) {
//            return true;
//        }
//        return false;
//    }

//    public void forward(double inches) {
//        Trajectory forwardTrajectory = mecanumDrive.trajectoryBuilder(new Pose2d())
//                .forward(inches)
//                .build();
//        mecanumDrive.followTrajectory(forwardTrajectory);
//    }

//    public void back(double inches) {
//        Trajectory forwardTrajectory = mecanumDrive.trajectoryBuilder(new Pose2d())
//                .back(inches)
//                .build();
//        mecanumDrive.followTrajectory(forwardTrajectory);
//    }

//    public void update() {
//        mecanumDrive.update();
//        //Log.v("DriveRoadRunnerMec", "DriveRoadRunnerMec.update() complete.");
//    }

//    public void turnDegrees(double angle) { mecanumDrive.turn(Math.toRadians(angle)); }
//
//    public void turnRadians(double angle) { mecanumDrive.turn(angle); }

    public void setPowerFromGamepad(Gamepad gamepad) {
//        FtcDashboard dash = FtcDashboard.getInstance();
//        List<Action> runningActions = new ArrayList<>();
//
////        double driveSpeedMultiplier = MAX_SPEED * drivePower;
////        if (turtleMode) {
////            driveSpeedMultiplier = MAX_TURTLE_SPEED;
////        }
////        //mecanumDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
////
////        mecanumDrive.setWeightedDrivePower(
////                new Pose2d(
////                        -gamepad.left_stick_y * driveSpeedMultiplier,
////                        -gamepad.left_stick_x * driveSpeedMultiplier,
////                        -gamepad.right_stick_x * driveSpeedMultiplier
////                )
////        );
//
//        // update running actions
//        List<Action> newActions = new ArrayList<>();
//        for (Action action : runningActions) {
//            action.preview(packet.fieldOverlay());
//            if (action.run(packet)) {
//                newActions.add(action);
//            }
//        }
//        runningActions = newActions;

    }

//    public Pose2d getPoseEstimate() {
//        return mecanumDrive.getPoseEstimate();
//    }

    public void stop() {
//        mecanumDrive.setDrivePower(new Pose2d());
//        mecanumDrive.setMotorPowers(0,0, 0, 0 );
    }
}
