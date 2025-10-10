package org.firstinspires.ftc.teamcode.Auto;

//RR-specific imports

// Non-RR imports
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.MecanumDrive;

// Don't edit!!!!!!!!!!!!!!!!!!!!!!
public abstract class redAuto extends LinearOpMode {
    public static class DcMotorA {
        private DcMotorEx DcMotorA;

        public DcMotorA(HardwareMap hardwareMap) {
            DcMotorA = hardwareMap.get(DcMotorEx.class, "Drive1");
        }
    }

    public class DcMotorB {
        private DcMotorEx DcMotorB;

        public DcMotorB(HardwareMap hardwareMap) {
            DcMotorB = hardwareMap.get(DcMotorEx.class, "Drive2");
        }
    }

    public class DcMotorC {
        private DcMotorEx DcMotorC;

        public DcMotorC(HardwareMap hardwareMap) {
            DcMotorC = hardwareMap.get(DcMotorEx.class, "Drive3");
        }
    }

    public class DcMotorD {
        private DcMotorEx DcMotorD;

        public DcMotorD(HardwareMap hardwareMap) {
            DcMotorD = hardwareMap.get(DcMotorEx.class, "Drive4");
        }
    }

    public void runOpMode() {
        waitForStart();
        // instantiate your MecanumDrive at a particular pose.
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(11.8, 61.7, Math.toRadians(90)));
        Action trajectoryAction1;
        trajectoryAction1 = drive.actionBuilder(drive.pose)
                .splineTo(new Vector2d(24.79, -44.75), Math.toRadians(52.24))
                .splineTo(new Vector2d(38.52, -28.44), Math.toRadians(49.90))
                .splineTo(new Vector2d(41.10, -0.54), Math.toRadians(84.73))
                .splineTo(new Vector2d(39.81, 23.07), Math.toRadians(93.12))
                .splineTo(new Vector2d(14.06, 48.61), Math.toRadians(135.24))
                .splineTo(new Vector2d(-38.09, 42.38), Math.toRadians(186.81))
                .splineTo(new Vector2d(-54.83, 15.34), Math.toRadians(238.24))
                .splineTo(new Vector2d(-47.11, -35.73), Math.toRadians(-81.40))
                .splineTo(new Vector2d(-28.65, -56.12), Math.toRadians(-47.85))
                .splineTo(new Vector2d(9.98, -47.32), Math.toRadians(12.83))
                .build();

    }
}
