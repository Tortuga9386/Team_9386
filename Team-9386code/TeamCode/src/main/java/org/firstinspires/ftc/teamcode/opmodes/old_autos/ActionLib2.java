package org.firstinspires.ftc.teamcode.opmodes.old_autos;

import android.util.Log;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class ActionLib2 {

    public ActionLib2() {

    }

    static int motorPrecision = 10;
    static double servoPrecision = 0.001;

//////////////////////////////////////////////////////////////////////////
///  ROBOT LIFT
//////////////////////////////////////////////////////////////////////////

    public static class RobotLift {
        private DcMotor lift;
        private boolean quitter = true;
        private double liftMotorSpeed = 1;
        private int downTarget = 0;
        private int downTargetPosition = 0;

        public RobotLift(HardwareMap hardwareMap) {
            lift = hardwareMap.get(DcMotor.class, "lift");
            lift.setDirection(DcMotor.Direction.FORWARD);
            lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        public void disableLiftHold() {
            quitter = true;
        }

        public void goToTarget(int targetPosition, double motorPower) {
            double slideCurrentPosition = lift.getCurrentPosition();
            if (true) {
                lift.setTargetPosition(targetPosition);
                lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                lift.setPower(1);
            }
        }

        public class ActionLiftUp implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {

                if (!initialized) {
                    lift.setPower(0.5);
                    initialized = true;
                }

                double pos = lift.getCurrentPosition();
                packet.put("liftPos", pos);
                if (pos < 3000.0) {
                    return true;
                } else {
                    lift.setPower(0);
                    return false;
                }
            }
        }

        public Action actionLiftUp() {
            return new ActionLiftUp();
        }

        /////LIFT DOWN
        public class ActionLiftDown implements Action {
            private boolean initialized = false;

            public ActionLiftDown(int liftPosition) {
                Log.v("ActionLiftDown", "START//liftPosition: "+liftPosition+"/////////////////////////////////////////////////");
                downTargetPosition = downTarget;
                if (liftPosition > 0) {
                    downTargetPosition = liftPosition;
                }
            }

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                //int targetPosition = 0;

                double currentPosition = lift.getCurrentPosition();
                packet.put("liftPos", currentPosition);
                if ((Math.abs(downTargetPosition - currentPosition) > motorPrecision) || !quitter) {
                    goToTarget(downTargetPosition, liftMotorSpeed);
                    return true;
                } else {
                    if (quitter) {
                        quitter = true;
                        return false;
                    }
                    return true;
                }
            }
        }

        public Action actionLiftDown(int liftPosition) {
            return new ActionLiftDown(liftPosition);
        }
        public Action actionLiftDown() {
            return new ActionLiftDown(0);
        }

        /////LIFT SPECIMEN
        public class ActionLiftSpecimen implements Action {

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                Log.v("ActionLiftSpecimen", "START/////////////////////////////////////////////////////////////////");
                int targetPosition = 2150;

                double currentPosition = lift.getCurrentPosition();
                packet.put("liftPos", currentPosition);
                if ((Math.abs(targetPosition - currentPosition) > motorPrecision) || !quitter) {
                    Log.v("ActionLiftSpecimen", "RUNNING//////targetPosition" + targetPosition + " vs currentPosition" + currentPosition + "////////////////////////////////////////");
                    Log.v("ActionLiftSpecimen", "RUNNING//////testVal:" + Math.abs(targetPosition - currentPosition) + " > 50///////////////////////////////////////");
                    goToTarget(targetPosition, liftMotorSpeed);
                    return true;
                } else {
                    if (quitter) {
                        quitter = true;
                        Log.v("ActionLiftSpecimen", "FINISH//////I QUIT!!!!////////////////////////////////////////");
                        Log.v("ActionLiftSpecimen", "FINISH//////testVal:" + Math.abs(targetPosition - currentPosition) + " < 50///////////////////////////////////////");
                        Log.v("ActionLiftSpecimen", "FINISH//////////////////////////////////////////////////////////////////");
                        return false;
                    }
                    Log.v("ActionLiftSpecimen", "FINISH//////ON TARGET BUT NOT A QUITTER!////////////////////////////////////////");
                    Log.v("ActionLiftSpecimen", "FINISH//////testVal:" + Math.abs(targetPosition - currentPosition) + " < 50///////////////////////////////////////");
                    Log.v("ActionLiftSpecimen", "FINISH//////////////////////////////////////////////////////////////////");
                    return true;
                }
            }
        }

        public Action actionLiftSpecimen() {
            return new ActionLiftSpecimen();
        }


        /////LIFT SPECIMEN
        public class ActionLiftScore implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                int targetPosition = 1600;

                double currentPosition = lift.getCurrentPosition();
                packet.put("liftPos", currentPosition);
                if ((Math.abs(targetPosition - currentPosition) > motorPrecision) || !quitter) {
                    goToTarget(targetPosition, 0.75);
                    return true;
                } else {
                    if (quitter) {
                        quitter = true;
                        return false;
                    }
                    return true;
                }
            }
        }

        public Action actionliftScore() {
            return new ActionLiftScore();
        }

        /////CLAW GRAB
        public class ActionClawGrab implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                Log.v("ActionClawGrab", "START/////////////////////////////////////////////////////////////////");
                int targetPosition = 210;//nick dropped this 55 ticks

                double currentPosition = lift.getCurrentPosition();
                packet.put("liftPos", currentPosition);
                if ((Math.abs(targetPosition - currentPosition) > motorPrecision) || !quitter) {
                    goToTarget(targetPosition, liftMotorSpeed);
                    Log.v("ActionClawGrab", "RUNNING//////targetPosition" + targetPosition + " vs currentPosition" + currentPosition + "////////////////////////////////////////");
                    Log.v("ActionClawGrab", "RUNNING//////testVal:" + Math.abs(targetPosition - currentPosition) + " > 50///////////////////////////////////////");
                    return true;
                } else {
                    if (quitter) {
                        quitter = true;
                        Log.v("ActionClawGrab", "FINISH//////I QUIT!!!!////////////////////////////////////////");
                        Log.v("ActionClawGrab", "FINISH//////testVal:" + Math.abs(targetPosition - currentPosition) + " < 50///////////////////////////////////////");
                        Log.v("ActionClawGrab", "FINISH//////////////////////////////////////////////////////////////////");
                        return false;
                    } else {
                        Log.v("ActionClawGrab", "FINISH//////ON TARGET BUT NOT A QUITTER!////////////////////////////////////////");
                        Log.v("ActionClawGrab", "FINISH//////testVal:" + Math.abs(targetPosition - currentPosition) + " < 50///////////////////////////////////////");
                        Log.v("ActionClawGrab", "FINISH//////////////////////////////////////////////////////////////////");
                    }
                    return true;
                }
            }
        }

        public Action actionClawGrab() {
            return new ActionClawGrab();
        }

        /////LIFT TO BASKET
        public class ActionLiftToBasket implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                Log.v("ActionLiftToBasket", "START/////////////////////////////////////////////////////////////////");
                int targetPosition = 4100;

                double currentPosition = lift.getCurrentPosition();
                packet.put("liftPos", currentPosition);
                if ((Math.abs(targetPosition - currentPosition) > motorPrecision) || !quitter) {
                    goToTarget(targetPosition, liftMotorSpeed);
                    Log.v("ActionLiftToBasket", "RUNNING//////targetPosition" + targetPosition + " vs currentPosition" + currentPosition + "////////////////////////////////////////");
                    Log.v("ActionLiftToBasket", "RUNNING//////testVal:" + Math.abs(targetPosition - currentPosition) + " > 50///////////////////////////////////////");
                    return true;
                } else {
                    if (quitter) {
                        quitter = true;
                        Log.v("ActionLiftToBasket", "FINISH//////I QUIT!!!!////////////////////////////////////////");
                        Log.v("ActionLiftToBasket", "FINISH//////testVal:" + Math.abs(targetPosition - currentPosition) + " < 50///////////////////////////////////////");
                        Log.v("ActionLiftToBasket", "FINISH//////////////////////////////////////////////////////////////////");
                        return false;
                    } else {
                        Log.v("ActionLiftToBasket", "FINISH//////ON TARGET BUT NOT A QUITTER!////////////////////////////////////////");
                        Log.v("ActionLiftToBasket", "FINISH//////testVal:" + Math.abs(targetPosition - currentPosition) + " < 50///////////////////////////////////////");
                        Log.v("ActionLiftToBasket", "FINISH//////////////////////////////////////////////////////////////////");
                    }
                    return true;
                }
            }
        }

        public Action actionLiftToBasket() {
            return new ActionLiftToBasket();
        }


        /////CANCEL ANY LIFT HOLD OVERRIDES
        public class ActionQuitLift implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                disableLiftHold();
                return false;
            }
        }

        public Action actionQuitLift() {
            return new ActionQuitLift();
        }

        public class ActionHoldLift implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                quitter = false;
                return false;
            }
        }

        public Action actionHoldLift() {
            return new ActionHoldLift();
        }

    }

//////////////////////////////////////////////////////////////////////////
///  INTAKE SLIDE
//////////////////////////////////////////////////////////////////////////

    public static class RobotIntake {
        private DcMotor intake;
        private boolean quitter = true;
        private double IntakeliftMotorSpeed = 1;
        private int downTarget = 0;
        private int downTargetPosition = 0;

        public RobotIntake(HardwareMap hardwareMap) {
            intake = hardwareMap.get(DcMotor.class, "bridge");
            intake.setDirection(DcMotor.Direction.REVERSE);
            intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            intake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            intake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        public void disableLiftHold() {
            quitter = true;
        }

        public void goToTarget(int targetPosition, double motorPower) {

                intake.setTargetPosition(targetPosition);
                intake.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                intake.setPower(1);

        }

        /////INTAKE DM
        public class ActionIntakeDM implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {

                if (!initialized) {
                    intake.setPower(0.5);
                    initialized = true;
                }

                double pos = intake.getCurrentPosition();
                packet.put("liftPos", pos);
                if (pos < 3000.0) {
                    return true;
                } else {
                    intake.setPower(0);
                    return false;
                }
            }
        }

        public Action actionIntakeDM() {
            return new ActionIntakeDM();
        }

        /////INTAKE MOVE
        public class ActionIntakeMove implements Action {
            private boolean initialized = false;

            public ActionIntakeMove(int liftPosition) {
                Log.v("ActionLiftDown", "START//liftPosition: "+liftPosition+"/////////////////////////////////////////////////");
                downTargetPosition = downTarget;
                if (liftPosition > 0) {
                    downTargetPosition = liftPosition;
                }
            }

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                //int targetPosition = 0;

                double currentPosition = intake.getCurrentPosition();
                packet.put("liftPos", currentPosition);
                if ((Math.abs(downTargetPosition - currentPosition) > motorPrecision) || !quitter) {
                    goToTarget(downTargetPosition, IntakeliftMotorSpeed);
                    return true;
                } else {
                    if (quitter) {
                        quitter = true;
                        return false;
                    }
                    return true;
                }
            }
        }

        public Action actionIntakeMove(int intakePosition) {
            return new ActionIntakeMove(intakePosition);
        }

        /////INTAKE DOWN
        public class ActionIntakeDown implements Action {

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                Log.v("ActionLiftSpecimen", "START/////////////////////////////////////////////////////////////////");
                int targetPosition = 1190;

                double currentPosition = intake.getCurrentPosition();
                packet.put("liftPos", currentPosition);
                if ((Math.abs(targetPosition - currentPosition) > motorPrecision) || !quitter) {
                    Log.v("ActionLiftSpecimen", "RUNNING//////targetPosition" + targetPosition + " vs currentPosition" + currentPosition + "////////////////////////////////////////");
                    Log.v("ActionLiftSpecimen", "RUNNING//////testVal:" + Math.abs(targetPosition - currentPosition) + " > 50///////////////////////////////////////");
                    goToTarget(targetPosition, IntakeliftMotorSpeed);
                    return true;
                } else {
                    if (quitter) {
                        quitter = true;
                        Log.v("ActionLiftSpecimen", "FINISH//////I QUIT!!!!////////////////////////////////////////");
                        Log.v("ActionLiftSpecimen", "FINISH//////testVal:" + Math.abs(targetPosition - currentPosition) + " < 50///////////////////////////////////////");
                        Log.v("ActionLiftSpecimen", "FINISH//////////////////////////////////////////////////////////////////");
                        return false;
                    }
                    Log.v("ActionLiftSpecimen", "FINISH//////ON TARGET BUT NOT A QUITTER!////////////////////////////////////////");
                    Log.v("ActionLiftSpecimen", "FINISH//////testVal:" + Math.abs(targetPosition - currentPosition) + " < 50///////////////////////////////////////");
                    Log.v("ActionLiftSpecimen", "FINISH//////////////////////////////////////////////////////////////////");
                    return true;
                }
            }
        }

        public Action actionIntakeDown() {
            return new ActionIntakeDown
                    ();
        }

        /////INTAKE UP
        public class ActionIntakeUp implements Action {

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                Log.v("ActionLiftSpecimen", "START/////////////////////////////////////////////////////////////////");
                int targetPosition = -1;

                double currentPosition = intake.getCurrentPosition();
                packet.put("liftPos", currentPosition);
                if ((Math.abs(targetPosition - currentPosition) > motorPrecision) || !quitter) {
                    Log.v("ActionLiftSpecimen", "RUNNING//////targetPosition" + targetPosition + " vs currentPosition" + currentPosition + "////////////////////////////////////////");
                    Log.v("ActionLiftSpecimen", "RUNNING//////testVal:" + Math.abs(targetPosition - currentPosition) + " > 50///////////////////////////////////////");
                    goToTarget(targetPosition, IntakeliftMotorSpeed);
                    return true;
                } else {
                    if (quitter) {
                        quitter = true;
                        Log.v("ActionLiftSpecimen", "FINISH//////I QUIT!!!!////////////////////////////////////////");
                        Log.v("ActionLiftSpecimen", "FINISH//////testVal:" + Math.abs(targetPosition - currentPosition) + " < 50///////////////////////////////////////");
                        Log.v("ActionLiftSpecimen", "FINISH//////////////////////////////////////////////////////////////////");
                        return false;
                    }
                    Log.v("ActionLiftSpecimen", "FINISH//////ON TARGET BUT NOT A QUITTER!////////////////////////////////////////");
                    Log.v("ActionLiftSpecimen", "FINISH//////testVal:" + Math.abs(targetPosition - currentPosition) + " < 50///////////////////////////////////////");
                    Log.v("ActionLiftSpecimen", "FINISH//////////////////////////////////////////////////////////////////");
                    return true;
                }
            }
        }

        public Action actionIntakeUp1() {
            return new ActionIntakeUp();
        }

        /////CANCEL ANY LIFT HOLD OVERRIDES
        public class ActionQuitIntake implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                disableLiftHold();
                return false;
            }
        }

        public Action actionQuitLift() {
            return new ActionQuitIntake();
        }

        public class ActionHoldIntake implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                quitter = false;
                return false;
            }
        }

        public Action actionHoldLift() {
            return new ActionHoldIntake();
        }

    }


//////////////////////////////////////////////////////////////////////////
///  INTAKE CLAW
//////////////////////////////////////////////////////////////////////////

    public static class RobotIntakeClaw {
        private Servo intakeClawServo;
        private double openPosition = 0.675;
        private double closedPosition = 0.44;

        private Servo  fingerServo;
        private double fingerInPosition = 0.0;
        private double fingerOutPosition = 1.0;

        public RobotIntakeClaw(HardwareMap hardwareMap) {
            intakeClawServo = hardwareMap.get(Servo.class, "frontclaw");
            intakeClawServo.setDirection(Servo.Direction.FORWARD);
//            fingerServo     = hardwareMap.get(com.qualcomm.robotcore.hardware.Servo.class, "finger");
//            fingerServo.setDirection(Servo.Direction.FORWARD);
        }

        public void clawOpen() {
            intakeClawServo.setPosition(openPosition);
        }

        public void clawClose() {
            intakeClawServo.setPosition(closedPosition);
        }

        public void extendTheFinger()  {
            fingerServo.setPosition(fingerOutPosition);
        }
        public void retractFinger()  {
            fingerServo.setPosition(fingerInPosition);
        }

        public class ActionClawOpen implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                Log.v("ActionClawOpen", "START/////////////////////////////////////////////////////////////////");
                //extendTheFinger();

                if (!initialized) {
                    intakeClawServo.setPosition(openPosition);
                    initialized = true;
                }

                double currentPosition = intakeClawServo.getPosition();
                packet.put("clawPos", currentPosition);
                if (Math.abs(openPosition - currentPosition) > servoPrecision) {
                    intakeClawServo.setPosition(openPosition);
                    return true;
                } else {
                    return false;
                }

            }
        }

        public Action actionClawOpen() {
            return new ActionClawOpen();
        }

        public class ActionClawClose implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {

                //retractFinger();
                if (!initialized) {
                    intakeClawServo.setPosition(closedPosition);
                    initialized = true;
                }

                double currentPosition = intakeClawServo.getPosition();
                packet.put("clawPos", currentPosition);
                if (Math.abs(closedPosition - currentPosition) > servoPrecision) {
                    intakeClawServo.setPosition(closedPosition);
                    return true;
                } else {
                    return false;
                }
            }
        }

        public Action actionClawClose() {
            return new ActionClawClose();
        }

    }

//////////////////////////////////////////////////////////////////////////
///  INTAKE TILTER
//////////////////////////////////////////////////////////////////////////

    public static class RobotIntakeTilter {
        private Servo intakeTilterServo;

        private double tilterUpPos = 0.5;
        private double tilterDownPos = 0.85;

        public RobotIntakeTilter(HardwareMap hardwareMap) {
            intakeTilterServo = hardwareMap.get(Servo.class, "tilter");
            intakeTilterServo.setDirection(Servo.Direction.FORWARD);
        }

        public void tilterDown() {
            intakeTilterServo.setPosition(tilterUpPos);
        }

        public void tilterUp() {
            intakeTilterServo.setPosition(tilterDownPos);
        }


        public class ActionTilterUp implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                Log.v("ActionClawOpen", "START/////////////////////////////////////////////////////////////////");
                //extendTheFinger();

                if (!initialized) {
                    intakeTilterServo.setPosition(tilterUpPos);
                    initialized = true;
                }

                double currentPosition = intakeTilterServo.getPosition();
                packet.put("clawPos", currentPosition);
                if (Math.abs(tilterUpPos - currentPosition) > servoPrecision) {
                    intakeTilterServo.setPosition(tilterUpPos);
                    return true;
                } else {
                    return false;
                }

            }
        }

        public Action actionTilterUp() {
            return new ActionTilterUp();
        }

        public class ActionTilterDown implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {

                //retractFinger();
                if (!initialized) {
                    intakeTilterServo.setPosition(tilterDownPos);
                    initialized = true;
                }

                double currentPosition = intakeTilterServo.getPosition();
                packet.put("clawPos", currentPosition);
                if (Math.abs(tilterDownPos - currentPosition) > servoPrecision) {
                    intakeTilterServo.setPosition(tilterDownPos);
                    return true;
                } else {
                    return false;
                }
            }
        }

        public Action actionTilterDown() {
            return new ActionTilterDown();
        }

    }


//////////////////////////////////////////////////////////////////////////
///  INTAKE LINKAGE
//////////////////////////////////////////////////////////////////////////

    public static class RobotIntakeLinkage {
        private Servo intakeLinkageServo;
        private double inPosition = 0;
        private double outPosition = -0.55;
        private double midPosition = -0.25;

        public RobotIntakeLinkage(HardwareMap hardwareMap) {
            intakeLinkageServo = hardwareMap.get(Servo.class, "linkage");
            intakeLinkageServo.setDirection(Servo.Direction.FORWARD);
        }

        public void linkageOut() {
            intakeLinkageServo.setPosition(outPosition);
        }

        public void linkageIn() {
            intakeLinkageServo.setPosition(inPosition);
        }
        public void linkageMid() {
            intakeLinkageServo.setPosition(midPosition);
        }

        public class ActionLinkageOut implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                Log.v("ActionLinkageOut2", "START/////////////////////////////////////////////////////////////////");

                if (!initialized) {
                    linkageOut();
                    //intakeLinkageServo.setPosition(outPosition);
                    initialized = true;
                }

                double currentPosition = intakeLinkageServo.getPosition();
                packet.put("linkagePos2", currentPosition);
                if (Math.abs(outPosition - currentPosition) > servoPrecision) {
                    //intakeLinkageServo.setPosition(outPosition);
                    linkageOut();
                    Log.v("ActionLinkageOut2", "RUNNING//////targetPosition:"+outPosition+ " vs currentPosition:"+currentPosition+"////////////////////////////////////////");
                    Log.v("ActionLinkageOut2", "RUNNING//////testVal:"+Math.abs(outPosition - currentPosition)+" > "+servoPrecision+"///////////////////////////////////////");
                    return true;
                } else {
                    Log.v("ActionLinkageOut2", "DONE//////testVal:"+Math.abs(outPosition - currentPosition)+" > "+servoPrecision+"///////////////////////////////////////");
                    return false;
                }

            }
        }

        public Action actionLinkageOut() {
            return new ActionLinkageOut();
        }

        public class ActionLinkageIn implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    linkageIn();
                    initialized = true;
                }

                double currentPosition = intakeLinkageServo.getPosition();
                packet.put("linkagePos2", currentPosition);
                if (Math.abs(inPosition - currentPosition) > servoPrecision) {
                    linkageIn();
                    Log.v("ActionLinkageIn2", "RUNNING//////targetPosition"+inPosition+ " vs currentPosition"+currentPosition+"////////////////////////////////////////");
                    Log.v("ActionLinkageIn2", "RUNNING//////testVal:"+Math.abs(inPosition - currentPosition)+" > "+servoPrecision+"///////////////////////////////////////");
                    return true;
                } else {
                    Log.v("ActionLinkageIn2", "DONE//////testVal:"+Math.abs(outPosition - currentPosition)+" > "+servoPrecision+"///////////////////////////////////////");
                    return false;
                }
            }
        }

        public Action actionLinkageIn() {
            return new ActionLinkageIn();
        }

        public class ActionLinkageMid implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    linkageMid();
                    initialized = true;
                }

                double currentPosition = intakeLinkageServo.getPosition();
                packet.put("linkagePos2", currentPosition);
                if (Math.abs(inPosition - currentPosition) > servoPrecision) {
                    linkageMid();
                    Log.v("ActionLinkageIn2", "RUNNING//////targetPosition"+inPosition+ " vs currentPosition"+currentPosition+"////////////////////////////////////////");
                    Log.v("ActionLinkageIn2", "RUNNING//////testVal:"+Math.abs(inPosition - currentPosition)+" > "+servoPrecision+"///////////////////////////////////////");
                    return true;
                } else {
                    Log.v("ActionLinkageIn2", "DONE//////testVal:"+Math.abs(outPosition - currentPosition)+" > "+servoPrecision+"///////////////////////////////////////");
                    return false;
                }
            }
        }

        public Action actionLinkageMid() {
            return new ActionLinkageMid();
        }

    }

}





