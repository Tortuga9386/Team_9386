package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.opmodes.RobotBase;

public class Indexer {
    protected HardwareMap hardwareMap;
    protected Telemetry telemetry;
    protected RobotBase robotBase;
    public IndexerMotor indexerMotor;

    public Indexer(HardwareMap hardwareMap, RobotBase opMode) {
        this.hardwareMap = hardwareMap;
        this.robotBase = opMode;
        this.telemetry = robotBase.telemetry;

        initHardware();
    }

    public Indexer(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
    }

    protected void initHardware() {
        indexerMotor = new IndexerMotor();

    }

    public class IndexerMotor {

        public DcMotor indexerMotor;

        public IndexerMotor() { //HardwareMap hardwareMap, RobotBase opMode
            initHardware();
        }

        public double indexerPower = 0;
        public boolean triggerRollerForward = false;


        protected void initHardware() {
            indexerMotor = hardwareMap.get(DcMotor.class, "TriggerRoller");
            indexerMotor.setDirection(DcMotor.Direction.FORWARD);
            indexerMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        public void doIndexerStuff(Gamepad gamepad2) {
            goToTarget(indexerPower);
            if (triggerRollerForward) {
                indexerPower = 1;
            }
            else {
                indexerPower = 0;
            }

        }

        public void goToTarget(double indexerPower) {
                indexerMotor.setPower(indexerPower);
            }
        }

    }

