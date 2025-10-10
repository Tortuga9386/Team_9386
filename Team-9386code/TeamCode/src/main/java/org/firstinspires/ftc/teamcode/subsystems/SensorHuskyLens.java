package org.firstinspires.ftc.teamcode.subsystems;

import android.util.Log;

import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.internal.system.Deadline;
import org.firstinspires.ftc.teamcode.opmodes.RobotBase;

import java.util.concurrent.TimeUnit;

public class SensorHuskyLens {
    protected HardwareMap hardwareMap;
    protected Telemetry telemetry;
    protected RobotBase robotBase;
    private HuskyLens huskyLens;
    private final int READ_PERIOD = 1;
    Deadline rateLimit = new Deadline(READ_PERIOD, TimeUnit.SECONDS);

    public SensorHuskyLens(HardwareMap hardwareMap, RobotBase opMode) {
        this.hardwareMap = hardwareMap;
        this.robotBase = opMode;
        this.telemetry = robotBase.telemetry;

        initHardware();
    }

    protected void initHardware() {
        huskyLens = hardwareMap.get(HuskyLens.class, "huskylens");
        setAlgorithm("COLOR");
        rateLimit.expire();
        telemetry.update();

    }

    public void setAlgorithm (String mode) {
        switch (mode) {
            case "TAG":
                huskyLens.selectAlgorithm(HuskyLens.Algorithm.TAG_RECOGNITION);
                break;
            case "COLOR":
                huskyLens.selectAlgorithm(HuskyLens.Algorithm.COLOR_RECOGNITION);
                break;
        }
    }

    public void getBlocks() {
        if (!rateLimit.hasExpired()) {
            //return;
        }
        rateLimit.reset();

        HuskyLens.Block[] blocks = huskyLens.blocks();
        telemetry.addData("Block count", blocks.length);
        for (int i = 0; i < blocks.length; i++) {
            telemetry.addData("Block", blocks[i].toString());
            /*
             * Here inside the FOR loop, you could save or evaluate specific info for the currently recognized Bounding Box:
             * - blocks[i].width and blocks[i].height   (size of box, in pixels)
             * - blocks[i].left and blocks[i].top       (edges of box)
             * - blocks[i].x and blocks[i].y            (center location)
             * - blocks[i].id                           (Color ID)
             *
             * These values have Java type int (integer).
             */
            telemetry.addData("Center of box X:", blocks[i].x);
            telemetry.addData("Center of box Y:", blocks[i].y);
        }

        telemetry.update();
    }

    public void getColorBlocks() {

        HuskyLens.Block[] blocks = huskyLens.blocks();
        telemetry.addData("Block count", blocks.length);
        for (int i = 0; i < blocks.length; i++) {
            if (blocks[i].height < 100) {
                Log.d("HuskyLense", "small box fail height> " + blocks[i].id);
                continue;
            }
            if (blocks[i].width < 100) {
                Log.d("HuskyLense", "small box fail width> " + blocks[i].id);
                continue;
            }
            Log.d("HuskyLense", "Processing large box> " + blocks[i].id);

            telemetry.addData("Block", blocks[i].toString());
            /*
             * Here inside the FOR loop, you could save or evaluate specific info for the currently recognized Bounding Box:
             * - blocks[i].width and blocks[i].height   (size of box, in pixels)
             * - blocks[i].left and blocks[i].top       (edges of box)
             * - blocks[i].x and blocks[i].y            (center location)
             * - blocks[i].id                           (Color ID)
             *
             * These values have Java type int (integer).
             */
            telemetry.addData("Center of box X:", blocks[i].x);
            telemetry.addData("Center of box Y:", blocks[i].y);
        }

        telemetry.update();
    }

}





