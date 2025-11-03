package org.firstinspires.ftc.teamcode.opmodes;

import android.util.Log;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.opmodes.MainAutonOp.State;

//@Disabled
public class MainAutonOp extends RobotBase {

    String pathChoice = "";

    //Define the states the robot will step through as it moves across the field
    public enum State {
        BEGIN,
        DO_SOMETHING,
        DRIVE_TO_PARK,
        PARK,
        IDLE
    }
    public State currentState = State.BEGIN;

    //Timers
    private final ElapsedTime time = new ElapsedTime();
    private long iterationCounter = 0;
    private long idleIterationCounter = 10;
    double elapsedTime = 0;


    /**
     * Constructor
     */
    public MainAutonOp() {
    }

    @Override
    public void init() {
        super.INITIALIZE_IMU    = false;

        super.init();

        telemetry.addData("State: ", "READY");
        iterationCounter = 1;
    }

    @Override
    public void init_loop() {
        super.init_loop();

        boolean allTestsPassed = true;

        if (allTestsPassed == false) {
            telemetry.addData("SYSTEM TEST:", "FAIL");
        } else {
            telemetry.addData("SYSTEM TEST:", "PASS");
        }
        telemetry.update();
    }

    @Override
    public void loop() {

        elapsedTime = time.seconds();
        telemetry.setAutoClear(true);
        State new_state;

        iterationCounter += 1;

        if (currentState == State.IDLE) {
            idleIterationCounter -= 1;
            if (idleIterationCounter > 0) {
                Log.d("MainAutonOp", "" + iterationCounter + "> State: IDLE log iteration (" + idleIterationCounter + ")");
            } else if (idleIterationCounter == 0) {
                Log.d("MainAutonOp", "" + iterationCounter + "> State: IDLE log iteration STOP");
            }
        } else {
            Log.d("MainAutonOp", "" + iterationCounter + "> State: pre " + currentState);
            new_state = handleState(currentState);
            if (new_state != currentState) {
                time.reset();
                currentState = new_state;
            }
            Log.d("MainAutonOp", "" + iterationCounter + "> New state: " + currentState);
        }

        telemetry.update();
    }

    protected State handleState(State state) {

        Log.i("MainAutonOp", "--- Executing handleState("+state.toString()+") ---");
        switch (state) {
            case BEGIN:
                return State.DO_SOMETHING;

            case DO_SOMETHING:

//                autoPath.targetLabel = "SPIKE";
//                String spikePathNameAsString = startPosition.toString() +"_SPIKE";
//                pathName = AutoPath.PathName.valueOf(spikePathNameAsString);
//                autoPath.getAutoPaths(pathName);
//
                return State.DRIVE_TO_PARK;


            case DRIVE_TO_PARK:
//                if (!drive.isBusy()) {
//                    try {
//                        Log.d("MainAutonOp", "Path: PARK followTrajectory(autoPath.fourthPosition)");
//                        drive.followTrajectoryAsync(autoPath.fourthPosition);
//                        Log.d("MainAutonOp", "Path: PARK followTrajectory(autoPath.fourthPosition)");
//                    } catch (Exception e) {
//                        Log.e("MainAutonOp", "Error running " + pathName.toString() + " fourthPosition");
//                        Log.e("MainAutonOp", "Exception Message: " + e);
//                        return IDLE;
//                    }
                    return State.PARK;
//                }
                //Log.d("MainAutonOp", "Drive is busy (" + time.seconds()+ ")");
                //break;

            case PARK:

                break;

            case IDLE:
            default:
                break;
        }
        return state;
    }

    @Override
    public void stop() {
        super.stop();
    }

}