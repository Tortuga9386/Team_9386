package org.firstinspires.ftc.teamcode.opmodes;


import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes.FiducialResult;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.List;


@Autonomous(name="Blueside_auto_longshot", group="autonomous")
public class Blueside_Auton_2 extends RobotBase
{

    private   ElapsedTime   runtime = new ElapsedTime();

    public Blueside_Auton_2() {}

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {

        super.INITIALIZE_DRIVE  = true;

        super.init();

        //Set initial positions
        telemetry.addData("Status", "init complete");
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {

    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        runtime.reset();
        indexer.indexerSystem.resetSequenceTimer();
        intake.intakeRoller.resetSequenceTimer();
        shooter.shooterMotor.resetSequenceTimer();
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        auto_loop();
        shooter_loop();
        intake_loop();
        indexer_loop();
        telemetry_loop();
        turret_loop();
        //lift_loop();
        //husky_loop();
    }

    protected void indexer_loop(){
        indexer.indexerSystem.doIndexerStuffAuto();

    }

    protected void intake_loop(){
        intake.intakeRoller.doIntakeStuffAuto();
    }

    protected void shooter_loop(){
        shooter.shooterMotor.autoTestRev2(); }

    protected void turret_loop(){
        turret.turretMotor.TargetSpeedBA();
    }

    protected void auto_loop(){
        if (runtime.seconds() > 17){
            auto_Paths.moveToPos(0,-22,10);
        }
        else {
            auto_Paths.moveToPos(0, 0, 0);
        }
    }


    protected void imu_loop() {

    }

    protected void telemetry_loop() {
        telemetry.addData("Hood Angle", shooter.shooterMotor.hoodAngle);
        telemetry.addData("selection for seq", indexer.indexerSystem.shooterSelection);
        telemetry.addData("selection for selector", indexer.indexerSystem.gamepadSelection);
        telemetry.addData("lifterHeight", indexer.indexerSystem.lifterPos);

        // --- Limelight AprilTag ID Extraction ---
        try {
            LLResult result = turret.limelight.getLatestResult();

            if (result != null && result.isValid()) {
                List<FiducialResult> fiducials = result.getFiducialResults();

                if (fiducials.isEmpty()) {
                    telemetry.addData("Detected AprilTag ID/s", "None");
                } else {
                    // To display all IDs detected
                    StringBuilder ids = new StringBuilder();
                    for (FiducialResult fiducial : fiducials) {
                        // **The key method is getFiducialId()**
                        int aprilTagId = fiducial.getFiducialId();
                        ids.append(aprilTagId).append(", ");
                    }

                    // Remove the trailing ", " if there is at least one ID
                    if (ids.length() > 0) {
                        ids.setLength(ids.length() - 2);
                    }

                    telemetry.addData("Detected AprilTag ID/s", ids.toString());

                    // If you only want the ID of the *first* detected tag:
                    // telemetry.addData("First AprilTag ID", fiducials.get(0).getFiducialId());
                }
            } else {
                telemetry.addData("Detected AprilTag ID/s", "No Valid Result");
            }
        } catch (Exception e) {
            // Handle case where limelight might not be initialized or throws an error
            telemetry.addData("Limelight Error", e.getMessage());
        }
        // --- End Limelight AprilTag ID Extraction ---

        telemetry.update();
    }

}
