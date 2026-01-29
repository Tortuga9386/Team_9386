package org.firstinspires.ftc.teamcode.opmodes;


import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes.FiducialResult;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.List;


@Autonomous(name="RedBack", group="auto")
public class RedBack extends RobotBase
{

    private   ElapsedTime   runtime = new ElapsedTime();

    public RedBack() {}

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {

        super.INITIALIZE_DRIVE  = true;

        super.init();

        //Set initial positions
        telemetry.addData("Status", "init complete");

        control_center.control_center1.rightLifterPos = 1;
        control_center.control_center1.leftLifterPos = 1;
        drive.limelight3A.pipelineSwitch(0);
        drive.limeLightOffset = -3.5;
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {

        control_center.control_center1.rightLifterPos = 1;
        control_center.control_center1.leftLifterPos = 1;

        drive.limelight3A.pipelineSwitch(0);

        control_center.control_center1.runLifters();
        try {
            LLResult result = drive.limelight3A.getLatestResult();

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
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        runtime.reset();
        control_center.control_center1.runtime.reset();
        drive.time.reset();
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        control_center_auto();
        telemetry_loop();
        drive_loop();
    }

    protected void control_center_auto(){
        control_center.control_center1.Auto_op(true);
        control_center.control_center1.runLifters();
    }

    protected void drive_loop(){
        drive.backShotRed();
    }

    protected void telemetry_loop() {
        telemetry.addData("LeftLifterHeight", indexer.indexerSystem.rightLifterMotor.getCurrentPosition());
        telemetry.addData("LeftLifterHeight", indexer.indexerSystem.leftLifterMotor.getCurrentPosition());

        telemetry.addData("RPM", (shooter.shooterMotor.shooterMotor.getVelocity() * 60) /28);
        telemetry.addData("TPS", shooter.shooterMotor.shooterMotor.getVelocity());

        telemetry.addData("Runtime",control_center.control_center1.runtime);


        // --- Limelight AprilTag ID Extraction ---
        try {
            LLResult result = drive.limelight3A.getLatestResult();

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
