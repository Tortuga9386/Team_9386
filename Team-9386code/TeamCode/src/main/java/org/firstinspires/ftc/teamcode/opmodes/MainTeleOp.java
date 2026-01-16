package org.firstinspires.ftc.teamcode.opmodes;


import static android.graphics.Color.RED;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes.FiducialResult;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;
import java.util.List;


import org.firstinspires.ftc.teamcode.subsystems.Control_center;
import org.firstinspires.ftc.teamcode.subsystems.Indexer;


@TeleOp(name="***TeleOp***", group="teleop")
public class MainTeleOp extends RobotBase
{

    private   ElapsedTime   runtime = new ElapsedTime();

    public MainTeleOp() {}

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
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        control_center_tele();
        telemetry_loop();
    }

    protected void control_center_tele(){
        control_center.control_center1.teleop(gamepad1,gamepad2, true);
    }

    protected void telemetry_loop() {
        telemetry.addData("Hood Angle", shooter.shooterMotor.hoodAngle);
        telemetry.addData("LeftLifterHeight", indexer.indexerSystem.rightLifterMotor.getCurrentPosition());
        telemetry.addData("LeftLifterHeight", indexer.indexerSystem.leftLifterMotor.getCurrentPosition());

        telemetry.addData("RPM", (shooter.shooterMotor.shooterMotor.getVelocity() * 60) /28);
        telemetry.addData("TPS", shooter.shooterMotor.shooterMotor.getVelocity());

        telemetry.addData("Runtime",control_center.control_center1.runtime);
        telemetry.addData("lifterTimer1",control_center.control_center1.lifterTimer1);
        telemetry.addData("lifterTimer2",control_center.control_center1.lifterTimer2);


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
