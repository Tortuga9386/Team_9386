package org.firstinspires.ftc.teamcode.opmodes;


import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes.FiducialResult;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.List;


@Autonomous(name="Redside_auto", group="autonomous")
public class Redside_Auton extends RobotBase
{

    private   ElapsedTime   runtime = new ElapsedTime();

    public Redside_Auton() {}

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
        indexer.indexerSystem.doIndexerStuff(gamepad2);
        indexer.indexerSystem.colorSensorStuff();
    }

    protected void intake_loop(){
        intake.intakeRoller.doIntakeStuff(gamepad2);
    }

    protected void shooter_loop(){
        shooter.shooterMotor.doShooterStuff(gamepad2);
    }

    protected void turret_loop(){
        turret.turretMotor.doTurretBStuff(gamepad2);
    }

    protected void auto_loop(){
        auto_Paths.redside_auto_path_v1();
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
