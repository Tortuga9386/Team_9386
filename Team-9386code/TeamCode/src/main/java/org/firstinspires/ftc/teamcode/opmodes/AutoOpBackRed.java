package org.firstinspires.ftc.teamcode.opmodes;


import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes.FiducialResult;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.List;


@Autonomous(name="BackRed", group="Autonomous")
public class AutoOpBackRed extends RobotBase
{

    private   ElapsedTime   runtime = new ElapsedTime();

    public AutoOpBackRed() {}

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
        shooter.shooterMotor.shooterTime.reset();
        turret.turretMotor.turretTimer.reset();
        indexer.indexerSystem.indexerTimer.reset();
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        drive_loop();
        shooter_loop();
        intake_loop();
        indexer_loop();
        telemetry_loop();
        turret_loop();
    }

    protected void indexer_loop(){
        indexer.indexerSystem.doIndexerStuff(gamepad2);
        indexer.indexerSystem.colorSensorStuff();
    }

    protected void intake_loop(){
        //intake.intakeRoller.doIntakeStuff();
    }

    protected void shooter_loop(){
        shooter.shooterMotor.Backshot();
    }

    protected void drive_loop() {
        if (runtime.seconds() < 5) {
            drive.moveToPos(0, 0, 0);
        }

//        if (runtime.seconds() > 5){
//            drive.moveToPos(30,12, -90);
//        }
    }

    protected void turret_loop(){
        turret.turretMotor.doTurretStuffRA();
    }





    protected void telemetry_loop() {
        telemetry.addData("MOTOR RF", drive.rightFrontMotor.getPower());
        telemetry.addData("MOTOR LF", drive.leftFrontMotor.getPower());
        telemetry.addData("MOTOR RR", drive.rightRearMotor.getPower());
        telemetry.addData("MOTOR LR", drive.leftRearMotor.getPower());


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
