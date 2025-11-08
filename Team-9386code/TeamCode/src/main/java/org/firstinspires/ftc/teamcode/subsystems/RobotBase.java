package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import java.util.Locale;

/**
 * This is NOT an opmode.
 *
 * This class initiates all the specific hardware for a robot.
 * The various subsystems classes are located in the subsystem folder.
 * This code is used by all the other opmodes.
 *
 */
@Disabled
public class RobotBase extends OpMode
{
    //Define constants that should NOT be adjusted by manual calibration
    protected boolean           INITIALIZE_IMU      = true;
    protected boolean           INITIALIZE_DRIVE    = true;

    //Make subsystems available to all class extensions
    public Drive drive;
    public Shooter shooter;
    public Indexer indexer;
    public Intake intake;
    public Turret turret;
    public Auto_paths auto_Paths;
    //public Lift lift;
    //public SensorHuskyLens sensorHuskyLens;

    /* Constructor */
    public RobotBase(){ }

    /* Initialize standard Hardware interfaces */
    @Override
    public void init() {

        //Initialize subsystems
       // lift = new Lift(hardwareMap, this);
        //sensorHuskyLens = new SensorHuskyLens(hardwareMap, this);
        //Initialize system
        drive = new Drive(hardwareMap, this);
        shooter = new Shooter(hardwareMap, this);
        intake = new Intake(hardwareMap, this);
        indexer = new Indexer(hardwareMap, this);
        turret = new Turret(hardwareMap, this);
        auto_Paths = new Auto_paths(hardwareMap, this);
    }

    @Override
    public void init_loop() {
    }

    @Override
    public void start() {
        telemetry.clearAll();
    }

    public void loop() {
        //Do nothing, use these classes in the opModes
    }

    /*
     * Code to run when the op mode is first disabled goes here
     */
    @Override
    public void stop() {

    }

    String formatDegrees(double degrees){
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }
    
 }
