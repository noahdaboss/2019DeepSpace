package frc.robot;

import edu.wpi.first.wpilibj.Solenoid;

import frc.lib.util.ConstantsBase;
import frc.lib.util.InterpolatingDouble;
import frc.lib.util.InterpolatingTreeMap;
import frc.lib.util.math.PolynomialRegression;

import java.net.NetworkInterface;
import java.net.SocketException;
import java.util.Enumeration;
import java.util.HashMap;

/**
 * A list of constants used by the rest of the robot code. This include physics constants as well as constants
 * determined through calibrations.
 */
public class Constants extends ConstantsBase {
    public static double kLooperDt = 0.005;

    
    /* ROBOT PHYSICAL CONSTANTS */

    // Wheels
    public static double kDriveWheelDiameterInches = 3.419;
    public static double kTrackWidthInches = 26.655;
    public static double kTrackScrubFactor = 0.924;

    // Geometry
    public static double kCenterToFrontBumperDistance = 16.33;
    public static double kCenterToIntakeDistance = 23.11;
    public static double kCenterToRearBumperDistance = 16.99;
    public static double kCenterToSideBumperDistance = 17.225;

    
    
    /* CONTROL LOOP GAINS */

    // PID gains for drive velocity loop (HIGH GEAR)
    // Units: setpoint, error, and output are in inches per second.
    public static double kDriveHighGearVelocityKp = 1.2;
    public static double kDriveHighGearVelocityKi = 0.0;
    public static double kDriveHighGearVelocityKd = 6.0;
    public static double kDriveHighGearVelocityKf = .15;
    public static int kDriveHighGearVelocityIZone = 0;
    public static double kDriveHighGearVelocityRampRate = 240.0;
    public static double kDriveHighGearNominalOutput = 0.5;
    public static double kDriveHighGearMaxSetpoint = 17.0 * 12.0; // 17 fps

    // PID gains for drive velocity loop (LOW GEAR)
    // Units: setpoint, error, and output are in inches per second.
    public static double kDriveLowGearPositionKp = 1.0;
    public static double kDriveLowGearPositionKi = 0.002;
    public static double kDriveLowGearPositionKd = 100.0;
    public static double kDriveLowGearPositionKf = .45;
    public static int kDriveLowGearPositionIZone = 700;
    public static double kDriveLowGearPositionRampRate = 240.0; // V/s
    public static double kDriveLowGearNominalOutput = 0.5; // V
    public static double kDriveLowGearMaxVelocity = 6.0 * 12.0 * 60.0 / (Math.PI * kDriveWheelDiameterInches); // 6 fps
                                                                                                               // in RPM
    public static double kDriveLowGearMaxAccel = 18.0 * 12.0 * 60.0 / (Math.PI * kDriveWheelDiameterInches); // 18 fps/s
                                                                                                             // in RPM/s

    public static double kDriveVoltageCompensationRampRate = 0.0;

    // Turn to heading gains
    public static double kDriveTurnKp = 3.0;
    public static double kDriveTurnKi = 1.5;
    public static double kDriveTurnKv = 0.0;
    public static double kDriveTurnKffv = 1.0;
    public static double kDriveTurnKffa = 0.0;
    public static double kDriveTurnMaxVel = 360.0;
    public static double kDriveTurnMaxAcc = 720.0;

  
    // Do not change anything after this line unless you rewire the robot and
    // update the spreadsheet!
    // Port assignments should match up with the spreadsheet here:
    // https://docs.google.com/spreadsheets/d/16Yat5u_Bs30ocKRlP4vNz9BPh5b0n9Pk_RTLPKM5GqE/edit#gid=1231815488

    // Drive ports
    public static final int kLeftDriveDIOpwmPort = 0;
    public static final int kLeftDriveEncoderAPort = 1;
    public static final int kLeftDriveEncoderBPort = 2;
    public static final int kLeftDriveDirectionpwmPort = 0;
    
    public static final int kRightDriveDIOpwmPort = 3;
    public static final int kRightDriveEncoderAPort = 4;
    public static final int kRightDriveEncoderBPort = 5;
    public static final int kRightDriveDirectionpwmPort = 1;
    
    public static final int kDriveEnablepwmPort = 2;
    
    //FEEDER ------------------------------------------------------
    public static final int kFeederTriggerOneMXPPort = 10; //trigger motor port
    public static final int kFeederTriggerTwoMXPPort = 12;
    
    public static final int kFeederEnablepwmPort = 5;
    
    public static final int kFeederIRInitialPort = 0;  //ports for IR's
    public static final int kFeederIRFinalPort = 1;
    
    //Threshold values for ir: (in volts i think)
    public static final double kFeederIRInitialMax = 4.0;
    public static final double kFeederIRInitialMin = 2.0;
    
    public static final double kFeederIRFinalMax = 4.0;
    public static final double kFeederIRFinalMin = 2.0;
    
    //Feeder Constants
    public static final double kFeederUnjamPeriod = .4;
    public static final double kFeederUnjamPower = -.75;
    public static final double kFeederClogPeriod = 2;
    public static final double kFeederIncrementFeedPower = 1.0;
    public static final double kFeederContinuousFeedPower = 1.0;
    
    
    
    
    
    
    //INTAKE ------------------------------------------------------------
    public static final int kIntakeRollerMXPPort = 14 ;  //roller motor port
    public static final int kIntakeRollerEnable = 6;
    
    
    public static final int kIntakeIRHopperPort = 0;  //hopper full IR port

  //Threshold values for ir: (in volts i think)
    public static final double kIntakeRollerMax = 4.0;
    public static final double kIntakeRollerMin = 2.0;
    
    //Intake Constants
    public static final double kIntakeUnjamPeriod = .4;
    public static final double kIntakeUnjamPower = -.25;
    public static final double kIntakePower = .5;
   
    public static final double kHopperSensePeriod=1;
    
    
    
    
    //SHOOTER --------------------------------------------------------------
    public static final int kShooterDIOpwmPort = 3;
    public static final int kShooterEncoderAPort = 4;
    public static final int kShooterEncoderBPort = 5;
    public static final int kShooterDirectionpwmPort = 1;
    
    public static final int kShooterEnablepwmPort = 2;
    
    public static final double kFlywheelOnTargetTolerance = 5;
    
    
    
    
    

   

    // Path following constants
    public static double kMinLookAhead = 12.0; // inches
    public static double kMinLookAheadSpeed = 9.0; // inches per second
    public static double kMaxLookAhead = 24.0; // inches
    public static double kMaxLookAheadSpeed = 120.0; // inches per second
    public static double kDeltaLookAhead = kMaxLookAhead - kMinLookAhead;
    public static double kDeltaLookAheadSpeed = kMaxLookAheadSpeed - kMinLookAheadSpeed;

    public static double kInertiaSteeringGain = 0.0; // angular velocity command is multiplied by this gain *
                                                     // our speed
                                                     // in inches per sec
    public static double kSegmentCompletionTolerance = 0.1; // inches
    public static double kPathFollowingMaxAccel = 120.0; // inches per second^2
    public static double kPathFollowingMaxVel = 120.0; // inches per second
    public static double kPathFollowingProfileKp = 5.00;
    public static double kPathFollowingProfileKi = 0.03;
    public static double kPathFollowingProfileKv = 0.02;
    public static double kPathFollowingProfileKffv = 1.0;
    public static double kPathFollowingProfileKffa = 0.05;
    public static double kPathFollowingGoalPosTolerance = 0.75;
    public static double kPathFollowingGoalVelTolerance = 12.0;
    public static double kPathStopSteeringDistance = 9.0;



    @Override
    public String getFileLocation() {
        return "~/constants.txt";
    }

   
}
