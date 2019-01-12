package frc.robot.subsystems;

import frc.robot.Constants;
import frc.robot.loops.Loop;
import frc.robot.loops.Looper;
import com.team6498.lib.util.drivers.IRSensor;
import com.team6498.lib.util.drivers.NidecBrushlessThree;
import edu.wpi.first.wpilibj.NidecBrushless;

/**
 * The feeder subsystem consists of dynamo motors that are meant to send one ball at a time into the shooter.
 * There are ir sensors placed before and after the feeder to sense blockage or emptiness of the hopper.
 * The main things this subsystem has to are feed fuel and unjam
 * 
 *
 */
public class Feeder extends Subsystem {
    
   

    private static Feeder sInstance = null;

    public static Feeder getInstance() {
        if (sInstance == null) {
            sInstance = new Feeder();
        }
        return sInstance;
    }

    private final NidecBrushless mTriggerL;
    private final NidecBrushless mTriggerR;
    private final IRSensor mIRInitial, mIRFinal;
    
    public Feeder() {
        mTriggerL=new NidecBrushless(Constants.kFeederEnablepwmPort, Constants.kFeederTriggerOneMXPPort);
        mTriggerR=new NidecBrushless(Constants.kFeederEnablepwmPort, Constants.kFeederTriggerTwoMXPPort);

        mIRInitial = new IRSensor(Constants.kFeederIRInitialPort, Constants.kFeederIRInitialMin, Constants.kFeederIRInitialMax);
        mIRFinal = new IRSensor(Constants.kFeederIRFinalPort, Constants.kFeederIRFinalMin, Constants.kFeederIRFinalMax);
       
    }

    public enum SystemState {
        CONTINUOUS_FEEDING, // feed balls into the shooter at full speed
        INCREMENTAL_FEEDING, //feed a ball at a time
        UNJAMMING, //Reverses motors continuously
        IDLE, // stop all motors
    }

    public enum WantedState {
        IDLE,
        UNJAM,
        CONTINUOUS_FEED,
        INCREMENT_FEED
    }

    private SystemState mSystemState = SystemState.IDLE;
    private WantedState mWantedState = WantedState.IDLE;

    private double mCurrentStateStartTime;
    private boolean mStateChanged;

    private Loop mLoop = new Loop() {
        @Override
        public void onStart(double timestamp) {
            stop();
            synchronized (Feeder.this) {
                mSystemState = SystemState.IDLE;
                mStateChanged = true;
                mCurrentStateStartTime = timestamp;
            }
        }

        @Override
        public void onLoop(double timestamp) {
            synchronized (Feeder.this) {
                SystemState newState;
                switch (mSystemState) {
                case IDLE:
                    newState = handleIdle();
                    break;
                case UNJAMMING:
                    newState = handleUnjamming(timestamp, mCurrentStateStartTime);
                    break;                
                case CONTINUOUS_FEEDING:
                    newState = handleContinuousFeeding();
                    break;  
                case INCREMENTAL_FEEDING:
                    newState = handleIncrementalFeeding(timestamp, mCurrentStateStartTime);
                    break;  
                default:
                    newState = SystemState.IDLE;
                }
                if (newState != mSystemState) {
                    System.out.println("Feeder state " + mSystemState + " to " + newState);
                    mSystemState = newState;
                    mCurrentStateStartTime = timestamp;
                    mStateChanged = true;
                } else {
                    mStateChanged = false;
                }
            }
        }

        @Override
        public void onStop(double timestamp) {
            stop();
        }
    };

    private SystemState defaultStateTransfer() {
        switch (mWantedState) {
        case INCREMENT_FEED:
            return SystemState.INCREMENTAL_FEEDING;
        case CONTINUOUS_FEED:
            return SystemState.CONTINUOUS_FEEDING;
        case UNJAM:
            return SystemState.UNJAMMING;
        default:
            return SystemState.IDLE;
        }
    }

    private SystemState handleIdle() {
        setMotors(0);
        return defaultStateTransfer();
    }

    private SystemState handleUnjamming(double now, double startStartedAt) {
        setMotors(Constants.kFeederUnjamPower);
        SystemState newState = SystemState.UNJAMMING;
        if (now - startStartedAt > Constants.kFeederUnjamPeriod) {
            newState = SystemState.IDLE;
        }
        switch (mWantedState) {
        case INCREMENT_FEED:
            return SystemState.INCREMENTAL_FEEDING;
        case CONTINUOUS_FEED:
            return SystemState.CONTINUOUS_FEEDING;
        case UNJAM:
            return newState;
        default:
            return SystemState.IDLE;
        }
    }
    
    //look to see if ball available, and check for output
    
    private SystemState handleIncrementalFeeding(double now, double startStartedAt) {
        SystemState newState = SystemState.INCREMENTAL_FEEDING;
        if (mStateChanged) {
           setMotors(0);
           if(!mIRInitial.seesBall()) {
               System.out.println("Hopper empty!");
               newState = SystemState.IDLE;
           }           
        }
        if (now - startStartedAt > Constants.kFeederClogPeriod) {
            newState = SystemState.UNJAMMING;
        } else if(mIRFinal.seesBall()) {
            newState=SystemState.IDLE;
        }else {
            setMotors(Constants.kFeederIncrementFeedPower);
        }

        switch (mWantedState) {
        case INCREMENT_FEED:
            return newState;
        case CONTINUOUS_FEED:
            return SystemState.CONTINUOUS_FEEDING;
        case UNJAM:
            return SystemState.UNJAMMING;
        default:
            return SystemState.IDLE;
        }
    }

    private SystemState handleContinuousFeeding() {
        if (mStateChanged) {
            setMotors(Constants.kFeederContinuousFeedPower);
        }
        return defaultStateTransfer();
    }

    public synchronized void setWantedState(WantedState state) {
        mWantedState = state;
    }


    @Override
    public void outputToSmartDashboard() {
        // SmartDashboard.putNumber("feeder_speed", mMasterTalon.get() / Constants.kFeederSensorGearReduction);
    }

    @Override
    public void stop() {
        setWantedState(WantedState.IDLE);
    }

    @Override
    public void zeroSensors() {
    }

    public void setMotors(double power){
        mTriggerL.set(power);
        mTriggerR.set(-power);
    }

    @Override
    public void registerEnabledLoops(Looper in) {
        in.register(mLoop);
    }

    public boolean checkSystem() {
        System.out.println("Testing FEEDER.-----------------------------------");
        boolean failure=false;       
        return !failure;
    }

}