package frc.robot.subsystems;



import frc.lib.util.ConstantsBase;
import frc.lib.util.drivers.NidecBrushlessThree;


import com.ctre.CANTalon;



import frc.robot.Constants;

//import frc.robot.ShooterAimingParameters;

import frc.robot.loops.Loop;

import frc.robot.loops.Looper;

import frc.lib.util.drivers.CANTalonFactory;


/**

 * The shooter subsystem consists of 4 775 Pro motors driving twin backspin flywheels. When run in reverse, these motors

 * power the robot's climber through a 1 way bearing. The shooter subsystem goes through 3 stages when shooting.

 * 1. Spin Up

 *  Use a PIDF controller to spin up to the desired RPM. We acquire this desired RPM by converting the camera's range

 *  value into an RPM value using the range map in the {@link Constants} class.

 * 2. Hold When Ready

 *  Once the flywheel's

 *  RPM stabilizes (remains within a certain bandwidth for certain amount of time), the shooter switches to the hold when

 *  ready stage. In this stage, we collect kF samples. The idea is that we want to run the shooter in open loop when we

 *  start firing, so in this stage we calculate the voltage we need to supply to spin the flywheel at the desired RPM.

 * 3. Hold

 *  Once we collect enough kF samples, the shooter switches to the hold stage. This is the stage that we begin

 *  firing balls. We set kP, kI, and kD all to 0 and use the kF value we calculated in the previous stage for essentially

 *  open loop control. The reason we fire in open loop is that we found it creates a much narrower stream and leads to

 *  smaller RPM drops between fuel shots.

 *

 * @see

 */

public class Flywheel extends Subsystem {

    private static Flywheel mInstance = null;


    public static Flywheel getInstance() {

        if (mInstance == null) {

            mInstance = new Flywheel();

        }

        return mInstance;

    }



    private final NidecBrushlessThree mFlywheel;


    private boolean mOnTarget = false;

    private double mSetpointRpm = 0;

    private Flywheel() {
        mFlywheel = new NidecBrushlessThree(Constants.kShooterEnablepwmPort, Constants.kShooterDIOpwmPort,
                Constants.kShooterDirectionpwmPort, Constants.kShooterEncoderAPort, Constants.kShooterEncoderBPort);

        //TODO set control mode to open loop on NIDEC

    }



    @Override

    public synchronized void outputToSmartDashboard() {



    }



    @Override

    public synchronized void stop() {

        setOpenLoop(0.0);

        mSetpointRpm = 0.0;

    }



    @Override

    public void zeroSensors() {



    }



    @Override

    public void registerEnabledLoops(Looper enabledLooper) {

        enabledLooper.register(new Loop() {

            @Override

            public void onStart(double timestamp) {

                synchronized (Flywheel.this) {

                    mSetpointRpm=0;
                    mOnTarget=false;

                }

            }



            @Override

            public void onLoop(double timestamp) {



            }



            @Override

            public void onStop(double timestamp) {

                setOpenLoop(0);

            }

        });

    }



    public synchronized double getRpm() {

        return mFlywheel.getSpeed();

    }



    /**

     * Sets the RPM of the flywheel. The flywheel will then spin up to the set

     * speed within a preset tolerance.

     *

     * @param

     *

     */

    synchronized void setRpm(double rpm) {

        //mFlywheel.changeControlMode(CANTalon.TalonControlMode.Speed);

        mFlywheel.set(rpm);

    }



    synchronized void setOpenLoop(double speed) {

       // mFlywheel.changeControlMode(CANTalon.TalonControlMode.PercentVbus);

        mFlywheel.set(speed);

    }



    public synchronized double getSetpoint() {

        return mFlywheel.getSetpoint();

    }



    /**

     * @return If the flywheel RPM is within the tolerance to the specified set

     *         point.

     */

    public synchronized boolean isOnTarget() {

        return (mFlywheel.getControlMode() == CANTalon.TalonControlMode.Speed

                && Math.abs(getRpm() - getSetpoint()) < Constants.kFlywheelOnTargetTolerance);

    }








}