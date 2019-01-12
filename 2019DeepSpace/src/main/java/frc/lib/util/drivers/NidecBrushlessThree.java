
package com.team6498.lib.util.drivers;

import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.MotorSafety;
import edu.wpi.first.wpilibj.MotorSafetyHelper;
import edu.wpi.first.wpilibj.PWM;
import edu.wpi.first.wpilibj.Sendable;
import edu.wpi.first.wpilibj.SendableBase;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.hal.FRCNetComm.tResourceType;
import edu.wpi.first.wpilibj.hal.HAL;
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilder;

/**
 * Nidec Brushless Motor FOR THE 3.3 FREAKING MOTOR
 */
public class NidecBrushlessThree extends SendableBase implements SpeedController, MotorSafety, Sendable {
  private final MotorSafetyHelper m_safetyHelper;
  private boolean m_isInverted = false;
  private DigitalOutput m_dio;
  private PWM m_pwmEnable;
  private PWM m_pwmDirection;
  private volatile double m_speed = 0.0;
  private volatile boolean m_disabled = false;
  Encoder encoder;

  /**
   * Constructor.
   *
   * @param pwmEnableChannel The PWM channel used to enable and disable.
   *                   
   * @param dioChannel The DIO channel used as a PWM signal.
   *                
   * @param pwmDirectionChannel The PWM channel used for the direction pin.
   *                  
   * @param encoderAChannel The A channel for the encoder (DIO).
   * 
   * @param encoderBChannel The B channel for the encoder (DIO).
   *              
   */
  public NidecBrushlessThree(final int pwmEnableChannel, final int dioChannel, final int pwmDirectionChannel, final int encoderAChannel, final int encoderBChannel) {
    m_safetyHelper = new MotorSafetyHelper(this);
    m_safetyHelper.setExpiration(0.0);
    m_safetyHelper.setSafetyEnabled(false);

    // the dio controls the output (in PWM mode)
    m_dio = new DigitalOutput(dioChannel);
    addChild(m_dio);
    m_dio.setPWMRate(15625); //frequency 
    m_dio.enablePWM(0);

    // the pwm enables the controller
    m_pwmEnable = new PWM(pwmEnableChannel);
    addChild(m_pwmEnable);

    //this pwm sets the direction of the motor
    m_pwmDirection = new PWM(pwmDirectionChannel);
    addChild(m_pwmDirection);
    
    HAL.report(tResourceType.kResourceType_NidecBrushless, pwmEnableChannel);
    setName("Nidec Brushless", pwmEnableChannel);
    
    encoder=new Encoder(encoderAChannel, encoderBChannel);
    encoder.setDistancePerPulse(.01); //100 ticks/revolution
  }
  
  public double getRawTicks() {
	  return encoder.getRaw();
  }
  
  public double getPosition() {
	  return encoder.getDistance();
  }
  
  public void resetDistance() {
	  encoder.reset();
  }
  public double getRate() {
	 return encoder.getRate();
  }

  /**
   *
   * @return RPM of Encoder (hopefully :D)
   */
  public double getSpeed(){
    return (encoder.getRate()*60)/(encoder.getDistancePerPulse()*100); //I dont know if this is right at all lol
  }



  //Reversing the sensor
  private boolean sensorReversed=false;
  public void reverseSensor(boolean reverse){
    sensorReversed=reverse;
    encoder.setReverseDirection(reverse);
  }

  public boolean getSensorReversed(){
    return sensorReversed;
  }






















  /**
   * Free the resources used by this object.
   */
  @Override
  public void free() {
    super.free();
    m_dio.free();
    m_pwmEnable.free();
    m_pwmDirection.free();
  }



  public enum ControlMode{
    RPM,
    Speed,
    PWM
  }

  private ControlMode mControlMode = ControlMode.PWM;

  public void setControlMode(ControlMode mode){
    mControlMode = mode;
  }

  @Override
  public void set(double speed){
    switch(mControlMode){
      case RPM:

        break;
      case Speed:

        break;
      case PWM:
        setRaw(speed);
        break;
    }
  }

  double speedToPWM(double speed){
    //TODO make conversion
    return 0;
  }

  double RPMtoPWM(double rpm){
    //TODO make conversion
    return 0;
  }


  /**
   * Set the PWM value.
   *
   * <p>The PWM value is set using a range of -1.0 to 1.0, appropriately scaling the value for the
   * FPGA.
   *
   * @param speed The speed value between -1.0 and 1.0 to set.
   */
  public void setRaw(double speed) {
    if (!m_disabled) {
    	 m_speed = speed;
    	speed=(m_isInverted ? -speed : speed);
      if(speed>=0) {
    	  m_pwmDirection.setDisabled();
      }else {
    	  m_pwmDirection.setRaw(0xffff);
      }
     
      m_dio.updateDutyCycle(Math.abs(speed));
      
    }
  
  }

  /**
   * Get the recently set value of the PWM.
   *
   * @return The most recently set value for the PWM between -1.0 and 1.0.
   */
  @Override
  public double get() {
    return m_speed;
  }

  @Override
  public void setInverted(boolean isInverted) {
    m_isInverted = isInverted;
  }

  @Override
  public boolean getInverted() {
    return m_isInverted;
  }



  /**
   * Write out the PID value as seen in the PIDOutput base object.
   *
   * @param output Write out the PWM value as was found in the PIDController
   */
  @Override
  public void pidWrite(double output) {
    set(output);
  }

  /**
   * Set the safety expiration time.
   *
   * @param timeout The timeout (in seconds) for this motor object
   */
  @Override
  public void setExpiration(double timeout) {
    m_safetyHelper.setExpiration(timeout);
  }

  /**
   * Return the safety expiration time.
   *
   * @return The expiration time value.
   */
  @Override
  public double getExpiration() {
    return m_safetyHelper.getExpiration();
  }

  /**
   * Check if the motor is currently alive or stopped due to a timeout.
   *
   * @return a bool value that is true if the motor has NOT timed out and should still be running.
   */
  @Override
  public boolean isAlive() {
    return m_safetyHelper.isAlive();
  }

  /**
   * Stop the motor. This is called by the MotorSafetyHelper object
   * when it has a timeout for this PWM and needs to stop it from running.
   * Calling set() will re-enable the motor.
   */
  @Override
  public void stopMotor() {
    m_dio.updateDutyCycle(0); //THIS IS WHY IT WAS FLIPPING MOVING :D (3.3's have 0-1 signal where 0 is off, unlike Dynamo's 0.5)
    m_pwmEnable.setDisabled();
  }

  /**
   * Check if motor safety is enabled.
   *
   * @return True if motor safety is enforced for this object
   */
  @Override
  public boolean isSafetyEnabled() {
    return m_safetyHelper.isSafetyEnabled();
  }

  @Override
  public void setSafetyEnabled(boolean enabled) {
    m_safetyHelper.setSafetyEnabled(enabled);
  }

  @Override
  public String getDescription() {
    return "Nidec " + getChannel();
  }

  /**
   * Disable the motor.  The enable() function must be called to re-enable
   * the motor.
   */
  @Override
  public void disable() {
    m_disabled = true;
    m_dio.updateDutyCycle(0);
   
    //m_pwmEnable.setRaw(0xffff);
    m_pwmEnable.setDisabled();
  }

  /**
   * Re-enable the motor after disable() has been called.  The set()
   * function must be called to set a new motor speed.
   */
  public void enable() {
	  m_pwmEnable.setRaw(0xffff);
	  //m_pwmEnable.setDisabled();
  }

  /**
   * Gets the channel number associated with the object.
   *
   * @return The channel number.
   */
  public int getChannel() {
    return m_dio.getChannel();
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.setSmartDashboardType("Nidec Brushless");
    builder.setSafeState(this::stopMotor);
    builder.addDoubleProperty("Value", this::get, this::set);
  }
}