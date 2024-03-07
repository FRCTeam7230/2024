// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import com.revrobotics.SparkAbsoluteEncoder.Type;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Mechanisms;
import frc.robot.Constants.ShooterConstants;
import com.revrobotics.CANSparkMax;


import edu.wpi.first.wpilibj.DutyCycleEncoder;

public class ShooterSubsystem extends SubsystemBase{

  
  /** Variables */
  private static DigitalInput intakeSensor = Mechanisms.m_noteBeamSensor;
  private static DigitalInput limitSwitch = Mechanisms.m_upperLimitSwitch; //povit low
  private static CANSparkMax transferMotor = Mechanisms.m_transferToShooterMotor;

  private static CANSparkMax rightShooterMotor = Mechanisms.m_rightShooterMotor;
  private static CANSparkMax leftShooterMotor = Mechanisms.m_leftShooterMotor;


  
  double motorRotateSpeed = ShooterConstants.kRotationalSpeed;


  public static boolean shooterOn = false;

  public ShooterSubsystem() {
  }

  /**
   * Example command factory method.
   *
   * @return a command
   */
 


  

  public void StartShooter() {
    rightShooterMotor.set(motorRotateSpeed);//check experimentally what the velocity is at a motorRotateSpeed Voltage
    leftShooterMotor.set(motorRotateSpeed);
    if(rightShooterMotor.getAbsoluteEncoder(Type.kDutyCycle).getVelocity() == (917*motorRotateSpeed) 
    && rightShooterMotor.getAbsoluteEncoder(Type.kDutyCycle).getVelocity()== (917*motorRotateSpeed)){
      transferMotor.set(motorRotateSpeed);
    }
  }
  

  public void StopShooter() {
    rightShooterMotor.stopMotor();
    leftShooterMotor.stopMotor();
  }

  /**
   * An example method querying a boolean state of the subsystem (for example, a
   * digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */

  

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
