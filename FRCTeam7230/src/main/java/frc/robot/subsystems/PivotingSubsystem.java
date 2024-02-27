// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PivotingSubsystem extends SubsystemBase {
  /** Creates a new PivotingSubsystem. */
  private static CANSparkMax pivotMotor = Mechanisms.m_shooterPivotMotor;
  private static DutyCycleEncoder pivotEncoder = Mechanisms.m_pivotEncoder;
  private static DigitalInput intakeSensor = Mechanisms.m_noteBeamSensor;
  private static DigitalInput limitSwitch = Mechanisms.m_upperLimitSwitch;
  
  public PivotingSubsystem() {
    pivotMotor.configFactoryDefault();

  }

  public Command RotateShooter(double motorRotateSpeed) {
    pivotMotor.set(ControlMode.PercentOutput, motorRotateSpeed);
  }

 /*public Command RotateShooterDown(double motorRotateSpeed) {
    pivotMotor.set(ControlMode.PercentOutput, -motorRotateSpeed);    
  }*/
  public Command StopRotation(){
    pivotMotor.stopMotor();
  }
  public int counterValue() {
    return pivotEncoder.getDistance();
  }
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
