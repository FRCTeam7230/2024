// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Mechanisms;

public class PivotingSubsystem extends SubsystemBase {
  /** Creates a new PivotingSubsystem. */
  private static CANSparkMax pivotMotor = Mechanisms.m_shooterPivotMotor;
  private static DutyCycleEncoder pivotEncoder = Mechanisms.m_pivotEncoder;
  private static DigitalInput intakeSensor = Mechanisms.m_noteBeamSensor;
  private static DigitalInput limitSwitch = Mechanisms.m_upperLimitSwitch;
  
  public PivotingSubsystem() {
    //pivotMotor.restoreFactoryDefaults();

  }

  public void RotateShooter(double motorRotateSpeed) {
    pivotMotor.set(motorRotateSpeed);
  }

 /*public Command RotateShooterDown(double motorRotateSpeed) {
    pivotMotor.set(ControlMode.PercentOutput, -motorRotateSpeed);    
  }*/
  public void StopRotation(){
    pivotMotor.stopMotor();
  }
  public static double counterValue() {
    return pivotEncoder.getDistance();
  }
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  public static boolean intakeSensor() {
    return intakeSensor.get();
  }
}
