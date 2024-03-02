// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Mechanisms;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj.DutyCycleEncoder;

public class ClimberSubsystem extends SubsystemBase {
  /** Variables */
  // private static CANSparkMax climberMotor = Mechanisms.m_climbingMotor;
  // private static DutyCycleEncoder climberEncoder = Mechanisms.m_climberEncoder;
  //private double motorRotateSpeed = 0.5;


  /** Creates a new ExampleSubsystem. */
  public ClimberSubsystem() {}

  /**
   * Example command factory method.
   *
   * @return a command
   */
  // public void startClimber(double motorRotateSpeed) {
  //   // Inline construction of command goes here.
  //   // Subsystem::RunOnce implicitly requires `this` subsystem.
  //   climberMotor.set(motorRotateSpeed);
        
  // }
  // public void stopClimber() {
  //   climberMotor.stopMotor();;
  // }
  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  // public double getClimbEncoder(){
  //   return climberEncoder.getDistance();
  // }
    // Query some boolean state, such as a digital sensor.
    
  

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
