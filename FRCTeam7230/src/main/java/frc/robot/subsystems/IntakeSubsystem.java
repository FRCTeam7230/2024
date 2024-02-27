// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Mechanisms;
import frc.robot.Constants.IntakeConstants;
import com.revrobotics.CANSparkMax;

public class IntakeSubsystem extends SubsystemBase {

  /** Variables */
  private static CANSparkMax intakeMotor = Mechanisms.m_intakeMotor;
  private static CANSparkMax transferMotor = Mechanisms.m_transferToShooterMotor;
  private static DigitalInput intakeSensor = Mechanisms.m_noteBeamSensor;

  //public static boolean intakeSystemOn = false;
  private double motorRotateSpeed = 0.5;


  public IntakeSubsystem() {
    intakeMotor.configFactoryDefault();
    transferMotor.configFactoryDefault();
  }

  /**
   * Example command factory method.
   *
   * @return a command
   */

  public Command startIntakeSystem() {
    intakeMotor.set(motorRotateSpeed);
  }

  public Command stopIntakeSystem() {
    intakeMotor.stopMotor();
  }

  /**
   * An example method querying a boolean state of the subsystem (for example, a
   * digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */

  public static boolean checkSensor() {
    return intakeSensor.get();
  }

@Override
public void periodic() {
  // This method will be called once per scheduler run :) 

}

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
