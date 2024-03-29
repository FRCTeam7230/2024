// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Mechanisms;
import frc.robot.Constants.IntakeConstants;
import com.revrobotics.CANSparkMax;

public class IntakeSubsystem extends SubsystemBase {

  /** Variables */
  private static CANSparkMax intakeMotor = Mechanisms.m_intakeMotor; // Motor for the main intake
  private static CANSparkMax shooterIntakeMotor = Mechanisms.m_ShooterIntakeMotor; // Motor for the shooter intake
 
  private static DigitalInput noteSensor = Mechanisms.m_noteBeamSensor;      // Digital input for the intake sensor

  //public static boolean intakeSystemOn = false;
  private double motorRotateSpeed = 1; // Speed for the intake motor


  public IntakeSubsystem() { // Constructor
    //intakeMotor.restoreFactoryDefaults();
    //ShooterIntakeMotor.restoreFactoryDefaults();
  }

  /**
   * Runs the intake motors forward.
   *
   * 
   */

  public void startIntakeSystem() {
    // intakeMotor.set(motorRotateSpeed); // Starts the intake system
    
    shooterIntakeMotor.set(motorRotateSpeed);

  }
   
  

  public void stopIntakeSystem() { // Stops the intake system
    // intakeMotor.stopMotor();
    shooterIntakeMotor.stopMotor();
  }

  public Command startIntake(double rotSpeed){
    return Commands.parallel(
      this.runOnce(() -> intakeMotor.set(rotSpeed)),
      this.runOnce(() -> shooterIntakeMotor.set(rotSpeed))
    );
  }

  public Command stopIntake(){
    return Commands.parallel(
      this.runOnce(() -> intakeMotor.stopMotor()),
      this.runOnce(() -> shooterIntakeMotor.stopMotor())
      );
  }

  /**
   * An example method querying a boolean state of the subsystem (for example, a
   * digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */


   //returns true if note is detected
  public static boolean checkSensor() { // Checks if the sensor is triggered
    // System.out.println(intakeSensor.get());
    return !noteSensor.get();
    
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
