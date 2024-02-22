// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.motorcontrol.Spark;

public class IntakeSubsystem extends SubsystemBase {

  /** Variables */
  public Spark flywheelMotor = new Spark(IntakeConstants.kFlywheelMotorPort1);
  public static DigitalInput intakeSensor = new DigitalInput(IntakeConstants.kSensorPort);

  public static boolean intakeSystemOn = false;

  /** Creates a new ExampleSubsystem. */
  public IntakeSubsystem() {

  }

  /**
   * Example command factory method.
   *
   * @return a command
   */
  public Command exampleMethodCommand() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {

        });
  }

  public Command startIntakeSystem() {
    return runOnce(
        () -> {
          intakeSystemOn = true;
        });
  }

  public Command stopIntakeSystem() {
    return runOnce(
        () -> {
          intakeSystemOn = false;
        });
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

    if (intakeSystemOn) {
      if (checkSensor() == false) {
        // this means we haven't picked up yet
        /* Talk with OPENCV when finioshed */
      } else {
        // means we picked up, skip
      }
    }
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
