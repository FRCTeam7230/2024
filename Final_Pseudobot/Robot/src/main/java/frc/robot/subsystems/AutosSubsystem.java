// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AutosSubsystem extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  public AutosSubsystem() {
  }

  /**
   * Example command factory method.
   *
   * @return a command
   */

  public Command doVision() {
    return runOnce(() -> {
      //command to do vision for one step
      //1. get position by vision
      //2. move in that direction once. Call swerve drive code
    });
  }

  /**
   * An example method querying a boolean state of the subsystem (for example, a
   * digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}