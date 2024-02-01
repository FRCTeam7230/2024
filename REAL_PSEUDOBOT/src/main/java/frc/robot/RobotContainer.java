// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.ExampleCommand;
import frc.robot.subsystems.AutosSubsystem;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final IntakeSubsystem m_intakeSubsystem = new IntakeSubsystem();
  private final ShooterSubsystem m_shooterSubsystem = new ShooterSubsystem();
  private final AutosSubsystem a_autos = new AutosSubsystem();

  private final Joystick driveJoystick = new Joystick(OperatorConstants.kDriverJoystickPort);
  private final Joystick shooterJoystick = new Joystick(OperatorConstants.kActionJoystickPort);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
  }

  private void configureBindings() {
    // All joystick bindings
    Trigger d_button0Pressed = new Trigger(new JoystickButton(driveJoystick, 0));
    Trigger d_axisUp = new Trigger(() -> driveJoystick.getRawAxis(1) > 0.25);
    Trigger d_axisDown = new Trigger(() -> driveJoystick.getRawAxis(1) < 0.25);

    // d_button0Pressed.onTrue(m_shooterSubsystem.whateverCoolCommand());

    d_axisUp.whileTrue(m_shooterSubsystem.RotateShooterUp());
    d_axisDown.whileTrue(m_shooterSubsystem.RotateShooterDown());

    d_axisUp.onFalse(m_shooterSubsystem.StopRotation());
    d_axisDown.onFalse(m_shooterSubsystem.StopRotation());

    // onTrue / onFalse: Schedules for one iteration.
    // whileTrue / whileFalse: Schedules every iteration it is true.
    // toggleOnTrue / toggleOnFalse: just dont use, as frc does not recommend it
    // YOU CAN CHAIN DIFFERENT CONDITIONALS. FOR EXAMPLE: .onTrue().onFalse()
    // YOU CAN ALSO DO AND OR XOR ETC
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public void getAutonomousCommand() {
    // An example command will be run in autonomous
    // return a_autos.soontobemadecommand();
  }
}
