// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.Vision2Subsystem;
import frc.robot.subsystems.Limelight;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */

public class Robot extends TimedRobot {
  private boolean red = true;
  private final SendableChooser<String> color_chooser = new SendableChooser<>();
  Limelight limelight = new Limelight();
  private String colorSelected;
  private Command m_autonomousCommand;
  private double offsetX, offsetY, tagDistance, tagID;
  private RobotContainer m_robotContainer;
  private void refreshLimeghtData() {
    offsetX = Limelight.getTargetAngleX();
    offsetY = Limelight.getTargetAngleY();
    tagID = Limelight.getTargetID();
    tagDistance = Limelight.apriltagDistance();
    SmartDashboard.putNumber("X offset", offsetX);
    SmartDashboard.putNumber("Y offset", offsetY);
    SmartDashboard.putNumber("Tag ID", tagID);
    SmartDashboard.putNumber("Tag Distance", tagDistance);
  }
  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();
    // Vision2Subsystem vision = new Vision2Subsystem();
    color_chooser.setDefaultOption("red", "red");
    color_chooser.addOption("blue", "blue");
    SmartDashboard.putData("Color choice", color_chooser);

  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    colorSelected = color_chooser.getSelected();
    switch (colorSelected) {
      case "red":
        red = true;
        break;
      case "blue":
        red = false;
        break;
    }
    CommandScheduler.getInstance().run();
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  //@Override
  /*public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }*/

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    // offsetX = Limelight.getTargetAngleX();
    // offsetY = Limelight.getTargetAngleY();
    // tagID = Limelight.getTargetID();
    // tagDistance = Limelight.apriltagDistance();
    // SmartDashboard.putNumber("X offset", offsetX);
    // SmartDashboard.putNumber("Y offset", offsetY);
    // SmartDashboard.putNumber("Tag ID", tagID);
    // SmartDashboard.putNumber("Tag Distance", tagDistance);
    // SmartDashboard.updateValues();
  }

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }

    // Vision2Subsystem visSub = new Vision2Subsystem();
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    Limelight.limelight(colorSelected);
    offsetX = Limelight.getTargetAngleX();
    offsetY = Limelight.getTargetAngleY();
    tagID = Limelight.getTargetID();
    tagDistance = Limelight.apriltagDistance();
    SmartDashboard.putNumber(" X offset", offsetX);
    SmartDashboard.putNumber(" Y offset", offsetY);
    SmartDashboard.putNumber(" Tag ID", tagID);
    SmartDashboard.putNumber(" Tag Distance", tagDistance);
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}