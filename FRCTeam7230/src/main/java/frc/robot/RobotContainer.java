package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;

import java.util.List;

import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.SwerveSubsystemSim;
import static frc.robot.Constants.AutoConstants.*;
import static frc.robot.Constants.DriveConstants.*;
import static frc.robot.Constants.OperatorConstants.*;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    // The robot's subsystems
    private final DriveSubsystem m_robotDrive = new DriveSubsystem();

    // The driver's controller
    private final Joystick m_driverController = Mechanisms.m_driverController;

    private final SwerveSubsystemSim m_robotDriveSim = new SwerveSubsystemSim();

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        if (RobotBase.isSimulation()) {
            // Configure default commands
            m_robotDriveSim.setDefaultCommand(
                    // The left stick controls translation of the robot.
                    // Turning is controlled by the X axis of the right stick.
                    new RunCommand(
                            () -> m_robotDriveSim.drive(
                                    // Multiply by max speed to map the joystick unitless inputs to actual units.
                                    // This will map the [-1, 1] to [max speed backwards, max speed forwards],
                                    // converting them to actual units.
                                    -MathUtil.applyDeadband(m_driverController.getY(), kDriveDeadband),
                                    -MathUtil.applyDeadband(m_driverController.getX(), kDriveDeadband),
                                    -MathUtil.applyDeadband(m_driverController.getZ(), kDriveDeadband),
                                    false,
                                    false),
                            m_robotDriveSim));
        } else {
            // Configure default commands
            m_robotDrive.setDefaultCommand(
                    // The left stick controls translation of the robot.
                    // Turning is controlled by the X axis of the right stick.
                    new RunCommand(
                            () -> m_robotDrive.drive(
                                    -MathUtil.applyDeadband(m_driverController.getY(), kDriveDeadband),
                                    -MathUtil.applyDeadband(m_driverController.getX(), kDriveDeadband),
                                    -MathUtil.applyDeadband(m_driverController.getZ(), kDriveDeadband),
                                    true, true),
                            m_robotDrive));
        }

        // Configure the button bindings
        configureButtonBindings();
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be
     * created by
     * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its
     * subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling
     * passing it to a
     * {@link JoystickButton}.
     */
    private void configureButtonBindings() {
        if (RobotBase.isSimulation()) {
            new JoystickButton(m_driverController, BRAKE)
                    .whileTrue(new RunCommand(
                            () -> m_robotDriveSim.setX(),
                            m_robotDriveSim));
        }
        else {
            new JoystickButton(m_driverController, BRAKE)
                    .whileTrue(new RunCommand(
                            () -> m_robotDrive.setX(),
                            m_robotDrive));
        }
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // Autos auto = new Autos(m_robotDriveSim);
        
        
         // Create config for trajectory
         TrajectoryConfig config = new TrajectoryConfig(
         kAutoMaxSpeedMetersPerSecond,
         kMaxAccelerationMetersPerSecondSquared)
         // Add kinematics to ensure max speed is actually obeyed
         .setKinematics(kDriveKinematics);
         
         // An example trajectory to follow. All units in meters.
         Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
         // Start at the origin facing the +X direction
         new Pose2d(0, 0, new Rotation2d(0)),
         // Pass through these two interior waypoints, making an 's' curve path
         List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
         // End 3 meters straight ahead of where we started, facing forward
         new Pose2d(3, 0, new Rotation2d(0)),
         config);
         
         var thetaController = new ProfiledPIDController(
         kPThetaController, 0, 0,
         kThetaControllerConstraints);
         thetaController.enableContinuousInput(-Math.PI, Math.PI);
         
         SwerveControllerCommand swerveControllerCommand = new
         SwerveControllerCommand(
         exampleTrajectory,
         m_robotDrive::getPose, // Functional interface to feed supplier
         kDriveKinematics,
         
         // Position controllers
         new PIDController(kPXController, 0, 0),
         new PIDController(kPYController, 0, 0),
         thetaController,
         m_robotDrive::setModuleStates,
         m_robotDrive);
         new TrajectoryConfig(
         kAutoMaxSpeedMetersPerSecond,
         kMaxAccelerationMetersPerSecondSquared)
         // Add kinematics to ensure max speed is actually obeyed
         .setKinematics(kDriveKinematics);
         // Reset odometry to the starting pose of the trajectory.

        //  return auto.getAutonomousCommand();
         
         // Run path following command, then stop at the end.
         return Commands.sequence(
         new InstantCommand(() ->
         m_robotDrive.resetOdometry(exampleTrajectory.getInitialPose())),
         swerveControllerCommand,
         new InstantCommand(() -> m_robotDrive.drive(0,0,0,false,false))
         );
         
        

        // return swerveControllerCommand.andThen(() -> m_robotDrive.drive(0, 0, 0,
        // false, false));
    }
}
