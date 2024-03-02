package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ModuleConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.Autos;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.SwerveSubsystemSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import java.util.List;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

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
    Joystick m_driverController = new Joystick(OIConstants.kDriverControllerPort);

    // Only used for testing simulation with an XBox controller.
    XboxController m_driverControllerSim = new XboxController(Constants.OIConstants.kDriverControllerPort);
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
                                    -MathUtil.applyDeadband(m_driverController.getY(), OIConstants.kDriveDeadband),
                                    -MathUtil.applyDeadband(m_driverController.getX(), OIConstants.kDriveDeadband),
                                    -MathUtil.applyDeadband(m_driverController.getZ(), OIConstants.kDriveDeadband),
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
                                    -MathUtil.applyDeadband(m_driverController.getY(), OIConstants.kDriveDeadband),
                                    -MathUtil.applyDeadband(m_driverController.getX(), OIConstants.kDriveDeadband),
                                    -MathUtil.applyDeadband(m_driverController.getZ(), OIConstants.kDriveDeadband),
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
            new JoystickButton(m_driverController, Constants.JoystickButtons.kButton1)
                    .whileTrue(new RunCommand(
                            () -> m_robotDriveSim.setX(),
                            m_robotDriveSim));
            new JoystickButton(m_driverController, Constants.JoystickButtons.kButton2)
                    .whileTrue(new RunCommand(
                            () -> m_robotDriveSim.printModulePositions(),
                            m_robotDriveSim));
            new JoystickButton(m_driverController, Constants.JoystickButtons.kButton3)
                    .whileTrue(new InstantCommand(
                            () -> m_robotDriveSim.setZero(),
                            m_robotDriveSim));
        }
        else {
            new JoystickButton(m_driverController, Constants.JoystickButtons.kButton1)
                    .whileTrue(new RunCommand(
                            () -> m_robotDrive.setX(),
                            m_robotDrive));
            new JoystickButton(m_driverController, Constants.JoystickButtons.kButton2)
                    .whileTrue(new RunCommand(
                            () -> m_robotDrive.printModulePositions(),
                            m_robotDrive));
            new JoystickButton(m_driverController, Constants.JoystickButtons.kButton3)
                    .whileTrue(new InstantCommand(
                            () -> m_robotDrive.setZero(),
                            m_robotDrive));
        }
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        Autos auto = new Autos(m_robotDriveSim);
        return auto.getAutonomousCommand();
        /*
         * // Create config for trajectory
         * TrajectoryConfig config = new TrajectoryConfig(
         * AutoConstants.kMaxSpeedMetersPerSecond,
         * AutoConstants.kMaxAccelerationMetersPerSecondSquared)
         * // Add kinematics to ensure max speed is actually obeyed
         * .setKinematics(DriveConstants.kDriveKinematics);
         * 
         * // An example trajectory to follow. All units in meters.
         * Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
         * // Start at the origin facing the +X direction
         * new Pose2d(0, 0, new Rotation2d(0)),
         * // Pass through these two interior waypoints, making an 's' curve path
         * List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
         * // End 3 meters straight ahead of where we started, facing forward
         * new Pose2d(3, 0, new Rotation2d(0)),
         * config);
         * 
         * var thetaController = new ProfiledPIDController(
         * AutoConstants.kPThetaController, 0, 0,
         * AutoConstants.kThetaControllerConstraints);
         * thetaController.enableContinuousInput(-Math.PI, Math.PI);
         * 
         * SwerveControllerCommand swerveControllerCommand = new
         * SwerveControllerCommand(
         * exampleTrajectory,
         * m_robotDrive::getPose, // Functional interface to feed supplier
         * DriveConstants.kDriveKinematics,
         * 
         * // Position controllers
         * new PIDController(AutoConstants.kPXController, 0, 0),
         * new PIDController(AutoConstants.kPYController, 0, 0),
         * thetaController,
         * m_robotDrive::setModuleStates,
         * m_robotDrive);
         * 
         * // Reset odometry to the starting pose of the trajectory.
         * 
         * 
         * // Run path following command, then stop at the end.
         * return Commands.sequence(
         * new InstantCommand(() ->
         * m_robotDrive.resetOdometry(exampleTrajectory.getInitialPose())),
         * swerveControllerCommand,
         * new InstantCommand(() -> m_robotDrive.drive(0,0,0,false,false))
         * );
         * 
         */

        // return swerveControllerCommand.andThen(() -> m_robotDrive.drive(0, 0, 0,
        // false, false));
    }
}