package frc.robot;

import edu.wpi.first.math.MathUtil;
// import edu.wpi.first.math.controller.PIDController;
// import edu.wpi.first.math.controller.ProfiledPIDController;
// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.math.geometry.Rotation2d;
// import edu.wpi.first.math.geometry.Translation2d;
// import edu.wpi.first.math.trajectory.Trajectory;
// import edu.wpi.first.math.trajectory.TrajectoryConfig;
// import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
// import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
// import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj.shuffleboard.EventImportance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

// import java.util.List;

// import frc.robot.commands.Autos;
// import frc.robot.commands.CirclingDriveCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.SwerveSubsystemSim;
import frc.robot.subsystems.VisionSubsystem;

// import static frc.robot.Constants.AutoConstants.*;
// import static frc.robot.Constants.DriveConstants.*;
import static frc.robot.Constants.OperatorConstants.*;
import static frc.robot.Constants.ShooterConstants.kClosePivotAngle;
import static frc.robot.Constants.ShooterConstants.kFarPivotAngle;

//Commented out imports for manual trajectory based auto

import frc.robot.commands.ClimberSubsystemCommand;
import frc.robot.commands.RunIntakeCommand;
import frc.robot.commands.RunShooterCommand;
import frc.robot.commands.PivotingSubsystemCommand;
import frc.robot.commands.Autos;
import frc.robot.commands.BackupShooterCommand;
// import frc.robot.commands.SmartIntakeCommand;
// import frc.robot.commands.SmartShooterCommand;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.PivotingSubsystem;




/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    // The robot's subsystems
    private final DriveSubsystem m_robotDrive = new DriveSubsystem();


    private final SwerveSubsystemSim m_robotDriveSim = new SwerveSubsystemSim();
 
    private boolean circlingMode = false;
    private boolean fieldRelative = true;
    private boolean manualLayout = false;

      // The robot's subsystems and commands are defined here...
  private final IntakeSubsystem s_intakeSubsystem = new IntakeSubsystem();
  private final ShooterSubsystem s_shooterSubsystem = new ShooterSubsystem();
  private final PivotingSubsystem s_pivotingSubsystem = new PivotingSubsystem();
  private final VisionSubsystem s_visionSubsystem = new VisionSubsystem();
  private final Limelight s_Limelight = new Limelight();

  private final ClimberSubsystem s_ClimberSubsystem = new ClimberSubsystem();


  private final Joystick driveJoystick = Mechanisms.m_driverController;
  private final Joystick mechJoystick = Mechanisms.m_mechanismsController;
  private JoystickButton intakeButton = new JoystickButton(mechJoystick, INTAKE_BUTTON);//is it always going to be autnonmous?
  private JoystickButton shooterButton = new JoystickButton(mechJoystick, SHOOT_BUTTON);
  private JoystickButton PivotUpButton = new JoystickButton(mechJoystick, PIVOT_UP_BUTTON);
  private JoystickButton PivotDownButton = new JoystickButton(mechJoystick,PIVOT_DOWN_BUTTON);//is it always going to be autnonmous?
  private JoystickButton ClimberUpButton = new JoystickButton(mechJoystick, CLIMBER_UP_BUTTON);
  private JoystickButton ClimberDownButton = new JoystickButton(mechJoystick, CLIMBER_DOWN_BUTTON);
  private JoystickButton FarPivotButton = new JoystickButton(mechJoystick, FAR_PIVOT_BUTTON);
  private JoystickButton ClosePivotButton = new JoystickButton(mechJoystick, CLOSE_PIVOT_BUTTON);
  private JoystickButton SmartToggleButton = new JoystickButton(mechJoystick, SMART_TOGGLE_BUTTON);
        private JoystickButton initShooterButton = new JoystickButton(mechJoystick, kButton3);
    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        // if (RobotBase.isSimulation()) {
        //     // Configure default commands
        //     m_robotDriveSim.setDefaultCommand(
        //             // The left stick controls translation of the robot.
        //             // Turning is controlled by the X axis of the right stick.
        //             new RunCommand(
        //                     () -> m_robotDriveSim.drive(
        //                             // Multiply by max speed to map the joystick unitless inputs to actual units.
        //                             // This will map the [-1, 1] to [max speed backwards, max speed forwards],
        //                             // converting them to actual units.
        //                             -MathUtil.applyDeadband(driveJoystick.getY(), kDriveDeadband),
        //                             -MathUtil.applyDeadband(driveJoystick.getX(), kDriveDeadband),
        //                             -MathUtil.applyDeadband(driveJoystick.getZ(), kDriveDeadband),
        //                             false,
        //                             false),
        //                     m_robotDriveSim));
        // } else {
            // Configure default commands
            m_robotDrive.setDefaultCommand(
                    // The left stick controls translation of the robot.
                    // Turning is controlled by the X axis of the right stick.
                    new RunCommand(
                            () -> m_robotDrive.drive(
                                    -MathUtil.applyDeadband(driveJoystick.getY(), kDriveDeadband),
                                    -MathUtil.applyDeadband(driveJoystick.getX(), kDriveDeadband),
                                    -MathUtil.applyDeadband(driveJoystick.getZ(), kDriveDeadband),
                                    driveJoystick.getThrottle(),
                                    true, false, false),
                            m_robotDrive));
                // m_robotDrive.setDefaultCommand(new CirclingDriveCommand(m_robotDrive, s_visionSubsystem, driveJoystick, circlingMode));
                // s_pivotingSubsystem.setDefaultCommand(new PivotingSubsystemCommand(s_pivotingSubsystem, mechJoystick.getY()));
        // }
        // s_visionSubsystem.setDefaultCommand(new RunCommand(
        //         () -> s_visionSubsystem.captureTask()
        // ));
        CommandScheduler.getInstance()
                .onCommandInitialize(
                     command -> 
                        Shuffleboard.addEventMarker(
                                "Command initialized", command.getName(), EventImportance.kNormal));
        CommandScheduler.getInstance()
                .onCommandExecute(
                     command -> 
                        Shuffleboard.addEventMarker(
                                "Command executed", command.getName(), EventImportance.kNormal));
        CommandScheduler.getInstance()
                .onCommandFinish(
                     command -> 
                        Shuffleboard.addEventMarker(
                                "Command finished", command.getName(), EventImportance.kNormal));
        CommandScheduler.getInstance()
                .onCommandInterrupt(
                     command -> 
                        Shuffleboard.addEventMarker(
                                "Command interrupted", command.getName(), EventImportance.kNormal));
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
            new JoystickButton(driveJoystick, BRAKE_BUTTON)
                    .whileTrue(new RunCommand(
                            () -> m_robotDriveSim.setX(),
                            m_robotDriveSim));
        }
        else {
            new JoystickButton(driveJoystick, BRAKE_BUTTON)
                    .whileTrue(new RunCommand(
                            () -> m_robotDrive.setX(),
                            m_robotDrive));
                // new JoystickButton(driveJoystick, Constants.JoystickButtons.kButton2)
                //         .whileTrue(new RunCommand(
                //                 () -> m_robotDrive.printModulePositions(),
                //                  m_robotDrive));
                new JoystickButton(driveJoystick, CIRCLING_TOGGLE_BUTTON)
                        .whileTrue(new InstantCommand(
                                () -> circlingMode = !circlingMode,
                                m_robotDrive));                
                new JoystickButton(driveJoystick, DRIVE_CONTROL_TOGGLE_BUTTON)
                        .whileTrue(new InstantCommand(
                                () -> fieldRelative = !fieldRelative,
                                m_robotDrive));
                new JoystickButton(driveJoystick, SET_FORWARD_BUTTON)
                        .whileTrue(new InstantCommand(
                                () -> m_robotDrive.zeroHeading(),
                                m_robotDrive));
                new JoystickButton(driveJoystick, SMART_TOGGLE_BUTTON)
                        .whileTrue(new InstantCommand(
                                () -> manualLayout = !manualLayout,
                                m_robotDrive));
                new JoystickButton(driveJoystick, FAR_PIVOT_BUTTON)
                        .whileTrue(new InstantCommand(
                                () -> s_pivotingSubsystem.rotateShooterToAngle(kFarPivotAngle),
                                s_pivotingSubsystem));
                new JoystickButton(driveJoystick, CLOSE_PIVOT_BUTTON)
                        .whileTrue(new InstantCommand(
                                () -> s_pivotingSubsystem.rotateShooterToAngle(kClosePivotAngle),
                                s_pivotingSubsystem));
                new JoystickButton(driveJoystick, TEST_BUTTON)
                        .whileTrue(new InstantCommand(
                                () -> System.out.println()
                                ));
                // new JoystickButton(mechJoystick, kButton8)
                //         .whileTrue(new RunCommand(
                //                 () -> s_shooterSubsystem.startEnd(() -> s_shooterSubsystem.StartShooter(1),
                //                 () -> s_shooterSubsystem.stopShooter())
                //                 ));
                // new JoystickButton(driveJoystick, kButton7)
                //         .whileTrue(new InstantCommand(
                //                 () -> m_robotDrive.()
                //                 ));
                //  new JoystickButton(driveJoystick, TEST_BUTTON)
                //         .whileTrue(new InstantCommand(
                //                 () -> s_shooterSubsystem.stopShooter()));
                initShooterButton.whileTrue(new BackupShooterCommand(s_shooterSubsystem));
                // PivotUpButton.whileTrue(new PivotingSubsystemCommand(s_pivotingSubsystem, 1));
                // PivotDownButton.whileTrue(new PivotingSubsystemCommand(s_pivotingSubsystem, -1));
                ClimberUpButton.whileTrue(new ClimberSubsystemCommand(s_ClimberSubsystem, mechJoystick, 1));
                ClimberDownButton.whileTrue(new ClimberSubsystemCommand(s_ClimberSubsystem, mechJoystick, -1));
                // if(manualLayout){
                intakeButton.whileTrue(new RunIntakeCommand(s_intakeSubsystem));
                shooterButton.whileTrue(new RunShooterCommand(s_shooterSubsystem));
                // }
                // else{
                // intakeButton.whileTrue(new SmartIntakeCommand(m_robotDrive,s_visionSubsystem,s_intakeSubsystem));
                // shooterButton.whileTrue(new SmartShooterCommand(m_robotDrive,s_visionSubsystem,s_shooterSubsystem,s_pivotingSubsystem));
                // }
                
        }
    }

    public void setForward() {
        m_robotDrive.zeroHeading();
    }

//     public double getGyroAngle(){
//         return m_robotDrive.fetchGyroData();
//     }
    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */

        // public void putChoosertoDashboard() {
        // auto.putChoosertoDashboard();
        // }

    public Command getAutonomousCommand() {
        Autos auto = new Autos(m_robotDrive, s_shooterSubsystem, s_intakeSubsystem);
        return auto.getAutonomousCommand();
    }
}





