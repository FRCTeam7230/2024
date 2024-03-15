// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.DriveConstants;

/**
 * Basic simulation of a swerve subsystem with the methods needed by PathPlanner
 */
public class SwerveSubsystemSim extends SubsystemBase {
    public Field2d m_field = new Field2d();
    private final StructArrayPublisher<SwerveModuleState> publisher;

    StructPublisher<Pose3d> publisher3d = NetworkTableInstance.getDefault()
            .getStructTopic("Pose3d", Pose3d.struct).publish();
    StructArrayPublisher<Pose3d> arrayPublisher3d = NetworkTableInstance.getDefault()
            .getStructArrayTopic("PoseArray3d", Pose3d.struct).publish();

    Pose3d poseA = new Pose3d();
    Pose3d poseB = new Pose3d();

    public static final Translation2d flModuleOffset = new Translation2d(0.4, 0.4);
    public static final Translation2d frModuleOffset = new Translation2d(0.4, -0.4);
    public static final Translation2d blModuleOffset = new Translation2d(-0.4, 0.4);
    public static final Translation2d brModuleOffset = new Translation2d(-0.4, -0.4);
    public static final double maxModuleSpeed = 4.5; // M/S

    private SimSwerveModule[] modules;
    private SwerveDriveKinematics m_kinematics;
    private SwerveDriveOdometry m_odometry;

    private SimGyro m_gyro;

    public static final double speedMultiplier = 3.0;
    public static final double angleMultiplier = 3.0;
    public double newthrottleValue;

    /**
     * Constructor. Set up simulation gyro and simulation swerve modules. Also
     * initialize
     * kinematics, odometry and SmartDashboard with the field.
     */
    public SwerveSubsystemSim() {
        m_gyro = new SimGyro();

        modules = new SimSwerveModule[] {
                new SimSwerveModule(),
                new SimSwerveModule(),
                new SimSwerveModule(),
                new SimSwerveModule()
        };

        m_kinematics = new SwerveDriveKinematics(
                flModuleOffset,
                frModuleOffset,
                blModuleOffset,
                brModuleOffset);

        // Creating my m_odometry object from the m_kinematics object, initial wheel
        // positions and starting pose.
        m_odometry = new SwerveDriveOdometry(m_kinematics,
                m_gyro.getRotation2d(),
                getModulePositions(),
                new Pose2d(1.85, 4.98, new Rotation2d())); // Starting Pose.

        SmartDashboard.putData("Field", m_field);

        publisher = NetworkTableInstance.getDefault()
                .getStructArrayTopic("/SwerveStates", SwerveModuleState.struct).publish();

        publisher3d.set(poseA);
        arrayPublisher3d.set(new Pose3d[] {poseA, poseB});

        SmartDashboard.putNumber("Speed Multiplier", speedMultiplier);
        SmartDashboard.putNumber("Angle Multiplier", angleMultiplier);
    }

    /**
     * This is where the robot wakes up the swerve drivetrain to process latest
     * inputs.
     */
    public void periodic() {
        // Update the simulated m_gyro, not needed in a real project
        m_gyro.updateRotation(getSpeeds().omegaRadiansPerSecond);

        m_odometry.update(m_gyro.getRotation2d(), getModulePositions());

        m_field.setRobotPose(getPose());

        // Periodically send a set of module states
        publisher.set(getModuleStates());
    }

    /**
     * Get the current pose from the odometry.
     * 
     * @return The current pose from the odometry.
     */
    public Pose2d getPose() {
        return m_odometry.getPoseMeters();
    }

    /**
     * Update the current pose of the swerve modules.
     * 
     * @param pose The pose to set to.
     */
    public void resetPose(Pose2d pose) {
        m_odometry.resetPosition(m_gyro.getRotation2d(), getModulePositions(), pose);
    }

    /**
     * Get the current chassis speed.
     * 
     * @return The current chassis speed.
     */
    public ChassisSpeeds getSpeeds() {
        return m_kinematics.toChassisSpeeds(getModuleStates());
    }

    /**
     * Update the swerve modules based on the current chassis speed.
     * 
     * @param fieldRelativeSpeeds The current chassis speed to update the swerve
     *                            modules to.
     */
    public void driveFieldRelative(ChassisSpeeds fieldRelativeSpeeds) {
        driveRobotRelative(ChassisSpeeds.fromFieldRelativeSpeeds(fieldRelativeSpeeds, getPose().getRotation()));
    }

    /**
     * Update the swerve modules based on the current chassis speed.
     * 
     * @param robotRelativeSpeeds The current chassis speed to update the swerve
     *                            modules to.
     */
    public void driveRobotRelative(ChassisSpeeds robotRelativeSpeeds) {
        ChassisSpeeds targetSpeeds = ChassisSpeeds.discretize(robotRelativeSpeeds, 0.02);

        SwerveModuleState[] targetStates = m_kinematics.toSwerveModuleStates(targetSpeeds);

        setModuleStates(targetStates);
    }

    /**
     * Set the states of each swerve module.
     * 
     * @param targetStates The states to set the swerve modules to.
     */
    public void setModuleStates(SwerveModuleState[] targetStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(targetStates, maxModuleSpeed);

        for (int i = 0; i < modules.length; i++) {
            modules[i].setTargetState(targetStates[i]);
        }
    }

    /**
     * Get the states of each swerve module.
     * 
     * @return The state of each swerve module.
     */
    public SwerveModuleState[] getModuleStates() {
        SwerveModuleState[] states = new SwerveModuleState[modules.length];

        for (int i = 0; i < modules.length; i++) {
            states[i] = modules[i].getState();
        }

        return states;
    }

    /**
     * Get the position of each swerve module.
     * 
     * @return The position of each swerve module.
     */
    public SwerveModulePosition[] getModulePositions() {
        SwerveModulePosition[] positions = new SwerveModulePosition[modules.length];

        for (int i = 0; i < modules.length; i++) {
            positions[i] = modules[i].getPosition();
        }

        return positions;
    }

    /**
     * Get the norm, or distance from the origin to the translation.
     * 
     * @return The norm, or distance from the origin to the translation.
     */
    public double getNorm() {
        return flModuleOffset.getNorm();
    }

    /**
     * Get the maximum speed of the swerve modules.
     *
     * @return The maximum speed of the swerve module.
     */
    public double getMaxModuleSpeed() {
        return maxModuleSpeed;
    }

    /**
     * Prints the pose. Used for debugging.
     */
    public void printPose() {
        Pose2d m_pose = getPose();
        System.out.println("x: " + m_pose.getX() + ", y: " + m_pose.getY());
    }

    /**
     * Prints the states of each swerve module. Used for debugging.
     */
    public void printModuleStates(SwerveModuleState[] states) {
        for (int i = 0; i < states.length; i++) {
            System.out.println(states[i]);
        }
    }

    /**
     * Prints the position of each swerve module. Used for debugging.
     */
    public void printModulePositions() {
        for (int i = 0; i < modules.length; i++) {
            System.out.println(modules[i].getPosition());
        }
    }

    /**
     * Method to drive the robot using joystick info.
     *
     * @param xSpeed        Speed of the robot in the x direction (forward).
     * @param ySpeed        Speed of the robot in the y direction (sideways).
     * @param rot           Angular rate of the robot.
     * @param fieldRelative Whether the provided x and y speeds are relative to the
     *                      field.
     * @param rateLimit     Whether to enable rate limiting for smoother control.
     */
    public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative, boolean rateLimit) {
        var swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(
                ChassisSpeeds.discretize(
                        fieldRelative
                                ? ChassisSpeeds.fromFieldRelativeSpeeds(
                                        xSpeed, ySpeed, rot, m_gyro.getRotation2d())
                                : new ChassisSpeeds(xSpeed, ySpeed, rot),
                        0.02));

        SwerveDriveKinematics.desaturateWheelSpeeds(
                swerveModuleStates, (DriveConstants.kMaxSpeedMetersPerSecond));

        setModuleStates(swerveModuleStates);
    }

  /**
   * Sets the wheels into an X formation to prevent movement.
   */
  public void setX() {
    //modules[0].setTargetState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
    //modules[1].setTargetState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    //modules[2].setTargetState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    //modules[3].setTargetState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));

    SwerveModuleState[] states = new SwerveModuleState[modules.length];

    states[0] = new SwerveModuleState(0, Rotation2d.fromDegrees(45));
    states[1] = new SwerveModuleState(0, Rotation2d.fromDegrees(-45));
    states[2] = new SwerveModuleState(0, Rotation2d.fromDegrees(-45));
    states[3] = new SwerveModuleState(0, Rotation2d.fromDegrees(45));

    setModuleStates(states);
  }

  public void setZero(){
    //modules[0].setTargetState(new SwerveModuleState(0, Rotation2d.fromDegrees(0)));
    //modules[1].setTargetState(new SwerveModuleState(0, Rotation2d.fromDegrees(0)));
    //modules[2].setTargetState(new SwerveModuleState(0, Rotation2d.fromDegrees(0)));
    //modules[3].setTargetState(new SwerveModuleState(0, Rotation2d.fromDegrees(0)));

    SwerveModuleState[] states = new SwerveModuleState[modules.length];

    states[0] = new SwerveModuleState(0, Rotation2d.fromDegrees(0));
    states[1] = new SwerveModuleState(0, Rotation2d.fromDegrees(0));
    states[2] = new SwerveModuleState(0, Rotation2d.fromDegrees(0));
    states[3] = new SwerveModuleState(0, Rotation2d.fromDegrees(0));

    setModuleStates(states);
  }

    /**
     * Basic simulation of a swerve module, will just hold its current state and not
     * use any hardware
     */
    class SimSwerveModule {
        //public static final double speedMetersPerSecond = 10.0;
        //public Rotation2d angle = Rotation2d.fromDegrees(0);

        private SwerveModulePosition currentPosition = new SwerveModulePosition();
        private SwerveModuleState currentState = new SwerveModuleState();

        public SwerveModulePosition getPosition() {
            return currentPosition;
        }

        public SwerveModuleState getState() {
            return currentState;
        }

        public void setTargetState(SwerveModuleState targetState) {
            double multiplier = SmartDashboard.getNumber("Speed Multiplier", speedMultiplier);
            // Optimize the state
            currentState = SwerveModuleState.optimize(targetState, currentState.angle);

            currentPosition = new SwerveModulePosition(
                    currentPosition.distanceMeters + (currentState.speedMetersPerSecond * 0.02 * multiplier), currentState.angle);
        }
    }

    /**
     * Basic simulation of a m_gyro, will just hold its current state and not use
     * any
     * hardware
     */
    class SimGyro {
        private Rotation2d currentRotation = new Rotation2d();

        public Rotation2d getRotation2d() {
            return currentRotation;
        }

        public void updateRotation(double angularVelRps) {
            double multiplier = SmartDashboard.getNumber("Angle Multiplier", angleMultiplier);
            currentRotation = currentRotation.plus(new Rotation2d(angularVelRps * 0.02 * multiplier));
        }
    }
}
