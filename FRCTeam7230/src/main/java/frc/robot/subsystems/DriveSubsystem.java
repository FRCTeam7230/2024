// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.WPIUtilJNI;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.kauailabs.navx.frc.AHRS;

import static frc.robot.Constants.DriveConstants.*;
import frc.robot.Mechanisms;
import frc.robot.Constants.DriveConstants;
import frc.utils.SwerveUtils;


public class DriveSubsystem extends SubsystemBase {
    public Field2d m_field = new Field2d();
  
  // Create MAXSwerveModules
  private final MAXSwerveModule frontLeft = Mechanisms.m_frontLeft;
  private final MAXSwerveModule frontRight = Mechanisms.m_frontRight;
  private final MAXSwerveModule rearLeft = Mechanisms.m_rearLeft;
  private final MAXSwerveModule rearRight = Mechanisms.m_rearRight;

    
  // The gyro sensor
  private final AHRS m_gyro = Mechanisms.m_gyro;
  private double gyroAngle = 0.0;
  

  // Slew rate filter variables for controlling lateral acceleration
  private double m_currentRotation = 0.0;
  private double m_currentTranslationDir = 0.0;
  private double m_currentTranslationMag = 0.0;
  private double m_prevTime = WPIUtilJNI.now() * 1e-6;
  // private double speedMultiplierCommanded = 0.0;
  


  private Translation2d frontrightWheelMeters = new Translation2d(0.3429,0.3429); 
  private Translation2d frontleftWheelMeters = new Translation2d(-0.3429,0.3429); 
  private Translation2d rearrightWheelMeters = new Translation2d(0.3429,-0.3429); 
  private Translation2d rearleftWheelMeters = new Translation2d(-0.3429,-0.3429); 
  
  private SwerveDriveKinematics kinematics = new SwerveDriveKinematics(frontrightWheelMeters,frontleftWheelMeters,rearrightWheelMeters,rearleftWheelMeters);
  
  private MAXSwerveModule[] modules = new MAXSwerveModule[]{
  frontRight,
  frontLeft,
  rearLeft,
  rearRight
  };

  // Odometry class for tracking robot pose
  SwerveDriveOdometry m_odometry = new SwerveDriveOdometry( 
      kDriveKinematics, 
      Rotation2d.fromDegrees(m_gyro.getPitch()), 
      new SwerveModulePosition[] {
          frontLeft.getPosition(),
          frontRight.getPosition(),
          rearLeft.getPosition(),
          rearRight.getPosition()
      });

    
  


  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {
	SmartDashboard.putData("Field", m_field);
  }

    public double fetchGyroData() {
      gyroAngle = m_gyro.getYaw();
        return gyroAngle;
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
      return kinematics.toChassisSpeeds(getModuleStates());
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

      SwerveModuleState[] targetStates = kinematics.toSwerveModuleStates(targetSpeeds);

      setModuleStates(targetStates);
  }

  /**
   * Get the norm, or distance from the origin to the translation.
   * 
   * @return The norm, or distance from the origin to the translation.
   */
  public double getNorm() {
    return frontrightWheelMeters.getNorm();
  }

  /**
   * Get the maximum speed of the swerve modules.
   *
   * @return The maximum speed of the swerve module.
   */
  public double getMaxModuleSpeed() {
      return kMaxSpeedMetersPerSecond;
  }

  @Override
  public void periodic() {
    // Update the odometry in the periodic block
    m_odometry.update(
        Rotation2d.fromDegrees(m_gyro.getPitch()),
        new SwerveModulePosition[] {
            frontLeft.getPosition(),
            frontRight.getPosition(),
            rearLeft.getPosition(),
            rearRight.getPosition()
        });

	m_field.setRobotPose(getPose());
  }

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    m_odometry.resetPosition(
        Rotation2d.fromDegrees(m_gyro.getPitch()),
        new SwerveModulePosition[] {
            frontLeft.getPosition(),
            frontRight.getPosition(),
            rearLeft.getPosition(),
            rearRight.getPosition()
        },
        pose);
  }

  //Prints the angular position of the swerve drive modules
  public void printModulePositions(){
    
    System.out.println(frontLeft.getPosition());
    System.out.println(frontRight.getPosition());
    System.out.println(rearLeft.getPosition());
    System.out.println(rearRight.getPosition());

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
   * @param circlingMode    Switches toggle to rotating in a circle
   */
   
   //speedMultiplier slows/speeds up the robot in a controlled manner: from 0.5x - 2x of current speed     

  public void drive(double xSpeed, double ySpeed, double rot, double speedMultiplier, boolean fieldRelative, boolean rateLimit, boolean circlingMode) {
    
    double xSpeedCommanded;
    double ySpeedCommanded;
    SlewRateLimiter m_magLimiter;
    SlewRateLimiter m_rotLimiter;
    
    // if (!circlingMode){
    //   if (speedMultiplier >= 0){
    //       speedMultiplierCommanded = Math.pow(speedMultiplier, 2) + 1;
    //     }
    //   else{
    //     speedMultiplierCommanded = -0.5*Math.pow(speedMultiplier, 2) + 1;
    //   }

      m_magLimiter = new SlewRateLimiter(kMagnitudeSlewRate /* * speedMultiplierCommanded */);
      m_rotLimiter = new SlewRateLimiter(kRotationalSlewRate /* * speedMultiplierCommanded */);

      if (rateLimit) {
        // Convert XY to polar for rate limiting
        double inputTranslationDir = Math.atan2(ySpeed, xSpeed);
        double inputTranslationMag = Math.sqrt(Math.pow(xSpeed, 2) + Math.pow(ySpeed, 2));

        // Calculate the direction slew rate based on an estimate of the lateral acceleration
        double directionSlewRate;
        if (m_currentTranslationMag != 0.0) {
          directionSlewRate = Math.abs(kDirectionSlewRate /* * speedMultiplierCommanded */ / m_currentTranslationMag);
        } else {
          directionSlewRate = 500.0; //some high number that means the slew rate is effectively instantaneous
        }
        

        double currentTime = WPIUtilJNI.now() * 1e-6;
        double elapsedTime = currentTime - m_prevTime;
        double angleDif = SwerveUtils.AngleDifference(inputTranslationDir, m_currentTranslationDir);
        if (angleDif < 0.45*Math.PI) {
          m_currentTranslationDir = SwerveUtils.StepTowardsCircular(m_currentTranslationDir, inputTranslationDir, directionSlewRate * elapsedTime);
          m_currentTranslationMag = m_magLimiter.calculate(inputTranslationMag);
        }
        else if (angleDif > 0.85*Math.PI) {
          if (m_currentTranslationMag > 1e-4) { //some small number to avoid floating-point errors with equality checking
            // keep currentTranslationDir unchanged
            m_currentTranslationMag = m_magLimiter.calculate(0.0);
          }
          else {
            m_currentTranslationDir = SwerveUtils.WrapAngle(m_currentTranslationDir + Math.PI);
            m_currentTranslationMag = m_magLimiter.calculate(inputTranslationMag);
          }
        }
        else {
          m_currentTranslationDir = SwerveUtils.StepTowardsCircular(m_currentTranslationDir, inputTranslationDir, directionSlewRate * elapsedTime);
          m_currentTranslationMag = m_magLimiter.calculate(0.0);
        }
        m_prevTime = currentTime;
        
        xSpeedCommanded = m_currentTranslationMag * Math.cos(m_currentTranslationDir);
        ySpeedCommanded = m_currentTranslationMag * Math.sin(m_currentTranslationDir);
        m_currentRotation = m_rotLimiter.calculate(rot);


      } else {
        xSpeedCommanded = xSpeed;
        ySpeedCommanded = ySpeed;
        m_currentRotation = rot;
      }

      // Convert the commanded speeds into the correct units for the drivetrain
      double xSpeedDelivered = xSpeedCommanded * kMaxSpeedMetersPerSecond /* * speedMultiplierCommanded */;
      double ySpeedDelivered = ySpeedCommanded * kMaxSpeedMetersPerSecond /* * speedMultiplierCommanded */;
      double rotDelivered = m_currentRotation * kMaxAngularSpeed /* * speedMultiplierCommanded */;

      // System.out.println(xSpeed);//logging inputs
      // System.out.println(ySpeed);
      // System.out.println(rot);
      

      var swerveModuleStates = kDriveKinematics.toSwerveModuleStates(
        fieldRelative
              ? ChassisSpeeds.fromRobotRelativeSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered, Rotation2d.fromDegrees(m_gyro.getPitch()))
              : new ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered));
      SwerveDriveKinematics.desaturateWheelSpeeds(
          swerveModuleStates, kMaxSpeedMetersPerSecond /* * speedMultiplierCommanded */);
      frontLeft.setDesiredState(swerveModuleStates[0]);
      frontRight.setDesiredState(swerveModuleStates[1]);
      rearLeft.setDesiredState(swerveModuleStates[2]);
      rearRight.setDesiredState(swerveModuleStates[3]);
    }
  // }

  public void testButton() {
    // System.out.println("the test button was pressed"); // i added logic just in case 
    System.out.println(m_gyro.getAngle());
  }
  /**
   * Sets the wheels into an X formation to prevent movement.
   */
  public void setX() {
    frontLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
    frontRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    rearLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    rearRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
  }

  //input directionToRotate as negative one for left, positive one for right
  public void rotateUntil(double rotateSpeed){
      setModuleStates(kDriveKinematics.toSwerveModuleStates(
        new ChassisSpeeds(0,0,rotateSpeed /* * speedMultiplierCommanded */)));
  }

  /**
   * Sets the swerve ModuleStates.
   *
   * @param desiredStates The desired SwerveModule states.
   */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(
        desiredStates, kMaxSpeedMetersPerSecond /* * speedMultiplierCommanded */);
    frontLeft.setDesiredState(desiredStates[0]);
    frontRight.setDesiredState(desiredStates[1]);
    rearLeft.setDesiredState(desiredStates[2]);
    rearRight.setDesiredState(desiredStates[3]);
  }
      
  public void circlingDrive(boolean circlingMode, double rotateSpeed, double distance, double visionAngle) {
    rotateSpeed = rotateSpeed /* * speedMultiplierCommanded */;
    ChassisSpeeds chassisSpeeds = new ChassisSpeeds(rotateSpeed, rotateSpeed, rotateSpeed);
    Rotation2d angle = new Rotation2d(visionAngle); //vision gives us this angle
    Translation2d distanceAngle = new Translation2d(distance, angle); //vision gives us distance
    
    setModuleStates(kinematics.toSwerveModuleStates(chassisSpeeds, distanceAngle));
  }
  
  

  /** Resets the drive encoders to currently read a position of 0. */
  public void resetEncoders() {
    frontLeft.resetEncoders();
    rearLeft.resetEncoders();
    frontRight.resetEncoders();
    rearRight.resetEncoders();
  }

  /** Zeroes the heading of the robot. */
  public void zeroHeading() {
    m_gyro.reset();
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
   */
  public double getHeading() {
    return Rotation2d.fromDegrees(m_gyro.getPitch()).getDegrees();
  }

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() {
    return m_gyro.getRate() * (kGyroReversed ? -1.0 : 1.0);
  }
}
