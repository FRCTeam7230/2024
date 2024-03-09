package frc.robot;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Joystick;
import frc.robot.subsystems.MAXSwerveModule;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.DutyCycleEncoder;

import com.revrobotics.CANSparkMax;
import static com.revrobotics.CANSparkLowLevel.MotorType.*;

import static frc.robot.Constants.OperatorConstants.*;
import static frc.robot.Constants.DriveConstants.*;
import static frc.robot.Constants.ShooterConstants.*;
import static frc.robot.Constants.IntakeConstants.*;
import static frc.robot.Constants.ClimbingConstants.*;

public class Mechanisms {
    //General
    public static final Joystick m_driverController = new Joystick(kDriverControllerPort);
    public static final Joystick m_mechanismsController = new Joystick(kMechanismsControllerPort);

    //Swerve Drive Subsystem
public static final MAXSwerveModule m_frontLeft = new MAXSwerveModule(
      kFrontLeftDrivingCanId,
      kFrontLeftTurningCanId,
      kFrontLeftChassisAngularOffset);

  public static final MAXSwerveModule m_frontRight = new MAXSwerveModule(
      kFrontRightDrivingCanId,
      kFrontRightTurningCanId,
      kFrontRightChassisAngularOffset);

  public static final MAXSwerveModule m_rearLeft = new MAXSwerveModule(
      kRearLeftDrivingCanId,
      kRearLeftTurningCanId,
      kBackLeftChassisAngularOffset);

  public static final MAXSwerveModule m_rearRight = new MAXSwerveModule(
      kRearRightDrivingCanId,
      kRearRightTurningCanId,
      kBackRightChassisAngularOffset);

    //Shooter Subsystem
    public static final CANSparkMax m_rightShooterMotor = new CANSparkMax(kRightShooterMotorId, kBrushless);
    public static final CANSparkMax m_leftShooterMotor = new CANSparkMax(kLeftShooterMotorId, kBrushless);
    public static final DigitalInput m_noteBeamSensor = new DigitalInput(kNoteBeamSensorId);

    public static final CANSparkMax m_shooterPivotMotor = new CANSparkMax(kShooterPivotMotorId, kBrushless);
    public static final DutyCycleEncoder m_pivotEncoder = new DutyCycleEncoder(kPivotEncoderId);
    public static final DigitalInput m_upperLimitSwitch = new DigitalInput(kUpperLimitSwitchId);    

    //Intake Subsystem
    public static final CANSparkMax m_intakeMotor = new CANSparkMax(kIntakeMotorId, kBrushless);
    public static final CANSparkMax m_ShooterIntakeMotor = new CANSparkMax(kShooterIntakeMotorId, kBrushless);

    //Climbing Subsystem
    public static final CANSparkMax m_climbingMotor = new CANSparkMax(kClimbingMotorId, kBrushless);
    public static final DutyCycleEncoder m_climberEncoder = new DutyCycleEncoder(kClimbingEncoderId);

    public static final AHRS m_gyro = new AHRS();
}
