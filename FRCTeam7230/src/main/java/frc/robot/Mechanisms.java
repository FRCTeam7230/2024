package frc.robot;

import static frc.robot.Constants.OperatorConstants.*;
import static frc.robot.Constants.DriveConstants.*;
import static frc.robot.Constants.ShooterConstants.*;
import static frc.robot.Constants.IntakeConstants.*;
import static frc.robot.Constants.ClimbingConstants.*;
import com.revrobotics.CANSparkMax;
import static com.revrobotics.CANSparkLowLevel.MotorType.*;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.DutyCycleEncoder;

public class Mechanisms {
    //General
    public static final Joystick m_driverController = new Joystick(kDriverControllerPort);
    public static final Joystick m_mechanismsController = new Joystick(kMechanismsControllerPort);

    //Swerve Drive Subsystem
    public static final CANSparkMax m_rearLeftTurningMotor = new CANSparkMax(kRearLeftTurningCanId, kBrushless);
    public static final CANSparkMax m_rearLeftDrivingMotor = new CANSparkMax(kRearLeftDrivingCanId, kBrushless);

    public static final CANSparkMax m_rearRightTurningMotor = new CANSparkMax(kRearRightTurningCanId, kBrushless);
    public static final CANSparkMax m_rearTightDrivingMotor = new CANSparkMax(kRearRightDrivingCanId, kBrushless);

    public static final CANSparkMax m_frontLeftTurningMotor = new CANSparkMax(kFrontLeftTurningCanId, kBrushless);
    public static final CANSparkMax m_frontLeftDrivingMotor = new CANSparkMax(kFrontLeftDrivingCanId, kBrushless);

    public static final CANSparkMax m_frontRightTurningMotor = new CANSparkMax(kFrontRightTurningCanId, kBrushless);
    public static final CANSparkMax m_frontRightDrivingMotor = new CANSparkMax(kFrontRightDrivingCanId, kBrushless);

    //Shooter Subsystem
    public static final CANSparkMax m_rightShooterMotor = new CANSparkMax(kRightShooterMotorId, kBrushless);
    public static final CANSparkMax m_leftShooterMotor = new CANSparkMax(kLeftShooterMotorId, kBrushless);
    public static final DigitalInput m_noteBeamSensor = new DigitalInput(kNoteBeamSensorId);

    public static final CANSparkMax m_shooterPivotMotor = new CANSparkMax(kShooterPivotMotorId, kBrushless);
    public static final DutyCycleEncoder m_pivotEncoder = new DutyCycleEncoder(kPivotEncoderId);
    public static final DigitalInput m_upperLimitSwitch = new DigitalInput(kUpperLimitSwitchId);    

    //Intake Subsystem
    public static final CANSparkMax m_intakeMotor = new CANSparkMax(kIntakeMotorId, kBrushless);
    public static final CANSparkMax m_transferToShooterMotor = new CANSparkMax(kTransferToShooterMotorId, kBrushless);

    //Climbing Subsystem
    public static final CANSparkMax m_climbingMotor = new CANSparkMax(kClimbingMotorId, kBrushless);
    public static final DutyCycleEncoder m_climberEncoder = new DutyCycleEncoder(kClimbingEncoderId);
}
