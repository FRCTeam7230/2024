// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Counter;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

public class ShooterSubsystem extends SubsystemBase {

  /** Creates a new ExampleSubsystem. */
  Spark rotationalMotorLeft = new Spark(0);
  Spark rotationalMotorRight = new Spark(0);

  Spark flyWheelMotor = new Spark(1);

  double motorRotateSpeed = ShooterConstants.rotationalSpeed;
  double rotateDegree = ShooterConstants.degreesPerStep;

  Counter motorCounter = new Counter();
  Counter absoluteCounter = new Counter();

  boolean inRotation = false;
  boolean inAbsoluteRotation = false;
  int targetDegree;

  public static boolean shooterOn = false;

  public ShooterSubsystem() {
  }

  /**
   * Example command factory method.
   *
   * @return a command
   */
  public Command exampleMethodCommand() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          /* one-time action goes here */
        });
  }

  public Command RotateToAngle(int degree){
    return runOnce(() -> {
      inAbsoluteRotation = true;
      targetDegree = degree;
    });
  }

  public Command RotateShooterUp() {
    return runOnce(
        () -> {
          motorCounter.reset();
          inRotation = true;

          rotationalMotorLeft.set(motorRotateSpeed);
          rotationalMotorRight.set(motorRotateSpeed);
        });
  }

  public Command RotateShooterDown() {
    return runOnce(
        () -> {
          motorCounter.reset();
          inRotation = true;

          rotationalMotorLeft.set(-motorRotateSpeed);
          rotationalMotorRight.set(-motorRotateSpeed);
        });
  }
  public Command StopRotation(){
    return runOnce(
      () -> {
        inRotation = false;
        rotationalMotorLeft.stopMotor();
        rotationalMotorRight.stopMotor();
      });
    
  }

  public Command StartShooter() {
    return runOnce(() -> {
      shooterOn = true;
    });
  }

  public Command StopShooter() {
    return runOnce(() -> {
      shooterOn = false;
    });
  }

  /**
   * An example method querying a boolean state of the subsystem (for example, a
   * digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */

  public int counterValue() {
    return motorCounter.get();
  }

  public int getCurrentDegree(){
    return absoluteCounter.get();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (shooterOn) {
      boolean noteIsIn = IntakeSubsystem.checkSensor();

      if (noteIsIn) {
        //DO SOME COOL STUFF
        //pivoting mechanism control
        //shooter flywheels
      }

      if (counterValue() <= -rotateDegree && inRotation) {
        inRotation = false;
        rotationalMotorLeft.stopMotor();
        rotationalMotorRight.stopMotor();
      }
      if (counterValue() >= rotateDegree && inRotation) {
        inRotation = false;
        rotationalMotorLeft.stopMotor();
        rotationalMotorRight.stopMotor();
      }

      if(inAbsoluteRotation){
        int currentDegree = getCurrentDegree();
        
        if(currentDegree < targetDegree){
          //it needs to go up
          RotateShooterUp();
        }else if(currentDegree > targetDegree){
          //it needs to go down
          RotateShooterDown();
        }else{
          inAbsoluteRotation = false;
          rotationalMotorLeft.stopMotor();
          rotationalMotorRight.stopMotor();
        }
      }
    }
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
