// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Autonomous extends SubsystemBase {
    /** Creates a new ExampleSubsystem. */
    public Autonomous() {
    }

    /**
     * 1. Move to speaker
     * 2. Align to april tags
     * 3. Score preloaded in speaker
     * 4. Drive to note
     * 5. Intake note
     * 6. Drive back to alliance zone (optional, in case if we have more time)
     * 
     */

    /**
     * Example command factory method.
     *
     * @return a command
     */

    public CommandBase MoveToSpeaker(int startingPosition, double startDelay) {
        // Inline construction of command goes here.
        // Subsystem::RunOnce implicitly requires `this` subsystem.

        /**
         * Starting Positions:
         * 0 = leftmost
         * 1 = middle
         * 2 = rightmost
         */

        return runOnce(
                () -> {
                    // code to wait for startDelay seconds before moving
                    // we get this startDelay by coordinating with alliances and their planned
                    // timing

                    if (startingPosition == 0) {
                        // code to move right, rotate to face front to speaker (90 degrees)
                    } else if (startingPosition == 1) {
                        // same but left
                    } else {
                        // same but move left more
                    }
                });
    }

  public CommandBase AlignToApriltag(double distX, double distY){
    return runOnce(() -> {
        //
    });
  }

    /**
     * An example method querying a boolean state of the subsystem (for example, a
     * digital sensor).
     *
     * @return value of some boolean subsystem state, such as a digital sensor.
     */
    public boolean exampleCondition() {
        // Query some boolean state, such as a digital sensor.
        return false;
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
    }
}
