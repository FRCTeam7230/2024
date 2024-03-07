package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Mechanisms;

// this is the file where the gyro gets its data put and refreshed, 
// to make an attempt to keep things clean

public class GyroSubsystem extends SubsystemBase {
    private static AHRS gyro = Mechanisms.gyro;
    public static double gyroAngle = gyro.getRoll();
    public static double FetchGyroData() {
        System.out.println("Roll" + gyroAngle);
        return gyroAngle;
        // System.out.println("Pitch" + Mechanisms.gyro.getPitch());
        // System.out.println("Yaw" + Mechanisms.gyro.getYaw());
    }
}
