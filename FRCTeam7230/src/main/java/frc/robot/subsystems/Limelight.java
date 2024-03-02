package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class Limelight{
    private static NetworkTableInstance table = NetworkTableInstance.getDefault();
    public static double targetX, targetY, visionTargets, targetID;
    public static double targetArea;


    public static void limelight(String teamColor) {
        // this is where we set which pipeline to use based on the team color that gets passed here, the
        // rest of the code works off of this
        if ("red".equals(teamColor)) {
            table.getEntry("pipeline").setNumber(1); // pipeline 0 is for red team
        } else {
            table.getEntry("pipeline").setNumber(0); // pipeline 1 is for blue team
        }
        
        NetworkTableInstance.getDefault().flush(); // Force an immediate update
    }
    
    public static void refreshData() {
        // this is where we refresh the limelight data
        NetworkTableInstance tableInstance = NetworkTableInstance.getDefault();  // Get the default instance
        NetworkTable table = tableInstance.getTable("limelight");
        NetworkTableEntry tx = table.getEntry("tx");
        NetworkTableEntry ty = table.getEntry("ty");
        NetworkTableEntry tv = table.getEntry("tv");
        NetworkTableEntry tid = table.getEntry("tid");
        
        // read values periodically
        visionTargets = tv.getDouble(0.0);
        targetX = tx.getDouble(0.0);
        targetY = ty.getDouble(0.0);
        targetID = tid.getDouble(0.0);
    }

    public static void manualLimelightOff(boolean manualLayout){
        if (manualLayout){
            NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").setNumber(3);
        }

    } // this is for when we want to manually turn off limelight

    public static void initializeLimelightOff(){
        NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").setNumber(3);
    } // this sets up limelight to be turned off

    public static double getTargetAngleX() {
        refreshData();
        return targetX;
    } // this gets the apriltag's x angle offset

    public static double getTargetAngleY(){
        refreshData();
        return targetY;
    } // this gets the apriltag's y angle offset

    public static double getTargetID(){
        refreshData();
        return targetID;
    } // this gets the apriltag's ID

    

    public static double apriltagDistance() {
        // this is where we calculate the distance to the apriltag
        double h1 = 9.120; // height of the limelight from the ground
        double h2 = 50.4; // height of the apriltag from the ground
        double a1deg = 22.85; // angle of the limelight
        double a2deg = 0; // angle of the apriltag
        double a1 = Math.toRadians(a1deg);
        double a2 = Math.toRadians(a2deg);
        double d = (h2 - h1) / Math.tan(a1 + a2);
        return d;
        
    } // this is where we calculate the distance to the apriltag
}