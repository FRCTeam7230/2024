package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
/* TODO:
 * - set up the general limelight [x (maybe? i dont know ill test with the limelight)]
 * - set up the apriltag detection [--68%--]
 * - set up the red blue team integration [x]
 * - test this integration with the apriltags and the red blue team chooserand integration []
 */
public class Limelight {
    private static NetworkTableInstance table = null;
    public static double targetX, targetY, visionTargets, targetID;
    public static double targetArea;
    
    public static void limelight(String teamColor) {
        // this is where we set which pipeline to use based on the team color that gets passed here, the
        // rest of the code works off of this
        if (teamColor == "red") {
            NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").setNumber(0); // pipeline 0 is for red team
        } else if (teamColor == "blue") {
            NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").setNumber(1); // pipeline 1 is for blue team
        }
    }
    public static void refreshData() {
        // this is where we refresh the limelight data
        NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
        NetworkTableEntry tx = table.getEntry("tx");
        NetworkTableEntry ty = table.getEntry("ty");
        NetworkTableEntry tv = table.getEntry("tv");
        NetworkTableEntry tid = table.getEntry("tid");
        
        // read values periodically
        visionTargets = tv.getDouble(0.0);
        targetX = tx.getDouble(0.0);
        targetY = ty.getDouble(0.0);
        targetID = tid.getDouble(0.0);
        // add tid here
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
        double h1 = 0; // height of the limelight from the ground
        double h2 = 0; // height of the apriltag from the ground
        double a1 = 0; // angle of the limelight
        double a2 = 0; // angle of the apriltag
        double d = (h2 - h1) / Math.tan(a1 + a2);
        return d;
    } // this is where we calculate the distance to the apriltag

    //public static boolean proceedMoving(double objAreaRequired) {
        // refreshData();
        // this method's placement might be temporary, ill be working apriltag detection and will deem later if needed
}