package frc.robot.utils;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public final class GeometryUtil {
    public static Transform3d toTransform(Pose3d pose) {
        return new Transform3d(pose.getTranslation(), pose.getRotation());
    }

    public static Twist2d multiplyTwist(Twist2d twist, double factor) {
        return new Twist2d(twist.dx * factor, twist.dy * factor, twist.dtheta * factor);
    }
    public static double distanceToLimit(double elevatorExtension, double shoulderAngle) {
        final double defaultShoulderAngle = -58.0;  // not actually used in this code; just for reference
        final double defaultShoulderClearance = 22.19;  // Distance from shoulder center to 48" upper limit
        final double elevatorAngle = 16.5;       // Angle of elevator relative to vertical
        final int numObstructions = 3;       // Trap wheel, idler roller, shooter compliant wheel
        final double[] obstructionAngles = { -57.33, -37.56, 2.6 };
        final double[] obstructionDistances = { 18, 15.98, 8.12 };
        final double[] obstructionRadii = { 3.94/2.0, 1.50/2.0, 4.0/2.0 };
    
        double shoulderClearance = defaultShoulderClearance - Math.cos(Math.toRadians(elevatorAngle)) * elevatorExtension;
    
        // get the maximum height above the shoulder of all obstructions
        double maximumHeight = defaultShoulderClearance-48.0; // height of floor
        for(int i=0;i<numObstructions;i++) {
            double obstructionCenterY = obstructionDistances[i]*Math.sin(Math.toRadians((-shoulderAngle+60.5) + obstructionAngles[i]));
            maximumHeight = Math.max(maximumHeight, obstructionCenterY + obstructionRadii[i]);
            String num = String.valueOf(i);
            // SmartDashboard.putNumber("maxHeight" + num, obstructionCenterY);
        }
    
        //SmartDashboard.putNumber("Clearance", shoulderClearance - maximumHeight);

        return shoulderClearance - maximumHeight;
    }
}
