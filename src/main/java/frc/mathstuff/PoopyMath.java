package frc.mathstuff;

import frc.robot.Networking.UDPServer.Double3;

public class PoopyMath {
    
    public static double[] rotateVector2(float x, float y, double degrees){
        double r = Math.toRadians(degrees);
        double rotatedX = x*Math.cos(r)-y*Math.sin(r);
        double rotatedY = y*Math.cos(r)+x*Math.sin(r);
        return(new double[]{rotatedX,rotatedY});
    }

    public static double getDistance(Double3 pose1, Double3 pose2){
        return Math.sqrt(((pose2.x-pose1.x)*(pose2.x-pose1.x))+((pose2.y-pose1.y)*(pose2.y-pose1.y)));
    }
}
