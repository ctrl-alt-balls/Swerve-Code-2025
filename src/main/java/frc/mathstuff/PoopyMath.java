package frc.mathstuff;

public class PoopyMath {
    
    public static double[] rotateVector2(float x, float y, double degrees){
        double r = Math.toRadians(degrees);
        double rotatedX = x*Math.cos(r)-y*Math.sin(r);
        double rotatedY = y*Math.cos(r)+x*Math.sin(r);
        return(new double[]{rotatedX,rotatedY});
    }
}
