package frc.robot.Constants;

public class AprilTagConstants {
    public static final float[] tag1Pos = {0,0};
    public static final float[] tag2Pos = {0,0};
    public static final float[] tag3Pos = {0,0};
    public static final float[] tag4Pos = {0,0};
    public static final float[] tag5Pos = {0,0};
    public static final float[] tag6Pos = {0,0};
    public static final float[] tag7Pos = {0,0};
    public static final float[] tag8Pos = {0,0};
    public static final float[] tag9Pos = {0,0};
    public static final float[] tag10Pos = {0,0};
    public static final float[] tag11Pos = {0,0};
    public static final float[] tag12Pos = {0,0};
    public static final float[] tag13Pos = {0,0};
    public static final float[] tag14Pos = {0,0};
    public static final float[] tag15Pos = {0,0};
    public static final float[] tag16Pos = {0,0};
    public static final float[] tag17Pos = {0,0};
    public static final float[] tag18Pos = {0,0};
    public static final float[] tag19Pos = {0,0};
    public static final float[] tag20Pos = {0,0};
    public static final float[] tag21Pos = {0,0};
    public static final float[] tag22Pos = {0,0};
    
    public static float[] GetTagPosition(int id){
        switch(id){
            case 1:
                return(tag1Pos);
            case 2:
                return(tag2Pos);
            case 3:
                return(tag3Pos);
            case 4:
                return(tag4Pos);
            case 5:
                return(tag5Pos);
            case 6:
                return(tag6Pos);
            case 7:
                return(tag7Pos);
            case 8:
                return(tag8Pos);
            case 9:
                return(tag9Pos);
            case 10:
                return(tag10Pos);
            case 11:
                return(tag11Pos);
            case 12:
                return(tag12Pos);
            case 13:
                return(tag13Pos);
            case 14:
                return(tag14Pos);
            case 15:
                return(tag15Pos);
            case 16:
                return(tag16Pos);
            case 17:
                return(tag17Pos);
            case 18:
                return(tag18Pos);
            case 19:
                return(tag19Pos);
            case 20:
                return(tag20Pos);
            case 21:
                return(tag21Pos);
            case 22:
                return(tag22Pos);
            default:
                // Error case, this should never happen
                // I just didn't feel like making an exception
                return(new float[]{69420,69420});
        }
    }
}
