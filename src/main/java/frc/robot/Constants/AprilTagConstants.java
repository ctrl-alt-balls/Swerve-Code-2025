package frc.robot.Constants;

public class AprilTagConstants {
    //X (in metres) , Z (in metres) , YRot (in degrees)
    public static final float[] tag1Pos = {16.697198f,0.65532f,126};
    public static final float[] tag2Pos = {16.697198f,7.39648f,234};
    public static final float[] tag3Pos = {11.56081f,8.05561f,270};
    public static final float[] tag4Pos = {9.27608f,6.137656f,0};
    public static final float[] tag5Pos = {9.27608f,1.914906f,0};
    public static final float[] tag6Pos = {13.474446f,3.306318f,300};
    public static final float[] tag7Pos = {13.890498f,4.0259f,0};
    public static final float[] tag8Pos = {13.474446f,4.745482f,60};
    public static final float[] tag9Pos = {12.643358f,4.745482f,120};
    public static final float[] tag10Pos = {12.227306f,4.0259f,180};
    public static final float[] tag11Pos = {12.643358f,3.306318f,240};
    public static final float[] tag12Pos = {0.851154f,0.65532f,54};
    public static final float[] tag13Pos = {0.851154f,7.39648f,306};
    public static final float[] tag14Pos = {8.272272f,6.137656f,180};
    public static final float[] tag15Pos = {8.272272f,1.914906f,180};
    public static final float[] tag16Pos = {5.987542f,-0.00381f,90};
    public static final float[] tag17Pos = {4.073906f,3.306318f,240};
    public static final float[] tag18Pos = {3.6576f,4.0259f,180};
    public static final float[] tag19Pos = {4.073906f,4.745482f,120};
    public static final float[] tag20Pos = {4.90474f,4.745482f,60};
    public static final float[] tag21Pos = {5.321046f,4.0259f,0};
    public static final float[] tag22Pos = {4.90474f,3.306318f,300};
    
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
                return(new float[]{69420,69420,69420});
        }
    }
}
