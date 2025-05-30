package frc.robot.Networking;

import java.io.IOException;
import java.net.DatagramPacket;
import java.net.DatagramSocket;
import java.net.SocketException;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.util.Arrays;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.mathstuff.PoopyMath;
import frc.robot.Constants.AprilTagConstants;
import frc.robot.subsystems.SubsystemConstants.SubsystemConstants.ScoringPoses;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class UDPServer extends SubsystemBase{

    /*
    public class AprilTagTransform{
        int id = 0;
        float posX = 0;
        float posY = 0;
        float posZ = 0;
        float rotX = 0;
        float rotY = 0;
        float rotZ = 0;
    }
    */

    static public class Double3 {
        public double x;
        public double y;
        public double z;
        public Double3(double x, double y, double z){
            this.x=x;
            this.y=y;
            this.z=z;
        }
    }

    public Double3[] path = new Double3[1];
    int pathIndex = 0;
    boolean[] atSetpoint = {false,false,false};

    public boolean enableAuto = false;
    double pidTransClamp = 1;
    double pidRotClamp = 1;
    PIDController pidTrans = new PIDController(0.5, 0, 0);
    PIDController pidRot = new PIDController(0.1, 0, 0);
    double[] pidValues = new double[3];
    double[] poseSetpoint = new double[3];
    double posSetpointTolerance = 0.2;
    double rotSetpointTolerance = 0.2;

    DatagramSocket socket;
    float[] recievedFloat = new float[3];

    double[] globalizedRecievedPose = new double[3];

    int tagNum = 0;
    //AprilTagTransform[] tagTransforms;
    double[] poseFromTag = new double[3];
    //double[] poseOffset = new double[3];
    double[] heheHawHawPoseOffset = new double[2];

    int[] id;
    float[] posX;
    float[] posY;
    float[] posZ;
    float[] rotX;
    float[] rotY;
    float[] rotZ;

    int id1 = 0;

    boolean useAprilTags = true;

    //int totalCalls = 0;

    double[] calibratedPose = new double[3];
    double[] extraAccuracyPose = new double[3];
    double[] extraAccuracyPoseHoldValue = new double[3];
    boolean motionlessCalib = false;
    int calibSamples = 5;
    int calibSampleIndex = 0;
    public boolean calibFinished = false;

    double[] rotatedRecievedFloat = new double[2];
    double rotSnapshot = 0;

    boolean takeSnapshot=false;


    // Do not touch, internal use ONLY
    double[] rotatedQuestPos = new double[2];
    double globalizeRot;

    boolean testbool = false;

    boolean pathOrSetpoint = false;
    boolean atSetpointReturnBool = false;


    private final Field2d m_field = new Field2d();

    public UDPServer(int port){
        path[0] = new Double3(0, 0, 0);
        pidTrans.setTolerance(posSetpointTolerance);
        pidRot.setTolerance(rotSetpointTolerance);
        pidRot.enableContinuousInput(-180, 180);
        SmartDashboard.putBoolean("test", testbool);
        //SmartDashboard.putData("Field", m_field);
        try{
            socket = new DatagramSocket(port);
            System.out.println("Started UDP server on port " + socket.getLocalPort());

            // New thread for UDP communication so it can run at maximum speed and not interrupt critical robot functions in case of a malfunction
            Thread t1 = new Thread(new Runnable() {
                public void run()
                {
                    while(true){
                        RecieveData();
                    }
                }
            });  
            t1.start();
        }catch(SocketException e){
            //e.printStackTrace();
            System.out.println("oopsies no server :3");
        }
    }

    public void RecieveData(){
        // This is the array that holds all of the bytes from the transmission. 
        // It is set to 632 because that is the maximum amount of bytes that the Quest can transmit
        // I do not believe there will ever be a situation in which all 632 bytes will be used, since that requires the Quest to be seeing 22 AprilTags at the same time
        byte[] buffer = new byte[632];

        // This is the thing that recieves the transmission and puts it into the buffer
        DatagramPacket packet = new DatagramPacket(buffer, buffer.length);
        
        // Recieving the transmission is a blocking function. No code in this thread will run until the transmission is finished
        try {
            socket.receive(packet);
        } catch (IOException e) {
            System.out.println("sry no data uwu");
        }
        
        // commented out code; pay no attention
        //recievedFloat = ByteBuffer.wrap(packet.getData()).asFloatBuffer().array();
        //System.out.println("uhhh data:"+Arrays.toString(recievedFloat));
        //System.out.println("fanumtaxohio");

        // penisBuffer is the intermediary between the transmission data as a byte array and as other datatypes, such as floats or ints
        ByteBuffer penisBuffer = ByteBuffer.wrap(buffer).order(ByteOrder.LITTLE_ENDIAN);

        // The first 12 bytes of the transmission are 3 floats. These floats are what the Quest thinks its position is
        // The format is quite similar to a Pose2D, but in floating point form instead of double precision.
        // The first float is the x-position, the second is the z-position, and the third is the y-rotation (euler angles)
        recievedFloat[1]=-penisBuffer.getFloat()-AprilTagConstants.questPositionFromRobotCenter[1];
        recievedFloat[0]=penisBuffer.getFloat()-AprilTagConstants.questPositionFromRobotCenter[0];
        recievedFloat[2]=-penisBuffer.getFloat();

        if(takeSnapshot){
            rotSnapshot = recievedFloat[2];
            //takeSnapshot=false;
        }

        globalizeRot = extraAccuracyPoseHoldValue[2]-rotSnapshot;

        rotatedRecievedFloat=PoopyMath.rotateVector2(recievedFloat[0], recievedFloat[1], globalizeRot);

        if(takeSnapshot){
            heheHawHawPoseOffset = rotatedRecievedFloat;
            takeSnapshot=false;
        }

        globalizedRecievedPose[0] = extraAccuracyPoseHoldValue[0]+rotatedRecievedFloat[0]-heheHawHawPoseOffset[0];
        globalizedRecievedPose[1] = extraAccuracyPoseHoldValue[1]+rotatedRecievedFloat[1]-heheHawHawPoseOffset[1];
        globalizedRecievedPose[2] = globalizeRot+recievedFloat[2];

        globalizedRecievedPose[2] %= 360;
        if(globalizedRecievedPose[2]<0){
            globalizedRecievedPose[2]+=360;
        }

        //rotatedQuestPos=PoopyMath.rotateVector2(AprilTagConstants.questPositionFromRobotCenter[0], AprilTagConstants.questPositionFromRobotCenter[1], recievedFloat[2]);
        //recievedFloat[0]-=rotatedQuestPos[0];
        //recievedFloat[1]-=rotatedQuestPos[1];

        // After the Quest pose, there is an int that says how many tags were detected.
        // It is stored in tagNum for future use
        tagNum = penisBuffer.getInt();

        // commented out code; pay no attention
        //id1 = penisBuffer.getInt();
        //tagTransforms = new AprilTagTransform[tagNum];
        //tagNum = 22;

        // Each of these arrays are set to a new array the size of tagNum, since tagNum is the number of tags seen
        id = new int[tagNum];
        posX = new float[tagNum];
        posY = new float[tagNum];
        posZ = new float[tagNum];
        rotX = new float[tagNum];
        rotY = new float[tagNum];
        rotZ = new float[tagNum];
        for(int i = 0; i<tagNum; i++){
            id[i] = penisBuffer.getInt();
            posX[i] = penisBuffer.getFloat();
            posY[i] = penisBuffer.getFloat();
            posZ[i] = penisBuffer.getFloat();
            rotX[i] = penisBuffer.getFloat();
            rotY[i] = penisBuffer.getFloat();
            rotZ[i] = penisBuffer.getFloat();
        }
        //totalCalls++;

        /*
        << uncomment this code when needed >>
        if(tagNum>0&&useAprilTags){
            setPoseFromTag();
            poseOffset[0] = recievedFloat[0];
            poseOffset[1] = recievedFloat[1];
            poseOffset[2] = recievedFloat[2];
        }

        calibratedPose[0] = recievedFloat[0]-poseOffset[0]+poseFromTag[0];
        calibratedPose[1] = recievedFloat[1]-poseOffset[1]+poseFromTag[1];
        calibratedPose[2] = recievedFloat[2]-poseOffset[2]+poseFromTag[2];
        calibratedPose[2] %= 360;
        if(calibratedPose[2]<0){
            calibratedPose[2]+=360;
        }
        */

        if(tagNum>0&&useAprilTags){
            setPoseFromTag();
        }

        if(motionlessCalib&&tagNum>0){
            calibFinished=false;
            //setPoseFromTag();
            extraAccuracyPose[0]+=poseFromTag[0];
            extraAccuracyPose[1]+=poseFromTag[1];
            extraAccuracyPose[2]+=poseFromTag[2];
            calibSampleIndex++;
            if(calibSampleIndex==calibSamples){
                calibFinished=true;
                motionlessCalib=false;
                calibSampleIndex=0;
                extraAccuracyPoseHoldValue[0]=0;
                extraAccuracyPoseHoldValue[1]=0;
                extraAccuracyPoseHoldValue[2]=0;
                extraAccuracyPose[0]/=calibSamples;
                extraAccuracyPose[1]/=calibSamples;
                extraAccuracyPose[2]/=calibSamples;
                extraAccuracyPoseHoldValue[0]=extraAccuracyPose[0];
                extraAccuracyPoseHoldValue[1]=extraAccuracyPose[1];
                extraAccuracyPoseHoldValue[2]=extraAccuracyPose[2];
                extraAccuracyPose[0]=0;
                extraAccuracyPose[1]=0;
                extraAccuracyPose[2]=0;
                takeSnapshot=true;
                System.out.println("hiiiiiiiiiii :3");
            }
        }

        //yeah put code here
        if(enableAuto&&pathOrSetpoint){
            updatePath();
            pidValues[0] = MathUtil.clamp(pidTrans.calculate(globalizedRecievedPose[0],path[pathIndex].x),-pidTransClamp,pidTransClamp);
            atSetpoint[0]=pidTrans.atSetpoint();
            pidValues[1] = MathUtil.clamp(pidTrans.calculate(globalizedRecievedPose[1],path[pathIndex].y),-pidTransClamp,pidTransClamp);
            atSetpoint[1]=pidTrans.atSetpoint();
            pidValues[2] = MathUtil.clamp(pidRot.calculate(globalizedRecievedPose[2],path[pathIndex].z),-pidRotClamp,pidRotClamp);
            atSetpoint[2] = pidRot.atSetpoint();
        }else if(enableAuto){
            pidValues[0] = MathUtil.clamp(pidTrans.calculate(globalizedRecievedPose[0],path[pathIndex].x),-pidTransClamp,pidTransClamp);
            atSetpoint[0]=pidTrans.atSetpoint();
            pidValues[1] = MathUtil.clamp(pidTrans.calculate(globalizedRecievedPose[1],path[pathIndex].y),-pidTransClamp,pidTransClamp);
            atSetpoint[1]=pidTrans.atSetpoint();
            pidValues[2] = MathUtil.clamp(pidRot.calculate(globalizedRecievedPose[2],path[pathIndex].z),-pidRotClamp,pidRotClamp);
            atSetpoint[2] = pidRot.atSetpoint();
            if(atSetpoint[0]&&atSetpoint[1]&&atSetpoint[2]){
                atSetpointReturnBool=true;
            }else{
                atSetpointReturnBool=false;
            }
        }else{
            pidValues[0]=0;
            pidValues[1]=0;
            pidValues[2]=0;
        }
        //updatePath();
    }

    void updatePath(){
        if(atSetpoint[0]&&atSetpoint[1]&&atSetpoint[2]&&pathIndex<path.length-1){
            pathIndex++;
        }else if(atSetpoint[0]&&atSetpoint[1]&&atSetpoint[2]){
            atSetpointReturnBool = true;
        }else{
            atSetpointReturnBool=false;
        }
    }

    public boolean atPose(){
        return atSetpointReturnBool;
    }

    public Command EnableAuto(){
        return run(
            ()->{
                enableAuto=true;
            }
        );
    }

    public Command DisableAuto(){
        return run(
            ()->{
                enableAuto=false;
            }
        );
    }

    public Command SetSetpoint(double[] setpoint){
        return runOnce(
            ()->{
                poseSetpoint=setpoint;
                pathOrSetpoint=false;
            }
        );
    }

    public Command SetPath(Double3[] putThePathHereLOL){
        return runOnce(
            ()->{
                path=putThePathHereLOL;
                pathOrSetpoint=true;
            }
        );
    }

    public double getPidValues(int index){
        return pidValues[index];
    }

    public double[] getCalibratedPose(){
        return calibratedPose;
    }

    public Pose2d getCalibratedPoseAsPose2d(){
        return new Pose2d(calibratedPose[0],calibratedPose[1],new Rotation2d(Math.toRadians(calibratedPose[2])));
    }

    public Pose2d getUwUPose(){
        return new Pose2d(globalizedRecievedPose[0],globalizedRecievedPose[1],new Rotation2d(Math.toRadians(globalizedRecievedPose[2])));
    }

    public Pose2d getTagPose(){
        return new Pose2d(poseFromTag[0],poseFromTag[1],new Rotation2d(Math.toRadians(poseFromTag[2])));
    }

    public double[] getMotionlessCalibPose(){
        return extraAccuracyPoseHoldValue;
    }

    public Command enableMotionlessCalib(int sampleAmount){
        return runOnce(
            ()->{
                calibSamples=sampleAmount;
                motionlessCalib=true;
                System.out.println("fuck you");
            }
        );
    }

    public double[] getClosestReefPose(boolean redOrBlue){
        double minDist = 1000000;
        double currentDist = 0;
        Double3 closestPose = new Double3(0,0,0);
        if(redOrBlue){
            for (Double3 scoringPose : ScoringPoses.redReefScoringPoses){
                currentDist=PoopyMath.getDistance(new Double3(globalizedRecievedPose[0], globalizedRecievedPose[1], globalizedRecievedPose[2]),scoringPose);
                if(currentDist<minDist){
                    minDist=currentDist;
                    closestPose=scoringPose;
                }
            }
            return new double[]{closestPose.x,closestPose.y,closestPose.z};
        }else{
            return new double[]{69,69,69};
        }
    }


    /*
    public float[] calculatePosition(){
        float calculatedX = 0;
        float calculatedZ = 0;
        float calculatedYRot = 0;
        for(int i = 0; i<tagNum; i++){
            calculatedX += AprilTagConstants.GetTagPosition(id[i])[0]-posX[i];
            calculatedZ += AprilTagConstants.GetTagPosition(id[i])[1]-posZ[i];
            calculatedYRot += AprilTagConstants.GetTagPosition(id[i])[2]-rotY[i];
        }
        calculatedX/=tagNum;
        calculatedZ/=tagNum;
        calculatedYRot/=tagNum;
        return(new float[]{calculatedX,calculatedZ,calculatedYRot});
    }
    */


    //add 180 degrees
    //swap x and y
    //and add x and y instead of subtract (actually dont do this rotate 180 first again instead lmao)

    void setPoseFromTag(){
        poseFromTag[0]=0;
        poseFromTag[1]=0;
        poseFromTag[2]=0;
        for(int i = 0; i<tagNum; i++){
            double[] rotatedPose = PoopyMath.rotateVector2(posZ[i],posX[i],AprilTagConstants.GetTagPosition(id[i])[2]/*-rotY[i]*/);
            poseFromTag[0] += AprilTagConstants.GetTagPosition(id[i])[0]+rotatedPose[0];//-AprilTagConstants.cameraPositionFromRobotCenter[0];
            poseFromTag[1] += AprilTagConstants.GetTagPosition(id[i])[1]-rotatedPose[1];//-AprilTagConstants.cameraPositionFromRobotCenter[1];
            poseFromTag[2] += AprilTagConstants.GetTagPosition(id[i])[2]-rotY[i];
        }
        poseFromTag[0]/=tagNum;
        poseFromTag[1]/=tagNum;
        poseFromTag[2]/=tagNum;

        //poseFromTag[2]+=180;

        poseFromTag[2] %= 360;
        if(poseFromTag[2]<0){
            poseFromTag[2]+=360;
        }

        double[] rotatedCamPos = PoopyMath.rotateVector2(AprilTagConstants.cameraPositionFromRobotCenter[0], AprilTagConstants.cameraPositionFromRobotCenter[1], poseFromTag[2]);
        poseFromTag[0]-=rotatedCamPos[0];
        poseFromTag[1]-=rotatedCamPos[1];
    }


    // Call this to print out the data recieved from the quest. Purely for debugging purposes
    public Command ErmWhatTheSigma(){
        return runOnce(
            ()->{
                System.out.println("uhhh data:"+Arrays.toString(recievedFloat));
                //System.out.println(id1);
                for(int i = 0; i < tagNum; i++){
                    System.out.println("ID = "+id[i]+", position = ("+posX[i]+","+posY[i]+","+posZ[i]+"), rotation = ("+rotX[i]+","+rotY[i]+","+rotZ[i]+")");
                    //System.out.println("Total calls: " + totalCalls);
                }
            }
        );
    }

    
    @Override
    public void periodic(){
        //m_field.setRobotPose(getUwUPose());
        poseSetpoint[0]=path[0].x;
        poseSetpoint[1]=path[0].y;
        poseSetpoint[2]=path[0].z;
        m_field.setRobotPose(getUwUPose());
        //SmartDashboard.putNumber("pathIndex", pathIndex);
        //SmartDashboard.putNumberArray("calibPose",poseFromTag);
        //SmartDashboard.putBoolean("calib", motionlessCalib);
        //SmartDashboard.putNumber("ids", tagNum);
        //SmartDashboard.putNumber("snapshot", rotSnapshot);
        //SmartDashboard.putNumberArray("Pid Values", pidValues);
        //SmartDashboard.putNumberArray("Setpoint", poseSetpoint);
        if(SmartDashboard.getBoolean("test",false)){
            System.out.println("peenor");
            calibSamples=50;
            motionlessCalib=true;
            testbool=false;
            SmartDashboard.putBoolean("test", testbool);
        }
    }
}
