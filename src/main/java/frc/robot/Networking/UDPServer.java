package frc.robot.Networking;

import java.io.IOException;
import java.net.DatagramPacket;
import java.net.DatagramSocket;
import java.net.SocketException;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.util.Arrays;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AprilTagConstants;
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

    DatagramSocket socket;
    float[] recievedFloat = new float[3];
    int tagNum = 0;
    //AprilTagTransform[] tagTransforms;
    float[] poseFromTag = new float[3];
    float[] poseOffset = new float[3];

    int[] id;
    float[] posX;
    float[] posY;
    float[] posZ;
    float[] rotX;
    float[] rotY;
    float[] rotZ;

    int id1 = 0;

    int totalCalls = 0;

    // for turning on the april tag position calibration (do not modify)
    boolean tagPoseCal = false;

    public UDPServer(int port){
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
        for(int i = 0; i<3;i++){
            recievedFloat[i]=penisBuffer.getFloat();
        }

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
        
        setPoseOffset();
        totalCalls++;

        if(tagPoseCal){
            tagPoseCal = false;
            
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

    void setPoseOffset(){
        for(int i = 0; i<tagNum; i++){
            poseFromTag[0] += AprilTagConstants.GetTagPosition(id[i])[0]-posX[i];
            poseFromTag[1] += AprilTagConstants.GetTagPosition(id[i])[1]-posZ[i];
            poseFromTag[2] += AprilTagConstants.GetTagPosition(id[i])[2]-rotY[i];
        }
        poseFromTag[0]/=tagNum;
        poseFromTag[1]/=tagNum;
        poseFromTag[2]/=tagNum;
    }


    public void calculateRobotPoseFromTag(){
        tagPoseCal = true;
    }


    // Call this to print out the data recieved from the quest. Purely for debugging purposes
    public Command ErmWhatTheSigma(){
        return runOnce(
            ()->{
                System.out.println("uhhh data:"+Arrays.toString(recievedFloat));
                //System.out.println(id1);
                for(int i = 0; i < tagNum; i++){
                    System.out.println("ID = "+id[i]+", position = ("+posX[i]+","+posY[i]+","+posZ[i]+"), rotation = ("+rotX[i]+","+rotY[i]+","+rotZ[i]+")");
                    System.out.println("Total calls: " + totalCalls);
                }
            }
        );
    }
}
