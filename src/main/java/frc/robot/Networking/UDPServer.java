package frc.robot.Networking;

import java.io.IOException;
import java.net.DatagramPacket;
import java.net.DatagramSocket;
import java.net.SocketException;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.util.Arrays;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Command;

public class UDPServer extends SubsystemBase{

    public class AprilTagTransform{
        int id = 0;
        float posX = 0;
        float posY = 0;
        float posZ = 0;
        float rotX = 0;
        float rotY = 0;
        float rotZ = 0;
    }

    DatagramSocket socket;
    float[] recievedFloat = new float[3];
    int tagNum = 0;
    AprilTagTransform[] tagTransforms;

    int[] id;
    float[] posX;
    float[] posY;
    float[] posZ;
    float[] rotX;
    float[] rotY;
    float[] rotZ;

    int id1 = 0;

    public UDPServer(int port){
        try{
            socket = new DatagramSocket(port);
            System.out.println("fanum tax started server on port " + socket.getLocalPort());
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
        byte[] buffer = new byte[632];
        DatagramPacket packet = new DatagramPacket(buffer, buffer.length);
        
        try {
            socket.receive(packet);
        } catch (IOException e) {
            System.out.println("sry no data uwu");
        }
        
        //recievedFloat = ByteBuffer.wrap(packet.getData()).asFloatBuffer().array();
        //System.out.println("uhhh data:"+Arrays.toString(recievedFloat));
        //System.out.println("fanumtaxohio");

        ByteBuffer penisBuffer = ByteBuffer.wrap(buffer).order(ByteOrder.LITTLE_ENDIAN);
        for(int i = 0; i<3;i++){
            recievedFloat[i]=penisBuffer.getFloat();
        }
        tagNum = penisBuffer.getInt();
        //id1 = penisBuffer.getInt();
        //tagTransforms = new AprilTagTransform[tagNum];
        //tagNum = 22;
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
    }

    public Command ErmWhatTheSigma(){
        return runOnce(
            ()->{
                System.out.println("uhhh data:"+Arrays.toString(recievedFloat));
                //System.out.println(id1);
                for(int i = 0; i < tagNum; i++){
                    System.out.println("ID = "+id[i]+", position = ("+posX[i]+","+posY[i]+","+posZ[i]+"), rotation = ("+rotX[i]+","+rotY[i]+","+rotZ[i]+")");
                }
            }
        );
    }
}
