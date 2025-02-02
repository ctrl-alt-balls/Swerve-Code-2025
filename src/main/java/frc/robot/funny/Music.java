package frc.robot.funny;

import com.ctre.phoenix6.Orchestra;
import com.ctre.phoenix6.hardware.ParentDevice;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Music extends SubsystemBase{
    Orchestra orchestra = new Orchestra();

    public Music(ParentDevice... devices){
        for(ParentDevice theDevice : devices){
            orchestra.addInstrument(theDevice);
        }
    }

    public Command PlayMusic(String trackName){
        return runOnce(
            ()->{
                orchestra.loadMusic(trackName);
                orchestra.play();
            }
        );
    }
}
