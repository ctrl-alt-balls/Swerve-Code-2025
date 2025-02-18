package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;


public class ElevatorSubsystem extends SubsystemBase{
    SparkMax elevatorNeoRight = new SparkMax(17, MotorType.kBrushless);
    SparkMax elevatorNeoLeft = new SparkMax(16, MotorType.kBrushless);
    PIDController pidController = new PIDController(0.8, 0.0, 0.0);

    PIDController ratePIDController = new PIDController(0.1, 0.0, 0);

    double elevatorZero = 0;

    // using quadrature encoder
    // if lacking in precision, then change EncodingType to k4X
    Encoder elevEnc = new Encoder(0, 1, false, EncodingType.k2X);

    DigitalInput limSwitchBottom = new DigitalInput(2);
    DigitalInput limSwichTop = new DigitalInput(3);
    boolean isZeroing = false;
    double zeroInitSpeed;
    double zeroExitSpeed;
    boolean isBottomed = false;

    double setpoint;
    double encVal;
    double encRate;
    double currentPIDVal;

    double topSwitchPIDResponse = -2;
    double bottomSwitchPIDResponse = 2;

    boolean isManualRun = false;
    
    //private final ShuffleboardTab m_tab = Shuffleboard.getTab("Elevator");

    public Command SetPositionCommand(double setpointInput){
        return runOnce(
            ()->{
                setpoint=setpointInput;
            }
        );
    }

    public Command Zero(double initSpeed,double exitSpeed){
        return runOnce(
            ()->{
                isZeroing=true;
                zeroInitSpeed = initSpeed;
                zeroExitSpeed = exitSpeed;
            }
        );
    }

    public Command ManualRun(double speed){
        return run(
        () -> {
            isManualRun = true;
            elevatorNeoLeft.set(currentPIDVal);
            elevatorNeoRight.set(-currentPIDVal);
        });
    }

    public Command DisableManualRun(){
        return run(
            ()->{
                isManualRun=false;
            }
        );
    }

    @Override
	public void periodic() {
        encVal = -elevEnc.getDistance()/2000;
        encRate = -elevEnc.getRate()/2000;

        if(isZeroing&&!limSwitchBottom.get()&&!isBottomed){
            currentPIDVal = ratePIDController.calculate(encRate,zeroInitSpeed);
        }else if(isZeroing&&limSwitchBottom.get()){
            currentPIDVal = ratePIDController.calculate(encRate,zeroExitSpeed);
            isBottomed = true;
        }else if(isZeroing&&!limSwitchBottom.get()&&isBottomed){
            currentPIDVal = 0;
            isBottomed = false;
            isZeroing = false;
            elevatorZero=encVal;
        }else if(limSwichTop.get()){
            currentPIDVal=ratePIDController.calculate(encRate,topSwitchPIDResponse);
            isManualRun = false;
        }else if(limSwitchBottom.get()){
            currentPIDVal=ratePIDController.calculate(encRate,bottomSwitchPIDResponse);
            isManualRun = false;
        }else{
            currentPIDVal = pidController.calculate(encVal-elevatorZero,setpoint);
        }

        if(currentPIDVal>1){
            currentPIDVal=1;
        }else if(currentPIDVal<-1){
            currentPIDVal=-1;
        }

        if(!isManualRun){
            elevatorNeoLeft.set(currentPIDVal);
            elevatorNeoRight.set(-currentPIDVal);
        }

        //isManualRun = false;

		SmartDashboard.putNumber("Encoder", encVal-elevatorZero);
        SmartDashboard.putNumber("EncoderRate", encRate);
        SmartDashboard.putNumber("PID", currentPIDVal);
        SmartDashboard.putNumber("setpoint", setpoint);
        SmartDashboard.putBoolean("topSwitch", limSwichTop.get());
        SmartDashboard.putBoolean("bottomSwitch", limSwitchBottom.get());
	}
}
