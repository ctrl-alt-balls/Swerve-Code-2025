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
    PIDController pidController = new PIDController(0.1, 0.01, 0.001);

    double elevatorZero = 0;
    double l1 = 0;
    double l2 = 0;
    double l3 = 0;
    double l4 = 0;

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
    double currentPIDVal;
    
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

    @Override
	public void periodic() {
        encVal = elevEnc.getDistance();

        if(isZeroing&&!limSwitchBottom.get()&&!isBottomed){
            currentPIDVal = pidController.calculate(elevEnc.getRate(),zeroInitSpeed);
        }else if(isZeroing&&limSwitchBottom.get()){
            currentPIDVal = pidController.calculate(elevEnc.getRate(),zeroExitSpeed);
            isBottomed = true;
        }else if(isZeroing&&!limSwitchBottom.get()&&isBottomed){
            currentPIDVal = 0;
            isBottomed = false;
            isZeroing = false;
            elevatorZero=encVal;
        }else{
            currentPIDVal = pidController.calculate(encVal-elevatorZero,setpoint);
        }

        elevatorNeoLeft.set(currentPIDVal);
        elevatorNeoRight.set(-currentPIDVal);

		SmartDashboard.putNumber("Encoder", elevEnc.getDistance()-elevatorZero);
        SmartDashboard.putNumber("PID", currentPIDVal);
	}
}
