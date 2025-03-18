package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Networking.UDPServer;
import frc.robot.subsystems.SubsystemConstants.SubsystemConstants;

public class ArmSubsystem extends SubsystemBase {

    public static enum Level{
        L1,
        L2,
        L3,
        L4,
        CoralStation,
        Resting
    }

    final TalonFX m_grippinator500 = new TalonFX(19,"rio");
    SparkMax armRotNeo = new SparkMax(18, MotorType.kBrushless);
    DutyCycleEncoder armEnc = new DutyCycleEncoder(4);
    SparkMax climberMotor = new SparkMax(20, MotorType.kBrushed);

    //DigitalInput coralSensor = new DigitalInput(5);

    PIDController pidController = new PIDController(10, 0, 0);
    public double rotSetpoint = 0.3;
    public double armEncVal;
    double currentPIDVal = 0;

    public boolean enableArm = true;

    double pidTolerance = 0.08;
    boolean ejectCoral = false;
    boolean intakeCoral = false;
    double ejectSpeed = 0.5;
    double intakeSpeed = 0.2;

    public double armCollisionPoint = 0.4;
    public double elevatorLowCollisionPoint;
    public double elevatorHighCollisionPoint;

    boolean manualGrippinatorMovement = false;
    boolean automaticGrippinatorMovement = false;

    ElevatorSubsystem elevator;
    UDPServer questServer;

    double intakeCurrentLimit = 10;
    double grippinatorCurrent;

    public ArmSubsystem(ElevatorSubsystem elevatorSubsystem,UDPServer udpServer){
        this.elevator=elevatorSubsystem;
        pidController.setTolerance(pidTolerance);
        this.questServer=udpServer;
    }

    public Command DefaultArmCommand(){
        return run(
            ()->{
                RunClimber(0.0);
                manualGrippinatorMovement=false;
            }
        );
    }

    public Command EnableManualGrippinator(){
        return run(
            ()->{
                manualGrippinatorMovement=true;
            }
        );
    }

    public Command SetArmRotationCommand(double setpointInput){
        return runOnce(
            ()->{
                rotSetpoint=setpointInput;
            }
        );
    }

    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        setDefaultCommand(DefaultArmCommand());
    }

    public Command RunClimber(double speed){
        return run(
        () -> {
            climberMotor.set(speed);
        });
    }


    public Command Score(Level level){
        return runOnce(
            ()->{
                switch(level){
                    case L1:
                        rotSetpoint = SubsystemConstants.ArmConstants.L1;
                        elevator.setpoint = SubsystemConstants.ElevatorConstants.L1;
                        ejectCoral = true;
                        intakeCoral = false;
                        break;
                    case L2:
                        rotSetpoint = SubsystemConstants.ArmConstants.L2;
                        elevator.setpoint = SubsystemConstants.ElevatorConstants.L2;
                        ejectCoral = true;
                        intakeCoral = false;
                        break;
                    case L3:
                        rotSetpoint = SubsystemConstants.ArmConstants.L3;
                        elevator.setpoint = SubsystemConstants.ElevatorConstants.L3;
                        ejectCoral = true;
                        intakeCoral = false;
                        break;
                    case L4:
                        rotSetpoint = SubsystemConstants.ArmConstants.L4;
                        elevator.setpoint = SubsystemConstants.ElevatorConstants.L4;
                        ejectCoral = true;
                        intakeCoral = false;
                        break;
                    case CoralStation:
                        rotSetpoint = SubsystemConstants.ArmConstants.CoralStation;
                        elevator.setpoint = SubsystemConstants.ElevatorConstants.CoralStation;
                        ejectCoral = false;
                        intakeCoral = true;
                        break;
                    case Resting:
                        rotSetpoint=SubsystemConstants.ArmConstants.Resting;
                        elevator.setpoint=SubsystemConstants.ElevatorConstants.Resting;
                        ejectCoral=false;
                        intakeCoral=false;
                }
            }
        );
    }


    @Override
    public void periodic(){
        grippinatorCurrent = m_grippinator500.getSupplyCurrent().getValueAsDouble();
        armEncVal = armEnc.get();

        // disable elevator if arm will collide with it
        if(armEncVal<=armCollisionPoint&&elevator.getEncVal()>=elevatorLowCollisionPoint||elevator.getEncVal()<=elevatorHighCollisionPoint){
            elevator.enableElevator=false;
        }else if (pidController.atSetpoint()/*&&questServer.atPose()*/){
            elevator.enableElevator=true;
        }
        
        currentPIDVal = pidController.calculate(armEncVal, rotSetpoint);

        if(currentPIDVal>1){
            currentPIDVal=1;
        }else if(currentPIDVal<-1){
            currentPIDVal=-1;
        }

        if(enableArm){
            armRotNeo.set(-currentPIDVal);
        }

        if(pidController.atSetpoint()&&elevator.pidController.atSetpoint()&&(manualGrippinatorMovement||automaticGrippinatorMovement)){
            if(ejectCoral){
                m_grippinator500.set(ejectSpeed);
            }else if(intakeCoral&&grippinatorCurrent<intakeCurrentLimit){
                m_grippinator500.set(intakeSpeed);
            }else{
                m_grippinator500.set(0);
            }
        }else{
            m_grippinator500.set(0);
        }

        SmartDashboard.putNumber("DutyCycleEncoder", armEncVal);
        SmartDashboard.putNumber("PID", currentPIDVal);
        SmartDashboard.putNumber("Grippinator Current", grippinatorCurrent);
    }


}
