package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmSubsystem extends SubsystemBase {
    final TalonFX m_grippinator500 = new TalonFX(19,"rio");
    SparkMax armRotNeo = new SparkMax(18, MotorType.kBrushless);
    DutyCycleEncoder armEnc = new DutyCycleEncoder(4);
    SparkMax climberMotor = new SparkMax(20, MotorType.kBrushed);

    PIDController pidController = new PIDController(0.1, 0, 0);
    double rotSetpoint = 0;
    double armEncVal;


    public Command SetArmRotationCommand(double setpointInput){
        return runOnce(
            ()->{
                rotSetpoint=setpointInput;
            }
        );
    }

    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        setDefaultCommand(RunClimber(0.0));
    }

    public Command RunClimber(double speed){
        return run(
        () -> {
            climberMotor.set(speed);
        });
    }


    @Override
    public void periodic(){
        armEncVal = armEnc.get();
        SmartDashboard.putNumber("DutyCycleEncoder", armEncVal);
    }


}
