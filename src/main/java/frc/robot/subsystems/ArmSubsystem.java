package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmSubsystem extends SubsystemBase {
    final TalonFX m_grippinator500 = new TalonFX(16,"rio");

    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        setDefaultCommand(MoveMotor(0.0));
    }

    public Command MoveMotor(double speed){
        return run(
        () -> {
            m_grippinator500.set(speed);
        });
    }


}
