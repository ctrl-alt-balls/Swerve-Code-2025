// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.SubsystemConstants.ElevatorConstants;
import frc.robot.Networking.UDPServer;
import frc.robot.funny.Music;

public class RobotContainer {
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController driverController = new CommandXboxController(0);
    private final CommandXboxController scorerController = new CommandXboxController(1);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    public final ArmSubsystem arm = new ArmSubsystem();

    // Port 5805 is the port I chose to communicate with the Quest over. Please do not change this, since it is hardcoded into the Quest app to communicate over this port
    public final UDPServer questServer = new UDPServer(5805);
    
    ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem();

    /*
    public final Music music = new Music(
        new TalonFX(34),
        new TalonFX(30),
        new CANcoder(26),
        new TalonFX(33),
        new TalonFX(29),
        new CANcoder(25),
        new TalonFX(32),
        new TalonFX(28),
        new CANcoder(24),
        new TalonFX(31),
        new TalonFX(27),
        new CANcoder(23)
    );
    */

    public RobotContainer() {
        configureBindings();
        //music.PlayMusic("caveStory.chrp");
    }

    private void configureBindings() {

        scorerController.a().onTrue(elevatorSubsystem.SetPositionCommand(2));
        scorerController.b().onTrue(elevatorSubsystem.SetPositionCommand(5));

        // Some testing stuff for elevator
        scorerController.x().onTrue(elevatorSubsystem.ManualRun(0.5));
        scorerController.y().onTrue(elevatorSubsystem.ManualRun(-0.5));
        scorerController.x().and(scorerController.y()).whileFalse(elevatorSubsystem.DisableManualRun());


        scorerController.povLeft().onTrue(elevatorSubsystem.Zero(-4, 0.5));

        // Not necessary at all, I still need to test this though. It will play a MIDI of the among us drip music
        arm.initDefaultCommand();
        //scorerController.x().whileTrue(arm.RunClimber(0.6));
        //scorerController.y().whileTrue(arm.RunClimber(-0.6));
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-driverController.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-driverController.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-driverController.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );

        
        driverController.a().whileTrue(drivetrain.applyRequest(() -> brake));
        driverController.b().whileTrue(drivetrain.applyRequest(() ->
            point.withModuleDirection(new Rotation2d(-driverController.getLeftY(), -driverController.getLeftX()))
        ));

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        driverController.back().and(driverController.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        driverController.back().and(driverController.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        driverController.start().and(driverController.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        driverController.start().and(driverController.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // reset the field-centric heading on left bumper press
        driverController.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        // Press this to print out data recieved from the Quest
        driverController.povDown().onTrue(questServer.ErmWhatTheSigma());

        drivetrain.registerTelemetry(logger::telemeterize);
    }

    public Command getAutonomousCommand() {
        return Commands.print("No autonomous command configured");
    }
}
