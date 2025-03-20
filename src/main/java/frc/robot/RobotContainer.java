// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
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
import frc.robot.subsystems.ArmSubsystem.Level;
import frc.robot.Networking.UDPServer;

public class RobotContainer {

    private double drivetrainSpeedMultiplier = 3;
    private double drivetrainRotationMultiplier = 1;

    ArmSubsystem.Level levelSelect;

    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(0.05*MaxSpeed).withRotationalDeadband(0.05*MaxSpeed) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController driverController = new CommandXboxController(0);
    private final CommandXboxController scorerController = new CommandXboxController(1);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    // Port 5805 is the port I chose to communicate with the Quest over. Please do not change this, since it is hardcoded into the Quest app to communicate over this port
    public final UDPServer questServer = new UDPServer(5805);
    
    public final ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem();

    public final ArmSubsystem arm = new ArmSubsystem(elevatorSubsystem,questServer);

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
        arm.initDefaultCommand();
        elevatorSubsystem.initDefaultCommand();


        

        // Elevator manual run
        scorerController.rightBumper().whileTrue(elevatorSubsystem.ManualRun(scorerController.getLeftY()));
        //scorerController.x().onTrue(elevatorSubsystem.ManualRun(0.5));
        //scorerController.y().onTrue(elevatorSubsystem.ManualRun(-0.5));

        scorerController.povLeft().onTrue(elevatorSubsystem.Zero(-4, 0.5));

        //scorerController.leftBumper().whileTrue(arm.RunClimber(0.6));
        //scorerController.rightBumper().whileTrue(arm.RunClimber(-0.6));

        // Scoring select
        scorerController.a().onTrue(Commands.runOnce(()->{levelSelect = ArmSubsystem.Level.L1;}));
        scorerController.b().onTrue(Commands.runOnce(()->{levelSelect = ArmSubsystem.Level.L2;}));
        scorerController.x().onTrue(Commands.runOnce(()->{levelSelect = ArmSubsystem.Level.L3;}));
        scorerController.y().onTrue(Commands.runOnce(()->{levelSelect = ArmSubsystem.Level.L4;}));

        // Intaking and scoring
        driverController.leftTrigger().and(driverController.rightTrigger()).whileFalse(arm.Score(ArmSubsystem.Level.Resting));
        driverController.leftTrigger().whileTrue(arm.Score(levelSelect));
        driverController.rightTrigger().whileTrue(arm.Score(ArmSubsystem.Level.CoralStation));


        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-driverController.getLeftY()*drivetrainSpeedMultiplier) // Drive forward with negative Y (forward)
                    .withVelocityY(-driverController.getLeftX()*drivetrainSpeedMultiplier) // Drive left with negative X (left)
                    .withRotationalRate(-driverController.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );

        
        driverController.a().whileTrue(drivetrain.applyRequest(() -> brake));
        driverController.b().whileTrue(drivetrain.applyRequest(() ->
            point.withModuleDirection(new Rotation2d(-driverController.getLeftY(), -driverController.getLeftX()))
        ));

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        //driverController.back().and(driverController.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        //driverController.back().and(driverController.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        //driverController.start().and(driverController.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        //driverController.start().and(driverController.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // reset the field-centric heading on left bumper press
        driverController.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        // Press this to print out data recieved from the Quest
        //driverController.povDown().onTrue(questServer.ErmWhatTheSigma());

        drivetrain.registerTelemetry(logger::telemeterize);
    }

    public Command getAutonomousCommand() {
        return Commands.print("No autonomous command configured");
    }
}
