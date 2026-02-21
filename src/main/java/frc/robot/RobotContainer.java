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
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.constants.Constants;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

import frc.robot.constants.Constants;
import frc.robot.commands.ConveyerTurn;
import frc.robot.commands.LauncherTurn;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LauncherSubsystem;
import frc.robot.subsystems.IndexerSubsystem;

    
public class RobotContainer {
    private double MaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
    
    /* Subsystems */
    private final LauncherSubsystem l_Launch = new LauncherSubsystem();
    private final IndexerSubsystem i_index = new IndexerSubsystem();
    private final IntakeSubsystem i_intake = new IntakeSubsystem();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController driver = new CommandXboxController(0);
    private final CommandXboxController operator = new CommandXboxController(1);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    public RobotContainer() {
        configureBindings();
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-driver.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-driver.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-driver.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );

        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );

        driver.a().whileTrue(drivetrain.applyRequest(() -> brake));
        driver.b().whileTrue(drivetrain.applyRequest(() ->
            point.withModuleDirection(new Rotation2d(-driver.getLeftY(), -driver.getLeftX()))
        ));

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        driver.back().and(driver.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        driver.back().and(driver.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        driver.start().and(driver.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        driver.start().and(driver.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // Reset the field-centric heading on left bumper press.
        driver.leftBumper().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

     /* OPERATOR */
     
        /* launcher */
            operator.a()
            .onTrue(new LauncherTurn(l_Launch, true))
            .onFalse(new LauncherTurn(l_Launch, true));

            operator.pov(0)
                .onTrue(new ConveyerTurn (l_Launch,Constants.LauncherConstants.ConveyerMotorSpeedRpm))
                .onFalse(new ConveyerTurn (l_Launch,0.0));

        /* index */
            operator.rightBumper()
                .onTrue(new InstantCommand(() ->
                i_index.setIndexerSpeed(Constants.IndexerConstants.kIndexMotorSpeed)
               ))
               .onFalse(new InstantCommand(() ->
                i_index.setIndexerSpeed(0)
               ));

            operator.rightTrigger()
              .onTrue(new InstantCommand(() ->
                i_index.setIndexerSpeed(-Constants.IndexerConstants.kIndexMotorSpeed)
               ))
               .onFalse(new InstantCommand(() ->
                i_index.setIndexerSpeed(0)
               ));
        
        /* intake  */
            operator.leftBumper()
             .onTrue(new InstantCommand(() ->
                i_intake.setIntakeSpeed(Constants.IntakeConstants.kUpperIntakeMotorSpeed, Constants.IntakeConstants.kLowerIntakeMotorSpeed) 
                ))
             .onFalse(new InstantCommand(() ->
                i_intake.setIntakeSpeed(0.0, 0.0)
             ));
           
             operator.leftTrigger()
             .onTrue(new InstantCommand(() ->
                i_intake.setIntakeSpeed(-Constants.IntakeConstants.kUpperIntakeMotorSpeed,-Constants.IntakeConstants.kLowerIntakeMotorSpeed) 
                ))
             .onFalse(new InstantCommand(() ->
                i_intake.setIntakeSpeed(0.0, 0.0)
             ));

             
            operator.x()
            .onTrue(new InstantCommand(() ->
                i_intake.turnDegrees(Constants.IntakeConstants.kRotatingMotorDegree)
                ));
            // .onFalse(new InstantCommand(() ->
            //     i_intake.turnDegrees(0.0)
            //     ));
                
            operator.y()
            .onTrue(new InstantCommand(() ->
                i_intake.turnDegrees(-Constants.IntakeConstants.kRotatingMotorDegree)
                ));
            // .onFalse(new InstantCommand(() ->
            //     i_intake.turnDegrees(0.0)
            //     ));
                       

        drivetrain.registerTelemetry(logger::telemeterize);
    }

    public Command getAutonomousCommand() {
        // Simple drive forward auton
        final var idle = new SwerveRequest.Idle();
        return Commands.sequence(
            // Reset our field centric heading to match the robot
            // facing away from our alliance station wall (0 deg).
            drivetrain.runOnce(() -> drivetrain.seedFieldCentric(Rotation2d.kZero)),
            // Then slowly drive forward (away from us) for 5 seconds.
            drivetrain.applyRequest(() ->
                drive.withVelocityX(0.5)
                    .withVelocityY(0)
                    .withRotationalRate(0)
            )
            .withTimeout(5.0),
            // Finally idle for the rest of auton
            drivetrain.applyRequest(() -> idle)
        );
    }
}
