// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.constants.Constants;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


import frc.robot.constants.Constants;
import frc.robot.commands.AimToHub;
import frc.robot.commands.ConveyerTurn;
import frc.robot.commands.FeedinCommand;
import frc.robot.commands.IndexerCommand;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.IntakeJoystickCommand;
import frc.robot.commands.LauncherTurn;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LauncherSubsystem;
import frc.robot.subsystems.PhotonVisionSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class RobotContainer {
    private double MaxSpeed = 0.5 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.RobotCentric roboDrivCentric = new SwerveRequest.RobotCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Add a 10% deadband
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
    
    /* Subsystems */
    private final LauncherSubsystem l_launch = new LauncherSubsystem();
    private final IndexerSubsystem i_index = new IndexerSubsystem();
    private final IntakeSubsystem i_intake = new IntakeSubsystem();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController driver = new CommandXboxController(0);
    private final CommandXboxController operator = new CommandXboxController(1);
    private final JoystickButton hubAligner = new JoystickButton(driver.getHID(), XboxController.Button.kA.value);
    private final PhotonVisionSubsystem pv_PhotonVisionSubsystem = new PhotonVisionSubsystem();
    

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    public RobotContainer() {

        configureBindings();
    }

    private final SendableChooser<Command> autoChooser = AutoBuilder.buildAutoChooser("Auto Paths");;


    private void configureBindings() {

            SmartDashboard.putNumber("desired Max Speed", MaxSpeed);
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

        driver.b().whileTrue(drivetrain.applyRequest(() ->
            point.withModuleDirection(new Rotation2d(-driver.getLeftY(), -driver.getLeftX()))
        ));
        hubAligner.whileTrue(new AimToHub(drivetrain,
                                            pv_PhotonVisionSubsystem,
                                            driver.getHID()
                                            ));
        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        driver.back().and(driver.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        driver.back().and(driver.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        driver.start().and(driver.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        driver.start().and(driver.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // Reset the field-centric heading on left bumper press.
        driver.y().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

    /* DRIVER */
        /* robot centric */
       driver.leftBumper().toggleOnTrue(drivetrain.applyRequest(() ->
                roboDrivCentric.withVelocityX(-driver.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-driver.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-driver.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
                ));

        driver.b().whileTrue(drivetrain.applyRequest(() -> brake));

        driver.x().whileTrue(drivetrain.applyRequest(() ->
            point.withModuleDirection(new Rotation2d(-driver.getLeftY(), -driver.getLeftX()))
        ));

        // Reset the field-centric heading on left bumper press.
        driver.y().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));



     /* OPERATOR */
        /* launcher */
            operator.a()
            .onTrue(new LauncherTurn(l_launch, true
            ))
            .onFalse(new LauncherTurn(l_launch, true
            ));

            operator.b()
             .onTrue(new LauncherTurn(l_launch, false
            ));

        /* index */
            operator.x()
                .onTrue(new FeedinCommand(i_index, l_launch, Constants.LauncherConstants.ConveyerMotorSpeed, -Constants.IndexerConstants.kIndexMotorSpeed)
               )
               .onFalse(new FeedinCommand(i_index, l_launch, 0.0, 0.0)
               );

            operator.y()
               .onTrue(new FeedinCommand(i_index, l_launch, -Constants.LauncherConstants.ConveyerMotorSpeed, Constants.IndexerConstants.kIndexMotorSpeed)
               )
               .onFalse(new FeedinCommand(i_index, l_launch, 0.0, 0.0)
               );
        
        /* intake  */ 
            operator.leftBumper()
             .onTrue(new IntakeCommand(i_intake, -Constants.IntakeConstants.kUpperIntakeMotorSpeed, Constants.IntakeConstants.kLowerIntakeMotorSpeed) 
                )
             .onFalse(new IntakeCommand(i_intake, 0.0, 0.0)
             );
           
             operator.rightBumper()
             .onTrue(new IntakeCommand(i_intake, Constants.IntakeConstants.kUpperIntakeMotorSpeed, -Constants.IntakeConstants.kLowerIntakeMotorSpeed)
                )
             .onFalse(new IntakeCommand(i_intake, 0.0, 0.0)
             );


            // i_intake.setDefaultCommand(
            //         Commands.run(
            //             () -> i_intake.setPivotSpeed(-operator.getRawAxis(XboxController.Axis.kLeftY.value)*Constants.IntakeConstants.kPivotMotorSpeed),
            //             i_intake
            //         ));
            i_intake.setDefaultCommand(
                new IntakeJoystickCommand(
                    i_intake, 
                    () -> operator.getRawAxis(XboxController.Axis.kLeftY.value)
                ));

            operator.pov(0)
            .whileTrue(new InstantCommand(() ->
                i_intake.turnDegrees(Constants.IntakeConstants.kPivotMotorDegree), i_intake));
        

            operator.pov(180)
            .whileTrue(new InstantCommand(() ->
                i_intake.turnDegrees(-Constants.IntakeConstants.kPivotMotorDegree), i_intake
                ));
            

        drivetrain.registerTelemetry(logger::telemeterize);

         SmartDashboard.putData("Auto Mode", autoChooser);
    }

    public Command getAutonomousCommand() {
        
         return autoChooser.getSelected();
    }
    
}
