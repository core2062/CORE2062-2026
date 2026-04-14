// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


import frc.robot.constants.Constants;
import frc.robot.commands.AimToHub;
import frc.robot.commands.ConveyerTurn;
import frc.robot.commands.FeedinCommand;
import frc.robot.commands.IndexerCommand;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.IntakeHold;
import frc.robot.commands.IntakeJoystickCommand;
import frc.robot.commands.IntakeRotate;
import frc.robot.commands.LauncherTurn;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LauncherSubsystem;
import frc.robot.subsystems.PhotonVisionSubsystem;


public class RobotContainer {
    private double MaxSpeed;
    private double MaxAngularRate;

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
    private final IntakeSubsystem i_intake = new IntakeSubsystem();
    private final IndexerSubsystem i_index = new IndexerSubsystem(i_intake);

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController driver = new CommandXboxController(0);
    private final CommandXboxController operator = new CommandXboxController(1);
    private final JoystickButton hubAligner = new JoystickButton(driver.getHID(), XboxController.Button.kA.value);
    private final PhotonVisionSubsystem pv_PhotonVisionSubsystem = new PhotonVisionSubsystem();
    

    public CommandSwerveDrivetrain drivetrain;
    private final SendableChooser<Command> autoChooser;
    
    public RobotContainer() {
        
        drivetrain = TunerConstants.createDrivetrain();
        defineAutoCommands();
        drivetrain.configureAutoBuilder();
        autoChooser = AutoBuilder.buildAutoChooser("Auto Paths");
        
        configureBindings();
    }



    private void configureBindings() {
            setMaxSpeed(false);
            changeLauncherSpeed(0);
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

        // driver.b().whileTrue(drivetrain.applyRequest(() ->
        //     point.withModuleDirection(new Rotation2d(-driver.getLeftY(), -driver.getLeftX()))
        // ));
        
        hubAligner.whileTrue(new AimToHub(drivetrain,
                                            pv_PhotonVisionSubsystem
                                            ));
        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        driver.back().and(driver.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        driver.back().and(driver.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        driver.start().and(driver.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        driver.start().and(driver.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

    /* DRIVER */
        /* robot centric */
       driver.leftBumper().whileTrue(drivetrain.applyRequest(() ->
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
        driver.rightBumper()
            .onTrue(new InstantCommand(()->setMaxSpeed(true)))
            .onFalse(new InstantCommand(()->setMaxSpeed(false)));



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
                .onTrue(new FeedinCommand(i_index, 
                    () -> SmartDashboard.getNumber(Constants.IndexerConstants.converyMotorString, Constants.IndexerConstants.ConveyerMotorSpeed),
                    () -> SmartDashboard.getNumber(Constants.IndexerConstants.indexerSpeedString, Constants.IndexerConstants.kIndexMotorSpeed),
                    () -> SmartDashboard.getNumber(Constants.IndexerConstants.agitaterSpeedString, Constants.IndexerConstants.kAgitateMotorSpeed)
                ))
                .onFalse(new FeedinCommand(i_index, 
                    () -> 0.0,  // returns a DoubleSupplier
                    () -> 0.0,   // returns a DoubleSupplier
                    () -> 0.0    //returns a DoubleSupplier
                ));

            operator.y()
               .onTrue(new FeedinCommand(i_index, 
                    () -> -SmartDashboard.getNumber(Constants.IndexerConstants.converyMotorString, Constants.IndexerConstants.ConveyerMotorSpeed),
                    () -> -SmartDashboard.getNumber(Constants.IndexerConstants.indexerSpeedString, Constants.IndexerConstants.kIndexMotorSpeed),
                    () -> -SmartDashboard.getNumber(Constants.IndexerConstants.agitaterSpeedString, Constants.IndexerConstants.kAgitateMotorSpeed)
                ))
               .onFalse(new FeedinCommand(i_index, 
                    () -> 0.0,  // returns a DoubleSupplier
                    () -> 0.0,  // retunrns a DoubleSupplier
                    () -> 0.0   // retunrns a DoubleSupplier
                ));
        
        /* intake  */ 
            operator.leftBumper()
             .onTrue(new IntakeCommand(i_intake, 
                    () -> SmartDashboard.getNumber(Constants.IntakeConstants.intakeSpeedString, Constants.IntakeConstants.kUpperIntakeMotorSpeed)) 
                )
             .onFalse(new IntakeCommand(i_intake, 
                    () -> 0.0)
             );
           
             operator.rightBumper()
             .onTrue(new IntakeCommand(i_intake, 
                    () -> -SmartDashboard.getNumber(Constants.IntakeConstants.intakeSpeedString, Constants.IntakeConstants.kUpperIntakeMotorSpeed))
                )
             .onFalse(new IntakeCommand(i_intake, 
                    () -> 0.0)
             );

            i_intake.setDefaultCommand(
                new IntakeJoystickCommand(
                    i_intake, 
                    () -> operator.getRawAxis(XboxController.Axis.kLeftY.value)
                ));

            operator.pov(0)
            .whileTrue(new InstantCommand(() ->
                i_intake.turnDegrees(Constants.IntakeConstants.kPivotMotorDegree), i_intake));

            operator.pov(90)//Speed up laucher
            .onTrue(new InstantCommand(() ->changeLauncherSpeed(1)));
        

            operator.pov(180)
            .whileTrue(new InstantCommand(() ->
                i_intake.turnDegrees(-Constants.IntakeConstants.kPivotMotorDegree), i_intake
                ));
            operator.pov(270) //Slow down launcher
            .onTrue(new InstantCommand(() ->changeLauncherSpeed(2)));

        drivetrain.registerTelemetry(logger::telemeterize);

         SmartDashboard.putData("Auto Mode", autoChooser);
    }

    private void setMaxSpeed(boolean slow){
        if(slow){
            MaxSpeed = 0.25 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
            MaxAngularRate = 0.5*RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity
        } else {
            MaxSpeed = 0.7 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
            MaxAngularRate = 1.0*RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity
        }
    }
    private void changeLauncherSpeed(int adjustLauncherSpeed){
        if(adjustLauncherSpeed==1){
            l_launch.adjustShootSpeed(100);
        }else if(adjustLauncherSpeed==2){
            l_launch.adjustShootSpeed(-100);
        }

    }
    public Command getAutonomousCommand() {
        
         return autoChooser.getSelected();
    }
    
    
    private void defineAutoCommands() {

         //TODO:CHECK FEED IN COMMANDS, INDEX ON/OFF (changed indexermotor configs), INTAKE, 


        NamedCommands.registerCommand("Launcher On", new LauncherTurn(l_launch, true));
        NamedCommands.registerCommand("Launcher Off", new LauncherTurn(l_launch, false));
        NamedCommands.registerCommand("Conveyer On", new ConveyerTurn(i_index, Constants.IndexerConstants.ConveyerMotorSpeed));
        NamedCommands.registerCommand("Conveyer Off", new ConveyerTurn(i_index,0.0));
        NamedCommands.registerCommand("Index On", new IndexerCommand(i_index, Constants.IndexerConstants.kIndexMotorSpeed, Constants.IndexerConstants.kAgitateMotorSpeed));
        NamedCommands.registerCommand("Index Off", new IndexerCommand(i_index, 0.0,0.0));
        NamedCommands.registerCommand("Intake On", new IntakeCommand(i_intake, () -> SmartDashboard.getNumber(Constants.IntakeConstants.intakeSpeedString, Constants.IntakeConstants.kUpperIntakeMotorSpeed)));
        NamedCommands.registerCommand("Feed in", new FeedinCommand(i_index,  
                () -> SmartDashboard.getNumber(Constants.IndexerConstants.converyMotorString, Constants.IndexerConstants.ConveyerMotorSpeed),
                () -> SmartDashboard.getNumber(Constants.IndexerConstants.indexerSpeedString, Constants.IndexerConstants.kIndexMotorSpeed),
                () -> SmartDashboard.getNumber(Constants.IndexerConstants.agitaterSpeedString, Constants.IndexerConstants.kAgitateMotorSpeed)
            ));
        NamedCommands.registerCommand("Feed in 2", new FeedinCommand(i_index,
                () -> SmartDashboard.getNumber(Constants.IndexerConstants.converyMotorString, Constants.IndexerConstants.ConveyerMotorSpeed),
                () -> SmartDashboard.getNumber(Constants.IndexerConstants.indexerSpeedString, Constants.IndexerConstants.kIndexMotorSpeed),
                () -> SmartDashboard.getNumber(Constants.IndexerConstants.agitaterSpeedString, Constants.IndexerConstants.kAgitateMotorSpeed)
            ));
        NamedCommands.registerCommand("Feed off", new FeedinCommand(i_index, 
                () -> 0.0,  // returns as a DoubleSupplier
                () -> 0.0,  // returns as a DoubleSupplier
                () -> 0.0   // returns as a DoubleSupplier
            ));
        NamedCommands.registerCommand("Intake off", new IntakeCommand(i_intake, () -> 0.0));
        NamedCommands.registerCommand("Intake Down", new IntakeRotate(i_intake, -Constants.IntakeConstants.kPivotMotorDegree));
        NamedCommands.registerCommand("Intake Down 2", new IntakeRotate(i_intake, -Constants.IntakeConstants.kPivotMotorDegree));
        NamedCommands.registerCommand("Intake Up", new IntakeRotate(i_intake, Constants.IntakeConstants.kPivotMotorDegree));
        NamedCommands.registerCommand("Intake Up 2", new IntakeRotate(i_intake, Constants.IntakeConstants.kPivotMotorDegree));
        NamedCommands.registerCommand("Timed Intake Up", new IntakeHold(i_intake, () -> 27));
        NamedCommands.registerCommand("Auto Align", new AimToHub(drivetrain, pv_PhotonVisionSubsystem));
    }
}
