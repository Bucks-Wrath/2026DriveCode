// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.commands.Intake.RunIntake;
import frc.robot.commands.Intake.StopIntake;
import frc.robot.commands.IntakePivot.ToggleIntakePivotPosition;
import frc.robot.commands.Serializer.RunSerializer;
import frc.robot.commands.Serializer.StopSerializer;
import frc.robot.commands.Upkicker.RunUpkicker;
import frc.robot.commands.Upkicker.StopUpkicker;
import frc.robot.generated.TunerConstants;

public class RobotContainer {
    private double MaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    private final Joystick driver = new Joystick(0);
    private final Joystick operator = new Joystick(1);
    private final CommandXboxController DriverController = new CommandXboxController(0);
    private final CommandXboxController OperatorController = new CommandXboxController(1);

    // driver buttons
    private final JoystickButton shootButton = new JoystickButton(driver, XboxController.Button.kRightBumper.value);
    private final JoystickButton intakeButton = new JoystickButton(driver, XboxController.Button.kLeftBumper.value);

    // operator buttons
    private final JoystickButton ShooterSpeedAdjustUpButton = new JoystickButton(operator, XboxController.Button.kStart.value);
    private final JoystickButton ShooterSpeedAdjustDownButton = new JoystickButton(operator, XboxController.Button.kBack.value);

    //subsystems
    public static Shooter shooter = new Shooter();
    public static ShooterLimelight shooterLimelight = new ShooterLimelight();
    public static Agitator agitator = new Agitator();
    public static Serializer serializer = new Serializer();
    public static Upkicker upkicker = new Upkicker();
    public static Intake intake = new Intake();
    public static Hood hood = new Hood();
    public static Turret turret = new Turret();
    public static IntakePivot intakePivot = new IntakePivot();

    public RobotContainer() {
        configureBindings();

        // Sets Default Commands for intake and Serializer motors
        intake.setDefaultCommand(new StopIntake());
        serializer.setDefaultCommand(new StopSerializer());
        upkicker.setDefaultCommand(new StopUpkicker());
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-DriverController.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-DriverController.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-DriverController.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );

        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );

        // driver button configurations
        shootButton.whileTrue(new RunSerializer());  
        shootButton.whileFalse(new StopSerializer());
        shootButton.whileTrue(new RunUpkicker());  
        shootButton.whileFalse(new StopUpkicker());

        intakeButton.whileTrue(new RunIntake());
        intakeButton.whileFalse(new StopIntake());


        DriverController.a().whileTrue(drivetrain.applyRequest(() -> brake));
        DriverController.b().whileTrue(drivetrain.applyRequest(() ->
            point.withModuleDirection(new Rotation2d(-DriverController.getLeftY(), -DriverController.getLeftX()))
        ));

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        DriverController.back().and(DriverController.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        DriverController.back().and(DriverController.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        DriverController.start().and(DriverController.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        DriverController.start().and(DriverController.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // Reset the field-centric heading on left bumper press.
        DriverController.leftBumper().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

        drivetrain.registerTelemetry(logger::telemeterize);

        // operator button configurations
        // operator buttons to increment shooter speed up and down
        ShooterSpeedAdjustUpButton.onTrue(new InstantCommand( () -> shooter.changeShooterSpeed(0.1) ));
        ShooterSpeedAdjustDownButton.onTrue(new InstantCommand(() -> shooter.changeShooterSpeed(-0.1) ));

        // operator button to toggle the intake up and down
        OperatorController.a().onTrue(new ToggleIntakePivotPosition());
    }

    /* Sets Joystick Deadband */
    public static double stickDeadband(double value, double deadband, double center) {
        return (value < (center + deadband) && value > (center - deadband)) ? center : value;
    }

    /* Passes Along Joystick Values for Elevator and Wrist */
    public double getOperatorLeftStickY() {
       return stickDeadband(this.OperatorController.getRawAxis(1), 0.05, 0.0);
    }

    public double getOperatorLeftStickX() {
        return stickDeadband(this.OperatorController.getRawAxis(0), 0.05, 0.0);
    }

    public double getOperatorRightStickY() {
        return stickDeadband(this.OperatorController.getRawAxis(5), 0.05, 0.0);
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
