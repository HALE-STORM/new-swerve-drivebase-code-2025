// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;
import frc.robot.subsystems.AutoCommands.AutoCommands;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;


import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class RobotContainer {

    public AutoCommands bot = new AutoCommands();

    private double MaxSpeed = 4;//TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandPS5Controller joystick = new CommandPS5Controller(0);
    private final CommandPS4Controller operator = new CommandPS4Controller(1);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    public RobotContainer() {
        NamedCommands.registerCommand("Run Shooter", bot.runShooter());
        NamedCommands.registerCommand("Stop Shooter/Eject",bot.stopShooter());
        NamedCommands.registerCommand("Run Eject", bot.ejectShooter());
        NamedCommands.registerCommand("Smart Shooter", bot.smartShooter());
        NamedCommands.registerCommand("Smart Intake",bot.smartIntake());
        NamedCommands.registerCommand("Elevator Level 1", bot.elevatorHeightFirst());
        NamedCommands.registerCommand("Elevator Level 2", bot.elevatorHeightSecond());
        NamedCommands.registerCommand("2test", bot.elevatorHeightSecond());
        NamedCommands.registerCommand("Elevator Bottom Algae", bot.elevatorFirstAlgae());
        NamedCommands.registerCommand("Elevator Top Algae", bot.elevatorSecondAlgae());
        NamedCommands.registerCommand("Elevator Base", bot.resetElevator());

        configureBindings();
    }




    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-joystick.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-joystick.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-joystick.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );

        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );

        //joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
        joystick.triangle().whileTrue(drivetrain.applyRequest(() ->
            point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))
        ));

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        joystick.create().and(joystick.triangle()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        joystick.create().and(joystick.square()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        joystick.options().and(joystick.triangle()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        joystick.options().and(joystick.square()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // reset the field-centric heading on left bumper press
        operator.square().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));
        //this is for the co-driver
        operator.povRight().onTrue(bot.elevatorHeightSecond());
        operator.povLeft().onTrue(bot.elevatorHeightFirst());
        operator.povDown().onTrue(bot.resetElevator());
        operator.triangle().whileTrue(bot.ejectShooter());
        //below is for driver
        joystick.L1().whileTrue(bot.runShooter());
        joystick.R1().onTrue(bot.smartIntake());
        joystick.cross().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));
        joystick.povRight().onTrue(bot.elevatorHeightSecond());
        joystick.povLeft().onTrue(bot.elevatorHeightFirst());
        joystick.povDown().onTrue(bot.resetElevator());
        joystick.circle().whileTrue(bot.ejectShooter());
        joystick.L2().whileTrue(bot.elevatorFirstAlgae());
        joystick.R2().whileTrue(bot.elevatorSecondAlgae());


        drivetrain.registerTelemetry(logger::telemeterize);
    }

    public Command getAutonomousCommand() {
        return new PathPlannerAuto("left_side_auto_1");
    }
}
