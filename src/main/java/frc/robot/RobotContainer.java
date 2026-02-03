// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.FlywheelSubsystem;
import frc.robot.subsystems.Intake;

import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.controller.BangBangController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final FlywheelSubsystem m_exampleSubsystem = new FlywheelSubsystem(31,32,33,34);
  private final CommandXboxController m_driverController = new CommandXboxController(OperatorConstants.kDriverControllerPort);
  private final Intake m_intake = new Intake(35, 36, 37);
  PIDController controller = new PIDController(4374, 4860, 5400);
  PIDController controller2 = new PIDController(6000, 0, 0);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    SmartDashboard.putNumber("Motor 1 RPM", 0);
    SmartDashboard.putNumber("Motor 2 RPM", 0);
    SmartDashboard.putNumber("Motor 3 RPM", 0);
    SmartDashboard.putNumber("Motor 4 RPM", 0);
    SmartDashboard.putData(controller);
    SmartDashboard.putData(controller2);
    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    m_intake.setDefaultCommand(
      new RunCommand(() -> m_intake.stop(), m_intake)
    );
    // m_driverController.x().whileTrue(
    //   new RunCommand(() -> m_exampleSubsystem.setWheelRPM(-controller.getP(), -controller.getI(), -controller.getD(), -controller2.getP()))
    // );
    // m_driverController.x().whileFalse(
    //   new RunCommand(() -> m_exampleSubsystem.stop())
    // );

    m_driverController.y().whileTrue(
      new RunCommand(() -> m_intake.runPivotMotor(0.1), m_intake)
    );
    m_driverController.a().whileTrue(
      new RunCommand(() -> m_intake.runPivotMotor(-0.1), m_intake)
    );
    m_driverController.b().whileTrue(
      new RunCommand(() -> m_intake.runIntake(0.5), m_intake)
    );
    m_driverController.b().whileTrue(
      new RunCommand(() -> m_intake.runPivotToSetpoint(0))
    );
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return Commands.none();
  }
  
  public void periodic() {
  }
}
