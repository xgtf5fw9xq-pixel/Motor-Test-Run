// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
  /** Creates a new Intake. */
  TalonFX pivot;
  TalonFX intake1;
  TalonFX intake2;
  MotionMagicVoltage pivotRequest = new MotionMagicVoltage(0.0).withSlot(0);
  public Intake(int pivotid, int intakeid1, int intakeid2) {
    pivot = new TalonFX(pivotid);
    intake1 = new TalonFX(intakeid1);
    intake2 = new TalonFX(intakeid2);
    Slot0Configs slot0 = new Slot0Configs();
    slot0.kP = 0.0;
    slot0.kI = 0.0;
    slot0.kD = 0.0;
    slot0.kG = 0.0;
    pivot.getConfigurator().apply(slot0);
    resetPivot();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Pivot Position", getPivotPosition());
  }

  public void runPivotMotor(double speed) {
    pivot.set(speed);
  }

  public void runIntake(double speed) {
    speed *= -1;
    intake1.set(speed);
    intake2.set(speed);
  }
  public void runPivotToSetpoint(double setpoint) {
    pivot.setControl(pivotRequest.withPosition(setpoint));
  }
  public double getPivotPosition() {
    return pivot.getPosition().getValueAsDouble();
  }
  public void resetPivot() {
    pivot.setPosition(0);
  }
  public void stop() {
    pivot.set(0.0);
    intake1.set(0.0);
    intake2.set(0.0);
  }
}
