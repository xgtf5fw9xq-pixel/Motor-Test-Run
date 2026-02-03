package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * TalonFX velocity control using VelocityVoltage (closed-loop velocity with voltage output).
 *
 * Notes on units:
 * - Phoenix 6 velocity setpoint is in rotations per second (RPS) at the sensor (rotor).
 * - getVelocity() returns RPS.
 *
 * If you want mechanism RPM (wheel RPM), include your gear ratio.
 */
public class FlywheelSubsystem extends SubsystemBase {

  private final TalonFX motor1;
  private final TalonFX motor2;
  private final TalonFX motor3;
  private final TalonFX motor4;

  // Control request object you reuse each loop
  private final VelocityVoltage velocityRequest = new VelocityVoltage(0.0);

  // If your flywheel is geared (motor -> wheel), set this.
  // Example: if motor spins 2x for 1x wheel, gearRatio = 2.0

  public FlywheelSubsystem(int id1, int id2, int id3, int id4) {
    this.motor1 = new TalonFX(id1);
    this.motor2 = new TalonFX(id2);
    this.motor3 = new TalonFX(id3);
    this.motor4 = new TalonFX(id4);

    // Basic motor setup
    motor1.setNeutralMode(NeutralModeValue.Coast);
    motor2.setNeutralMode(NeutralModeValue.Coast);
    motor3.setNeutralMode(NeutralModeValue.Coast);
    motor4.setNeutralMode(NeutralModeValue.Coast);

    // Configure PID + feedforward (start values, you WILL tune these)
    TalonFXConfiguration cfg = new TalonFXConfiguration();

    Slot0Configs slot0 = new Slot0Configs();
    slot0.kP = 0.10; // tune
    slot0.kI = 0.00; // tune
    slot0.kD = 0.00; // tune
    slot0.kS = 0.00; // tune (static friction FF)
    slot0.kV = 0.095; // tune (velocity FF)
    slot0.kA = 0.00; // usually 0 for simple velocity
    cfg.Slot0 = slot0;

    // Optional: current limits, voltage limits, etc. add as needed
    // cfg.CurrentLimits.SupplyCurrentLimitEnable = true;
    // cfg.CurrentLimits.SupplyCurrentLimit = 40;

    motor1.getConfigurator().apply(cfg);
    motor2.getConfigurator().apply(cfg);
    motor3.getConfigurator().apply(cfg);
    motor4.getConfigurator().apply(cfg);
    // Optional: make velocity control feel snappier/less noisy
    // velocityRequest.WithEnableFOC(true); // if you want FOC (works on supported hardware)
  }

  /** Command wheel RPM (mechanism RPM), converted to motor RPS internally. */
  public void setWheelRPM(
    double wheel1RPM,
    double wheel2RPM,
    double wheel3RPM,
    double wheel4RPM
  ) {
    double wheelRPS1 = wheel1RPM / 60.0;
    double wheelRPS2 = wheel2RPM / 60.0;
    double wheelRPS3 = wheel3RPM / 60.0;
    double wheelRPS4 = wheel4RPM / 60.0;

    // 18:15 means motor spins faster than the wheel by 18/15
    final double kGearRatio34 = 15.0 / 18.0;

    double motorRPS1 = wheelRPS1;
    double motorRPS2 = wheelRPS2;
    double motorRPS3 = wheelRPS3 * kGearRatio34;
    double motorRPS4 = wheelRPS4 * kGearRatio34;

    motor1.setControl(velocityRequest.withVelocity(-motorRPS1));
    motor2.setControl(velocityRequest.withVelocity(motorRPS2));
    motor3.setControl(velocityRequest.withVelocity(-motorRPS3));
    motor4.setControl(velocityRequest.withVelocity(motorRPS4));
  }

  /** Command motor RPM directly (rotor RPM). */
  public void setMotorRPM(
    double motor1RPM,
    double motor2RPM,
    double motor3RPM,
    double motor4RPM
  ) {
    double motor1RPS = motor1RPM / 60.0;
    double motor2RPS = motor2RPM / 60.0;
    double motor3RPS = motor3RPM / 60.0;
    double motor4RPS = motor4RPM / 60.0;
    motor1.setControl(velocityRequest.withVelocity(motor1RPS));
    motor2.setControl(velocityRequest.withVelocity(motor2RPS));
    motor3.setControl(velocityRequest.withVelocity(motor3RPS));
    motor4.setControl(velocityRequest.withVelocity(motor4RPS));
  }

  public void stop() {
    motor1.set(0.0);
    motor2.set(0.0);
    motor3.set(0.0);
    motor4.set(0.0);
  }

  public double getMotorRPS() {
    return motor4.getVelocity().getValueAsDouble();
  }

  public double getMotorRPM() {
    return getMotorRPS() * 60.0;
  }

  public double getWheelRPM() {
    double motorRPS = getMotorRPS();
    double wheelRPS = motorRPS;
    return wheelRPS * 60.0;
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Flywheel/MotorRPM", getMotorRPM());
    SmartDashboard.putNumber("Flywheel/WheelRPM", getWheelRPM());
    SmartDashboard.putNumber(
      "Flywheel/ClosedLoopErrorRPS",
      motor1.getClosedLoopError().getValueAsDouble()
    );
  }
}
