package first.robot.subsystems;

import static org.wpilib.units.Units.MetersPerSecond;
import static org.wpilib.units.Units.Rotations;
import static org.wpilib.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.sim.CANcoderSimState;
import com.ctre.phoenix6.sim.TalonFXSimState;
import org.wpilib.hardware.bus.CANPort;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.kinematics.SwerveModulePosition;
import org.wpilib.math.kinematics.SwerveModuleVelocity;
import org.wpilib.math.system.DCMotor;
import org.wpilib.math.system.Models;
import org.wpilib.math.util.MathUtil;
import org.wpilib.simulation.DCMotorSim;
import org.wpilib.system.RobotController;

import first.robot.Constants.DriveConstants;
import first.robot.Constants.SteerConstants;

/**
 * One swerve corner. No longer a mechanism — the Drivetrain owns four of these
 * and commands them. A single tick of control happens on demand in
 * setDesiredState(); there is no scheduler tick of its own.
 */
public class SwerveModule {
  /** Position of this module relative to robot center, in meters. */
  public final Translation2d location;

  private final TalonFX m_driveMotor;
  private final TalonFX m_steerMotor;
  private final CANcoder m_steerEncoder;
  private final TalonFXSimState m_driveSim;
  private final TalonFXSimState m_steerSim;

  private final CANcoderSimState m_steerEncoderSim;

  // Physics models depend only on constants, so they still initialize inline.
  private final DCMotorSim m_driveModel =
      new DCMotorSim(
          Models.singleJointedArmFromPhysicalConstants(
              DCMotor.getKrakenX60(1), 0.025, DriveConstants.kDriveGearRatio),
          DCMotor.getKrakenX60(1));
  private final DCMotorSim m_steerModel =
      new DCMotorSim(
          Models.singleJointedArmFromPhysicalConstants(
              DCMotor.getKrakenX60(1), 0.004, SteerConstants.kSteerGearRatio),
          DCMotor.getKrakenX60(1));
  
  private final PositionVoltage m_steerRequest = new PositionVoltage(0);
  private final VelocityVoltage m_driveRequest = new VelocityVoltage(0);
  public SwerveModule(
      int driveId, int steerId, int cancoderId, double magnetOffsetRotations,
      Translation2d location) {
    this.location = location;
    m_driveMotor = new TalonFX(driveId, new CANBus(CANPort.CAN_S0));
    m_steerMotor = new TalonFX(steerId, new CANBus(CANPort.CAN_S0));
    m_steerEncoder = new CANcoder(cancoderId, new CANBus(CANPort.CAN_S0));
    m_driveSim = m_driveMotor.getSimState();
    m_steerSim = m_steerMotor.getSimState();
    m_steerEncoderSim = m_steerEncoder.getSimState();

    // Same CANcoder priming as Lesson 5, now paid off for the real 25:1 ratio:
    // seed the motor's rotor-side counter, not the wheel-side reading.
    CANcoderConfiguration cancoderConfig = new CANcoderConfiguration();
    cancoderConfig.MagnetSensor.MagnetOffset = magnetOffsetRotations;
    m_steerEncoder.getConfigurator().apply(cancoderConfig);

    // Which way the steering motor counts: a fact about your gearbox, set in SteerConstants.
    TalonFXConfiguration steerConfig = new TalonFXConfiguration();
    steerConfig.MotorOutput.Inverted = SteerConstants.kSteerInverted;
    steerConfig.Feedback.FeedbackRemoteSensorID = cancoderId;
    steerConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
    steerConfig.Feedback.RotorToSensorRatio = SteerConstants.kSteerGearRatio;
    steerConfig.Feedback.SensorToMechanismRatio = SteerConstants.kSteerGearRatio;
    steerConfig.ClosedLoopGeneral.ContinuousWrap = true;
    steerConfig.Slot0.kP = SteerConstants.kSteerKP;
    m_steerMotor.getConfigurator().apply(steerConfig);
    //m_steerMotor.setPosition(
    //    m_steerEncoder.getAbsolutePosition().getValue().in(Rotations) * SteerConstants.kSteerGearRatio);
    TalonFXConfiguration driveConfig = new TalonFXConfiguration();
    driveConfig.Feedback.SensorToMechanismRatio = DriveConstants.kDriveGearRatio;
    driveConfig.Slot0.kV = DriveConstants.kDriveKV;
    driveConfig.Slot0.kP = DriveConstants.kDriveKP;
    m_driveMotor.getConfigurator().apply(driveConfig);
  }

  /**
   * ====== NEXT LESSON: CHANGE THE CODE BELOW ======
   * Take a SwerveModuleVelocity — a speed in meters per second plus an angle — instead
   * of two bare numbers. Wrap the steering error with MathUtil.inputModulus instead of
   * the two while loops, and turn the speed into a fraction of max speed before
   * applying the cosine scale.
   */

  /** One tick of control: steer toward 'angleDegrees', drive at 'speedFraction'. */
  public void setDesiredState(SwerveModuleVelocity state) {
    m_steerMotor.setControl(m_steerRequest.withPosition(state.angle.getMeasure()));

    double error = state.angle.getDegrees() - getSteerAngleDegrees();
    double alignment = Math.cos(Math.toRadians(error));
    double wheelRps = state.velocity * alignment / DriveConstants.kWheelCircumferenceMeters;
    m_driveMotor.setControl(m_driveRequest.withVelocity(RotationsPerSecond.of(wheelRps)));
  }

  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(getDistanceMeters(), Rotation2d.fromDegrees(getSteerAngleDegrees()));
  }

  /** Zero the drive encoder — start measuring distance from *here*. */
  public void resetDrivePosition() {
    m_driveMotor.setPosition(0);
  }

  /** Keeps 'value' between 'min' and 'max'. */
  private double clamp(double value, double min, double max) {
    if (value > max) {
      return max;
    } else if (value < min) {
      return min;
    } else {
      return value;
    }
  }

  /** Current steering angle in degrees (through the real 25:1 reduction). */
  public double getSteerAngleDegrees() {
    return m_steerMotor.getPosition().getValue().in(Rotations) * 360;
  }

  /** How far this module's wheel has driven, in meters, since the last reset. */
  public double getDistanceMeters() {
    return m_driveMotor.getPosition().getValue().in(Rotations) * DriveConstants.kWheelCircumferenceMeters;
  }

  /** Current wheel speed in meters per second. */
  public double getDriveVelocityMetersPerSec() {
    return m_driveMotor.getVelocity().getValue().in(RotationsPerSecond) * DriveConstants.kWheelCircumferenceMeters;
  }

  /** Advances the physics model by one tick. Only ever called in simulation. */
  public void simulatePeriodic() {
    // Drive motor: model reports wheel motion, convert back to rotor-side.
    m_driveSim.setSupplyVoltage(RobotController.getBatteryVoltage());
    m_driveModel.setInputVoltage(m_driveSim.getMotorVoltage());
    m_driveModel.update(0.020);
    m_driveSim.setRawRotorPosition(
        m_driveModel.getAngularPosition() / (2 * Math.PI) * DriveConstants.kDriveGearRatio);
    m_driveSim.setRotorVelocity(
        m_driveModel.getAngularVelocity() / (2 * Math.PI) * DriveConstants.kDriveGearRatio);

    // Steer motor: same, through the 25:1 reduction.
    m_steerSim.setSupplyVoltage(RobotController.getBatteryVoltage());
    m_steerModel.setInputVoltage(m_steerSim.getMotorVoltage());
    m_steerModel.update(0.020);
    m_steerSim.setRawRotorPosition(
        m_steerModel.getAngularPosition() / (2 * Math.PI) * SteerConstants.kSteerGearRatio);
    m_steerSim.setRotorVelocity(
        m_steerModel.getAngularVelocity() / (2 * Math.PI) * SteerConstants.kSteerGearRatio);
    m_steerEncoderSim.setRawPosition(m_steerModel.getAngularPosition() / (2 * Math.PI));
    m_steerEncoderSim.setVelocity(m_steerModel.getAngularVelocity() / (2 * Math.PI));
  }
}
