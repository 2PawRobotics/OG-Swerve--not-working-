// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Encoder;

import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
//import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;

public class SwerveModule {
  private static final double kWheelRadius = 0.0508;
  private static final int kEncoderResolution = 4096;
  private static final int kEncoderResolution2 = 2048;

  private static final double kModuleMaxAngularVelocity = Drivetrain.kMaxAngularSpeed;
  private static final double kModuleMaxAngularAcceleration = 2 * Math.PI; // radians per second squared

  private final WPI_TalonFX m_driveMotor;
  private final CANSparkMax m_turningMotor;

  /*
   * private final Encoder m_driveEncoder = new Encoder(0, 1); private final
   * Encoder m_turningEncoder = new Encoder(2, 3);
   */

  public static Encoder m_Encoder;
  public static WPI_TalonFX m_driveEncoder;

  private final PIDController m_drivePIDController = new PIDController(15.23, 0.01, 0.087);

  private final ProfiledPIDController m_turningPIDController = new ProfiledPIDController(1, 0, 0,
      new TrapezoidProfile.Constraints(kModuleMaxAngularVelocity, kModuleMaxAngularAcceleration));

  // Gains are for example purposes only - must be determined for your own robot!
  private final SimpleMotorFeedforward m_driveFeedforward = new SimpleMotorFeedforward(1, 3);
  private final SimpleMotorFeedforward m_turnFeedforward = new SimpleMotorFeedforward(1, 0.5);

  /**
   * Constructs a SwerveModule.
   *
   * @param driveMotorChannel   ID for the drive motor.
   * @param turningMotorChannel ID for the turning motor.
   * 
   */
  public SwerveModule(int driveMotorChannel, int turningMotorChannel, int aChannel, int bChannel) {
    m_driveMotor = new WPI_TalonFX(driveMotorChannel);
    m_turningMotor = new CANSparkMax(turningMotorChannel, MotorType.kBrushed);

    m_Encoder = new Encoder(aChannel, bChannel); 
    m_driveEncoder = new WPI_TalonFX();

    m_driveMotor.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, 60, 65, 0.25));
    
    m_driveMotor.set(m_drivePIDController.calculate(m_driveEncoder.get(), .5));
    m_turningMotor.set(m_turningPIDController.calculate(m_Encoder.getDistance(), .5));

    // Set the distance per pulse for the drive encoder. We can simply use the
    // distance traveled for one rotation of the wheel divided by the encoder
    // resolution.
    //m_Encoder.setDistancePerPulse(2 * Math.PI * kWheelRadius / kEncoderResolution);
    m_driveEncoder.set(2 * Math.PI * kWheelRadius / kEncoderResolution2);
    // Set the distance (in this case, angle) per pulse for the turning encoder.
    // This is the the angle through an entire rotation (2 * wpi::math::pi)
    // divided by the encoder resolution.
    m_Encoder.setDistancePerPulse(2 * Math.PI / kEncoderResolution);

    // Limit the PID Controller's input range between -pi and pi and set the input
    // to be continuous.
    m_turningPIDController.enableContinuousInput(-Math.PI, Math.PI);
  }
  public void periodic() throws InterruptedException{
    m_Encoder.wait(500);
  }

  public boolean getDirection(){
    return m_Encoder.getDirection();
  }

  public double getDistance(){
    return m_Encoder.getDistance();
  }

  public void reset(){
    m_Encoder.reset();
  }
  /**
   * Returns the current state of the module.
   *
   * @return The current state of the module.
   */
  public SwerveModuleState getState() {
    return new SwerveModuleState(m_driveEncoder.getSelectedSensorVelocity(), new Rotation2d(m_Encoder.get()));
  }

  /**
   * Sets the desired state for the module.
   *
   * @param desiredState Desired state with speed and angle.
   */
  public void setDesiredState(SwerveModuleState desiredState) {
    // Optimize the reference state to avoid spinning further than 90 degrees
    SwerveModuleState state = SwerveModuleState.optimize(desiredState, new Rotation2d(m_Encoder.get()));

    // Calculate the drive output from the drive PID controller.
    final double driveOutput = m_drivePIDController.calculate(m_driveEncoder.getSelectedSensorVelocity(), state.speedMetersPerSecond);

    final double driveFeedforward = m_driveFeedforward.calculate(state.speedMetersPerSecond);

    // Calculate the turning motor output from the turning PID controller.
    final double turnOutput = m_turningPIDController.calculate(m_Encoder.get(), state.angle.getRadians());

    final double turnFeedforward =
        m_turnFeedforward.calculate(m_turningPIDController.getSetpoint().velocity);

    m_driveMotor.setVoltage(driveOutput + driveFeedforward);
    m_turningMotor.setVoltage(turnOutput + turnFeedforward);
  }
}
