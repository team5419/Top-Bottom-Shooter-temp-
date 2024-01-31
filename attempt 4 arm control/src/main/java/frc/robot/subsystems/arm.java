// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

//import com.ctre.phoenix6.hardware.CANCoder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.wpilibj.DutyCycle;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.ControlModeValue;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;


//test
public class arm extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  // private final PIDController pid = new PIDController(1,0,0); // change for talonFX pid 
  
  private final TalonFX motor;
  private final TalonFX follower;
  private final CANcoder encoder;

  private final MotionMagicVoltage positioner = new MotionMagicVoltage(0);
  //final TrapezoidProfile m_profile = new TrapezoidProfile(
  // new TrapezoidProfile.Constraints(80, 160)
  //);
 //. TrapezoidProfile.State m_goal = new TrapezoidProfile.State(10, 0);
 // TrapezoidProfile.State m_setpoint = new TrapezoidProfile.State();
  private final DutyCycleOut ds = new DutyCycleOut(0).withOverrideBrakeDurNeutral(true);

  private double mSelectedPosition = 0;

  public arm() {
    //right bicep
    motor = new TalonFX(42); //change back to 42
    encoder = new CANcoder(43); 

    follower = new TalonFX(41);
    follower.setControl(new Follower(42, true));

    // motor.getConfigurator().apply(slot0Configs);
    
    var talonFXConfigs = new TalonFXConfiguration();

    // set slot 0 gains
    var slot0Configs = talonFXConfigs.Slot0;
    // slot0Configs.kS = 0.24; // add 0.24 V to overcome friction
    // slot0Configs.kV = 0.12; // apply 12 V for a target velocity of 100 rps
    // PID runs on position
    slot0Configs.kP = 4.8;
    slot0Configs.kI = 0;
    slot0Configs.kD = 0.1;

    // set Motion Magic settings
    var motionMagicConfigs = talonFXConfigs.MotionMagic;
    motionMagicConfigs.MotionMagicCruiseVelocity = 40; // 80 rps cruise velocity
    motionMagicConfigs.MotionMagicAcceleration = 160; // 160 rps/s acceleration (0.5 seconds)
    // motionMagicConfigs.MotionMagicJerk = 1600; // 1600 rps/s^2 jerk (0.1 seconds)

    motor.getConfigurator().apply(talonFXConfigs, 0.050);

    // cfg.Feedback.FeedbackRemoteSensorID = 43;
    // cfg.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
    // motor.getConfigurator().apply(cfg);
    //left bicep

  }

  public void armToPosition(double targetPosition){
    SmartDashboard.putNumber("target", targetPosition);

    positioner.Slot = 0;
    motor.setControl(positioner.withPosition(targetPosition));
    //m_setpoint = m_profile.calculate(0.020, m_setpoint, m_goal);
    // apply the setpoint to the control request
    //positioner.Position = m_setpoint.position;
    //positioner.Velocity = m_setpoint.velocity;
    //motor.setControl(positioner);
  }

  public void manual(double movement){
    motor.setControl(ds.withOutput(movement));
  }

  public double position(){
    return motor.getPosition().getValueAsDouble();
  }

  public void stop(){
    motor.set(0);
  }

  public void resetPosition() {
    motor.setPosition(0);
  }
  /**
   * Example command factory method.
   *
   * @return a command
   */

  public Command exampleMethodCommand() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          /* one-time action goes here */
        });
  }

  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putString("taggyvontaggems", "test");
    SmartDashboard.putNumber("Speed", motor.getClosedLoopOutput().getValue());
    SmartDashboard.putNumber("position",motor.getPosition().getValue());
    //SmartDashboard.putNumber("closed loop reference", motor.getClosedLoopReference().getValue());
    SmartDashboard.putNumber("closed loop error", motor.getClosedLoopError().getValue());

  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}