package frc.robot.subsystems;

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FollowerType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.TalonFXSensorCollection;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.TrapezoidProfileSubsystem;

import frc.robot.Constants;

public class ShoulderNormal extends SubsystemBase {
    private static final double SHOULDER_MAX_ANGLE = Math.toRadians(30.0);
    private static final double SHOULDER_MIN_ANGLE = Math.toRadians(-65.0);

    public static final double SHOULDER_ANGLE_TOLERANCE_RADIAN = Math.toRadians(3.0);

    private static final double LEADER_CURRENT_LIMIT = 40.0;
    private static final double FOLLOW_CURRENT_LIMIT = 40.0;

    // All units are MKS with angles in Radians

    // Feedforward constants for the shoulder
    private static final double SHOULDER_KS = 0.182; // TODO: This may need to be tuned
    // The following constants are computed from https://www.reca.lc/arm
    private static final double SHOULDER_KG = 0.09; // V
    private static final double SHOULDER_KV = 6.60; // V*sec/rad
    private static final double SHOULDER_KA = 0.01; // V*sec^2/rad

    // Constants to limit the shoulder rotation speed
    private static final double SHOULDER_MAX_VEL_RADIAN_PER_SEC = Units.degreesToRadians(300.0); // 120 deg/sec
    private static final double SHOULDER_MAX_ACC_RADIAN_PER_SEC_SQ = Units.degreesToRadians(450.0); // 120 deg/sec^2

    private static final double SHOULDER_POSITION_OFFSET = 62.0 / 360.0;
    private static final double SHOULDER_OFFSET_RADIAN = SHOULDER_POSITION_OFFSET * 2 * Math.PI;

    // The Shoulder gear ratio is 288, but let's get it exactly.
    // private static final double SHOULDER_GEAR_RATIO = (84.0 /12.0) * (84.0 / 18.0) * (84.0 / 26.0) * (60.0 / 22.0);
    private static final double SHOULDER_GEAR_RATIO = (84.0 / 12.0) * (84.0 / 18.0) * (70.0 / 40.0) * (60.0 / 22.0);

    // PID Constants for the shoulder PID controller
    // Since we're using Trapeziodal control, all values will be 0 except for P
    private static final double SHOULDER_K_P = 0.15;
    private static final double SHOULDER_K_I = 0.0;
    private static final double SHOULDER_K_D = 0.0;
    private static final double SHOULDER_K_FF = 0.0;
    private static final int kPIDLoopIdx = 0;
    private static final int kTimeoutMs = 0;

    // The TalonFX, the integrated motor controller for the Falcon, uses ticks as it's noative unit.
    // There are 2048 ticks per revolution. Need to account for the gear ratio.
    private static final double SHOULDER_RADIAN_PER_UNIT = 2 * Math.PI / (2048.0 * SHOULDER_GEAR_RATIO);

    // private final ArmFeedforward m_feedForward = new ArmFeedforward(SHOULDER_KS, SHOULDER_KG, SHOULDER_KV,
    // SHOULDER_KA);

    // Define the motor and encoders
    private final WPI_TalonFX m_motorLeader;
    private final WPI_TalonFX m_motorFollower;
    private final TalonFXSensorCollection m_encoder;
    DutyCycleEncoder m_dutyEncoder;

    // The P gain for the PID controller that drives this shoulder.
    private double m_kPShoulder = SHOULDER_K_P;

    // current goal in radians
    private double m_goal;

    public ShoulderNormal() {
        m_motorLeader = new WPI_TalonFX(Constants.SHOULDER_CAN_ID_LEADER);
        m_motorFollower = new WPI_TalonFX(Constants.SHOULDER_CAN_ID_FOLLOWER);

        m_motorLeader.configFactoryDefault();
        m_motorFollower.configFactoryDefault();

        m_encoder = m_motorLeader.getSensorCollection();

        m_motorLeader.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, kPIDLoopIdx, kTimeoutMs);
        // Set follower
        m_motorFollower.follow(m_motorLeader, FollowerType.PercentOutput);

        m_motorLeader.config_kF(kPIDLoopIdx, SHOULDER_K_FF, kTimeoutMs);
        m_motorLeader.config_kP(kPIDLoopIdx, SHOULDER_K_P, kTimeoutMs);
        m_motorLeader.config_kI(kPIDLoopIdx, SHOULDER_K_I, kTimeoutMs);
        m_motorLeader.config_kD(kPIDLoopIdx, SHOULDER_K_D, kTimeoutMs);

        m_dutyEncoder = new DutyCycleEncoder(0);

        // Encoder distance is in radians
        m_dutyEncoder.setDistancePerRotation(2 * Math.PI);
        m_dutyEncoder.setPositionOffset(SHOULDER_POSITION_OFFSET);

        double initialAngle = -m_dutyEncoder.getDistance();
        // SmartDashboard.putNumber("shoulder/initAngle", Units.radiansToDegrees(initialAngle));

        // Set the motor encoder and Position setpoint to the initialAngle from the absolute encoder
        m_encoder.setIntegratedSensorPosition(initialAngle / SHOULDER_RADIAN_PER_UNIT, 0);

        // m_motorLeader.setSelectedSensorPosition(-m_Duty_Encoder.getDistance());
        // m_motorLeader.setSelectedSensorPosition(m_encoder.getIntegratedSensorPosition());
        // m_motorLeader.setSelectedSensorPosition(initialAngle / SHOULDER_RADIAN_PER_UNIT);
        // SmartDashboard.putNumber("shoulder/motorLeaderIntegSensPos", m_motorLeader.getSelectedSensorPosition());

        // m_motorLeader.set(ControlMode.Position, )
        // m_motorLeader.set(ControlMode.Position, m_encoder.getIntegratedSensorPosition(),
        // DemandType.ArbitraryFeedForward, 0.0);//feedforward/12.0);
        // setAngle(m_encoder.getIntegratedSensorPosition() * SHOULDER_RADIAN_PER_UNIT);
    }

    @Override
    public void periodic() {
        // Display current values on the SmartDashboard
        // Add some extra numbers to diagnose the load on the motors
        SmartDashboard.putNumber("shoulder/leaderOutput", m_motorLeader.get());
        SmartDashboard.putNumber("shoulder/encoder", Math.toDegrees(getAngle()));
        SmartDashboard.putNumber("shoulder/encoderSpeed", Math.toDegrees(getSpeed()));
        SmartDashboard.putNumber("shoulder/goal", Math.toDegrees(m_goal));
        SmartDashboard.putNumber("shoulder/absoluteEncoder", Math.toDegrees(-m_dutyEncoder.getDistance()));

        set(m_goal / SHOULDER_RADIAN_PER_UNIT);
    }

    public void set(double position) {
        // Add the feedforward to the PID output to get the motor output
        // The ArmFeedForward computes in radians. We need to convert back to degrees.
        // Remember that the encoder was already set to account for the gear ratios.

        m_motorLeader.set(ControlMode.Position, position);
    }

    // return current shoulder angle in radians
    public double getAngle() {
        // return m_encoder.getIntegratedSensorAbsolutePosition() * SHOULDER_RADIAN_PER_UNIT;
        return m_encoder.getIntegratedSensorPosition() * SHOULDER_RADIAN_PER_UNIT;
    }

    // return current shoulder angular speed in radians/sec
    public double getSpeed() {
        return m_encoder.getIntegratedSensorVelocity() * SHOULDER_RADIAN_PER_UNIT;
    }

    public void resetShoulderPos() {
        setAngle(getAngle());
    }

    // needs to be public so that commands can get the restricted angle
    public static double limitShoulderAngle(double angle) {
        return Math.min(SHOULDER_MAX_ANGLE, Math.max(SHOULDER_MIN_ANGLE, angle));
    }

    // set shoulder angle in radians
    public void setAngle(double angle) {
        m_goal = limitShoulderAngle(angle);
    }
}
