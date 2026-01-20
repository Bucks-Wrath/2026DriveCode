package frc.robot.subsystems;

import frc.lib.models.*;
import frc.robot.DeviceIds;
import frc.robot.Robot;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Hood extends SubsystemBase implements IPositionControlledSubsystem {

	private boolean isHoldingPosition = false;

    // Set Different Heights
	private double homePosition = 0;
	private double maxUpTravelPosition = 52.3; //TODO: find this value later

	public double upPositionLimit = maxUpTravelPosition;
	public double downPositionLimit = 0;
	private double targetPosition = 0;
    private MotionMagicDutyCycle targetPositionDutyCycle = new MotionMagicDutyCycle(0);
	private double feedForward = 0.0;
	public double shooterAddValue;

	private final static double onTargetThreshold = 0.25; //TODO: find this value later
		
	private TalonFX HoodKraken = new TalonFX(DeviceIds.Hood.MotorId);

    private TalonFXConfiguration HoodFXConfig = new TalonFXConfiguration();

	public Hood() {
		// Clear Sticky Faults
		this.HoodKraken.clearStickyFaults();
		
        /** Hood Motor Configuration */
        /* Motor Inverts and Neutral Mode */
		HoodFXConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive; //TODO: find this direction later
        HoodFXConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        /* Current Limiting */
		HoodFXConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        HoodFXConfig.CurrentLimits.StatorCurrentLimit = 20; //TODO: find this value later

        /* PID Config */
        HoodFXConfig.Slot0.kP = 0.2; //TODO: tune
        HoodFXConfig.Slot0.kI = 0; 
        HoodFXConfig.Slot0.kD = 0.01; 

        /* Open and Closed Loop Ramping */
        HoodFXConfig.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = 0.25;
        HoodFXConfig.OpenLoopRamps.VoltageOpenLoopRampPeriod = 0.25;

        HoodFXConfig.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = 0;
        HoodFXConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 0;

        //Config Acceleration and Velocity
        HoodFXConfig.MotionMagic.withMotionMagicAcceleration(300); //TODO: tune
        HoodFXConfig.MotionMagic.withMotionMagicCruiseVelocity(300);

        // Config Motor
        HoodKraken.getConfigurator().apply(HoodFXConfig);
        HoodKraken.getConfigurator().setPosition(0.0);
	}

	public void motionMagicControl() {
		this.manageMotion(targetPosition);
        targetPositionDutyCycle.withPosition(targetPosition);
        targetPositionDutyCycle.withFeedForward(feedForward);
		this.HoodKraken.setControl(targetPositionDutyCycle);
	}

	public double getCurrentPosition() {
		return this.HoodKraken.getRotorPosition().getValueAsDouble();
	}

	public double getCurrentDraw() {
		return this.HoodKraken.getSupplyCurrent().getValueAsDouble();
	}

	public boolean isHoldingPosition() {
		return this.isHoldingPosition;
	}

	public void setIsHoldingPosition(boolean isHoldingPosition) {
		this.isHoldingPosition = isHoldingPosition;
	}

	public double getTargetPosition() {
		return this.targetPosition;
	}

	public boolean setTargetPosition(double position) {
		if (!isValidPosition(position)) {
			return false;
		} else {
			this.targetPosition = position;
			return true;
		}
	}

	public void forceSetTargetPosition(double position) {
		this.targetPosition = position;
	}

	public void incrementTargetPosition(double increment) {
		double currentTargetPosition = this.targetPosition;
		double newTargetPosition = currentTargetPosition + increment;
		if (isValidPosition(newTargetPosition)) {
			this.targetPosition = newTargetPosition;
		}
	}

	public boolean isValidPosition(double position) {
		boolean withinBounds = position <= upPositionLimit && position >= downPositionLimit;
		return withinBounds;
	}

    // communicate with commands
	public double getHomePosition() {
		return this.homePosition;
	}

	public double getMaxUpTravelPosition() {
		return this.maxUpTravelPosition;
	}

	public double getFeedForward() {
		return this.feedForward;
	}

	public void resetHoodEncoder() {
        try {
			HoodKraken.getConfigurator().setPosition(0.0);
        }
        catch (Exception e) {
            DriverStation.reportError("Hood.resetHoodEncoders exception.  You're Screwed! : " + e.toString(), false);
        }
	}

	public double JoystickHood(){
		double value = 0;
		value = -Robot.m_robotContainer.getOperatorLeftStickY();
		return value;
	}

	public double getPositionError() {
		double currentPosition = this.getCurrentPosition();
		double targetPosition = this.getTargetPosition();
		double positionError = Math.abs(currentPosition - targetPosition);
		return positionError;
	}

	public void manageMotion(double targetPosition) {
		double currentPosition = getCurrentPosition();
		if (currentPosition < targetPosition) {
				// set based on gravity
		}
		else {
				//set based on gravity
		}
	}

	public void zeroTarget() {
		targetPosition = 0;
	}

	public void updateDashboard() {
		SmartDashboard.putNumber("Hood Position", this.getCurrentPosition());
		SmartDashboard.putNumber("Hood Target Position", this.getTargetPosition());
		SmartDashboard.putNumber("Hood Position Error", this.getPositionError());
		SmartDashboard.putNumber("Hood Velocity", this.getCurrentVelocity());
		SmartDashboard.putNumber("Hood Current", this.getCurrentDraw());
	}

	@Override
	public double getCurrentVelocity() {
		double currentVelocity = this.HoodKraken.getVelocity().getValueAsDouble();
		return currentVelocity;
	}

	@Override
	public boolean isInPosition(double targetPosition) {
		double currentPosition = this.getCurrentPosition();
		double positionError = Math.abs(currentPosition - targetPosition);
		if (positionError < onTargetThreshold) {
			return true;
		} else {
			return false;
		}
	}
}   