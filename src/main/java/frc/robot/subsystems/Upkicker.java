package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.DeviceIds;

public class Upkicker extends SubsystemBase {

	private TalonFX UpkickerKraken = new TalonFX(DeviceIds.Upkicker.LeadMotorId);
    private TalonFXConfiguration UpkickerFXConfig = new TalonFXConfiguration();
    private TalonFX UpkickerKrakenFollower = new TalonFX(DeviceIds.Upkicker.FollowerMotorId);

    private double speed = 0;


	public Upkicker() {
        /** Upkicker Motor Configuration */
        /* Motor Inverts and Neutral Mode */
		UpkickerFXConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        UpkickerFXConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;

        // Set Followers
		UpkickerKrakenFollower.setControl(new Follower(UpkickerKraken.getDeviceID(), null));


        /* Current Limiting */
        //UpkickerFXConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        //UpkickerFXConfig.CurrentLimits.SupplyCurrentLimit = 20;
        //UpkickerFXConfig.CurrentLimits.SupplyCurrentThreshold = 30;
        //UpkickerFXConfig.CurrentLimits.SupplyTimeThreshold = 0.01;

        UpkickerFXConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        UpkickerFXConfig.CurrentLimits.StatorCurrentLimit = 25;

        /* PID Config */
        UpkickerFXConfig.Slot0.kP = 0.2;
        UpkickerFXConfig.Slot0.kI = 0;
        UpkickerFXConfig.Slot0.kD = 0;

        /* Open and Closed Loop Ramping */
        UpkickerFXConfig.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = 0.1;
        UpkickerFXConfig.OpenLoopRamps.VoltageOpenLoopRampPeriod = 0.1;

        UpkickerFXConfig.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = 0.1;
        UpkickerFXConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 0.1;

        // Config Motor
        UpkickerKraken.getConfigurator().apply(UpkickerFXConfig);
        UpkickerKraken.getConfigurator().setPosition(0.0);
        UpkickerKrakenFollower.getConfigurator().setPosition(0.0);
	}

	public void setSpeed(double speed) {
        this.UpkickerKraken.set(speed);
        this.speed = speed;
	}

    public double getSpeed() {
        return this.speed;
	}

    public void changeUpkickerSpeed(double adjustmentValue){
		this.setSpeed(adjustmentValue + this.getSpeed());
	}

	public double getCurrentDrawLeader() {
		return this.UpkickerKraken.getSupplyCurrent().getValueAsDouble();
	}

    public double getCurrentDrawFollower() {
		return this.UpkickerKrakenFollower.getSupplyCurrent().getValueAsDouble();
	}

	public void resetUpkickerEncoder() {
        try {
			UpkickerKraken.getConfigurator().setPosition(0.0);
            UpkickerKrakenFollower.getConfigurator().setPosition(0.0);
        }
        catch (Exception e) {
            DriverStation.reportError("Upkicker.resetUpkickerEncoders exception.  You're Screwed! : " + e.toString(), false);
        }
	}

	public void updateDashboard() {
		SmartDashboard.putNumber("Upkicker Current", this.getCurrentDrawLeader());
        SmartDashboard.putNumber("Upkicker Follower Current", this.getCurrentDrawFollower());
	}
}