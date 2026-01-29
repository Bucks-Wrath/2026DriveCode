package frc.robot.commands.Turret;

import frc.robot.RobotContainer;
import frc.robot.config.AutoAimConfig;
import frc.robot.subsystems.ShooterLimelight;

import edu.wpi.first.wpilibj2.command.Command;

public class AutoAim extends Command {      

    private double tx;
    private double ta;
    private double tAng;
    private double rotationVal;
    private double strafeVal;
    private double distanceVal;

    private ShooterLimelight limelight; 

    public AutoAim() {
        this.limelight = RobotContainer.shooterLimelight;
        addRequirements(RobotContainer.shooterLimelight);
        addRequirements(RobotContainer.shooter);
        addRequirements(RobotContainer.hood);
        addRequirements(RobotContainer.turret);
        ;
    }

    public void initialize() {
        limelight.strafeController.setSetpoint(AutoAimConfig.StrafeTarget);
        limelight.distanceController.setSetpoint(AutoAimConfig.DistanceTarget);
        limelight.angleController.setSetpoint(AutoAimConfig.AngleTarget);
    }
    
    @Override
    public void execute() {
        // If we don't see a target, don't do anything
        if (!limelight.ifValidTag()) return;

        // find target location
        tx = limelight.gettx();
        ta = limelight.gettz();
        tAng = limelight.gettAng();
         
        // Uses PID to point at target
        rotationVal = -limelight.angleController.calculate(tAng, AutoAimConfig.AngleTarget);
        strafeVal = limelight.strafeController.calculate(tx, AutoAimConfig.StrafeTarget);
        distanceVal = -limelight.distanceController.calculate(ta, AutoAimConfig.DistanceTarget);

        if(limelight.distanceController.atSetpoint())
           distanceVal = 0;
        if (limelight.strafeController.atSetpoint())
            strafeVal = 0;
        if (limelight.angleController.atSetpoint())
            rotationVal = 0;

        //Distance affects Hood Angle and Shooter Speed
        //TX affects the turret rotation
        //tAng affects the aim left or right of center

        
        
        
    }

    // Make this return true when this Command no longer needs to run execute()
	public boolean isFinished() {
        // If all 3 PIDs are at their target, we're done
		return !limelight.ifValidTag() || (limelight.distanceController.atSetpoint() 
            && limelight.strafeController.atSetpoint() 
            && limelight.angleController.atSetpoint());
	}

	// Called once after isFinished returns true
	protected void end() {

    }

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	protected void interrupted() {
        end(); 
	}

} 
