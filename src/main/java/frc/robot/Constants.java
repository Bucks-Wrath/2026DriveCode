// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kOperatorControllerPort = 1;
  }

  public static class Serializer{
    public static class Uptake{
        public static final double UptakeRunSpeed = 1.0;
        public static final double UptakeStopSpeed = 0.0;
    }
      public static class Drum{
        public static final double DrumRunSpeed = 0.55;
        public static final double DrumStopSpeed = 0.0;
    }
      public static class Agitator{
        public static final double AgitatorRunSpeed = -0.5;
        public static final double AgitatorSlowSpeed = -0.2;
        public static final double AgitatorStopSpeed = 0.0;
    }
  }

  public static class Intake{
    public static class IntakeRoller{
      public static final double IntakeRunSpeed = 0.5;
      public static final double IntakeStopSpeed = 0.0;
    }
    public static class IntakeWrist{
      public static double StoreIntakePosition = 0.0;
      public static double RunIntakePosition = 6.0;  // todo
    }
  }

  public static class Shooter{
    public static class Hood {
      public static double StoreHoodPosition = 0.0;
      public static double FullUpPosition = 12.8;  //todo
    }
    public static class Turret {
      public static double MinimumTurretPosition = -10;
      public static double MaximumHoodPosition = 10;  //todo
    }
  }

}
