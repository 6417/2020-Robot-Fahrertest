package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PneumaticSubsystem extends SubsystemBase{
   private static PneumaticSubsystem instance;
   private Compressor compressor;

   public static PneumaticSubsystem getInstance() {
       if (instance == null) {
           instance = new PneumaticSubsystem();
       }
       return instance;
   }

   private PneumaticSubsystem() {
       compressor = new Compressor(30, PneumaticsModuleType.CTREPCM);
       compressor.enableDigital();

       
   }
}
