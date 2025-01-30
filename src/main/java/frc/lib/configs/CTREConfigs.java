// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.



/*for new configs going forward, configs will contain whole set of parameters inside which is
different from last year, watch around minute 1:15 of this video: https://www.youtube.com/watch?v=naRnzRN6Drw */
//Burn to flash has been changed, is now part of running configs, around minute 2:10 of video





// package frc.lib.configs;
// import com.ctre.phoenix.sensors.AbsoluteSensorRange; /*don't think this is used?  if it is here is documentation on 2025 migration
// https://v6.docs.ctr-electronics.com/en/stable/docs/yearly-changes/yearly-changelog.html (look at deprecations/removals) */
// import com.ctre.phoenix6.configs.CANcoderConfiguration;
// import com.ctre.phoenix.sensors.SensorInitializationStrategy;
// import com.ctre.phoenix.sensors.SensorTimeBase;
// import frc.robot.Constants;

// public final class CTREConfigs {
//   public CANcoderConfiguration swerveCanCoderConfig;

//   public CTREConfigs() {
//     swerveCanCoderConfig = new CANcoderConfiguration();
//     /*look to see if there are any samples online of the "Swerve CANCoder Configuration" listed below online (migrated for 2025) */

//     /* Swerve CANCoder Configuration */
//     swerveCanCoderConfig.absoluteSensorRange = AbsoluteSensorRange.Unsigned_0_to_360;
//     swerveCanCoderConfig.sensorDirection = Constants.Swerve.canCoderInvert;
//     swerveCanCoderConfig.initializationStrategy =
//         SensorInitializationStrategy.BootToAbsolutePosition;
//     swerveCanCoderConfig.sensorTimeBase = SensorTimeBase.PerSecond;
//   }
// }