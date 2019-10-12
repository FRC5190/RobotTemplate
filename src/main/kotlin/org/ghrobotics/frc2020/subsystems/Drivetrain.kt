/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *
 * Copyright 2019, Green Hope Falcons
 */

package org.ghrobotics.frc2020.subsystems

import edu.wpi.first.wpilibj.controller.RamseteController
import edu.wpi.first.wpilibj.geometry.Rotation2d
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry
import org.ghrobotics.frc2020.Constants
import org.ghrobotics.frc2020.commands.TeleopDriveCommand
import org.ghrobotics.lib.mathematics.units.Meter
import org.ghrobotics.lib.mathematics.units.SIUnit
import org.ghrobotics.lib.mathematics.units.nativeunit.DefaultNativeUnitModel
import org.ghrobotics.lib.motors.ctre.FalconSRX
import org.ghrobotics.lib.physics.MotorCharacterization
import org.ghrobotics.lib.subsystems.drive.FalconWestCoastDrivetrain

/**
 * Represents the drivetrain of the robot.
 */
object Drivetrain : FalconWestCoastDrivetrain() {
    // Create motors
    override val leftMotor = FalconSRX(Constants.Drivetrain.kLeftMasterId, Constants.Drivetrain.kNativeUnitModel)
    override val rightMotor = FalconSRX(Constants.Drivetrain.kRightMasterId, Constants.Drivetrain.kNativeUnitModel)

    // Path following
    override val controller = RamseteController(2.0, 0.7)
    override val kinematics = DifferentialDriveKinematics(Constants.Drivetrain.kTrackWidth.value)
    override val odometry = DifferentialDriveOdometry(kinematics)

    // Gyro
    override val gyro = { Rotation2d() }

    // Motor characterization
    override val leftCharacterization = MotorCharacterization<Meter>(SIUnit(0.0), SIUnit(0.0), SIUnit(0.0))
    override val rightCharacterization = MotorCharacterization<Meter>(SIUnit(0.0), SIUnit(0.0), SIUnit(0.0))

    // Emergency mode
    override fun activateEmergency() {}
    override fun recoverFromEmergency() {}

    // Initialize follower motors and other motor configs
    init {
        val leftSlave1 = FalconSRX(Constants.Drivetrain.kLeftSlave1Id, DefaultNativeUnitModel)
        val rightSlave1 = FalconSRX(Constants.Drivetrain.kRightSlave1Id, DefaultNativeUnitModel)

        leftSlave1.follow(leftMotor)
        rightSlave1.follow(rightMotor)

        leftMotor.outputInverted = false
        leftSlave1.outputInverted = false
        rightMotor.outputInverted = true
        rightMotor.outputInverted = true

        // Set the default command
        defaultCommand = TeleopDriveCommand()
    }
}
