/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *
 * Copyright 2019, Green Hope Falcons
 */

package org.ghrobotics.frc2020

import org.ghrobotics.lib.mathematics.units.amps
import org.ghrobotics.lib.mathematics.units.inches
import org.ghrobotics.lib.mathematics.units.milli
import org.ghrobotics.lib.mathematics.units.nativeunit.NativeUnitLengthModel
import org.ghrobotics.lib.mathematics.units.nativeunit.nativeUnits

@Suppress("MemberVisibilityCanBePrivate", "unused")
/**
 * Contains all the constants for the robot.
 */
object Constants {
    /**
     * Constants for the drivetrain.
     */
    object Drivetrain {
        const val kLeftMasterId = 1
        const val kLeftSlave1Id = 2
        const val kRightMasterId = 3
        const val kRightSlave1Id = 4

        val kWheelRadius = 3.inches
        val kTrackWidth = 27.75.inches
        val kNativeUnitModel = NativeUnitLengthModel(1440.nativeUnits, kWheelRadius)

        val kPeakCurrentLimit = 68.amps
        val kPeakCurrentLimitDuration = 250.milli.seconds
        val kContinuousCurrentLimit = 38.amps
    }
}
