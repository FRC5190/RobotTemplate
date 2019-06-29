/*
 * FRC Team 5190
 * Green Hope Falcons
 */

package org.ghrobotics.frc2020

import org.ghrobotics.lib.mathematics.units.amp
import org.ghrobotics.lib.mathematics.units.inch
import org.ghrobotics.lib.mathematics.units.nativeunits.NativeUnitLengthModel
import org.ghrobotics.lib.mathematics.units.nativeunits.nativeUnits

@Suppress("MemberVisibilityCanBePrivate", "unused")
object Constants {

    // Drivetrain
    const val kDriveLeftMasterId = 1
    const val kDriveLeftSlaveId = 2
    const val kDriveRightMasterId = 3
    const val kDriveRightSlaveId = 4

    val kDriveWheelRadius = 2.inch
    val kDriveNativeUnitModel = NativeUnitLengthModel(1440.nativeUnits, kDriveWheelRadius)

    val kDriveCurrentLimit = 38.amp

}
