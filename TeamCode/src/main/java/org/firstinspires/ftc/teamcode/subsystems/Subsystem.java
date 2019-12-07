package org.firstinspires.ftc.teamcode.subsystems;

import android.support.annotation.Nullable;

import com.acmerobotics.dashboard.canvas.Canvas;

import java.util.Map;

/** much thanks to rbrott (https://github.com/rbrott)
 * & https://github.com/acmerobotics/relic-recovery
 */


 public abstract class Subsystem {
    /**
     * Run control code and return telemetry.
     */
    public abstract Map<String, Object> update(@Nullable Canvas fieldOverlay);
 }