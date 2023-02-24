package org.firstinspires.ftc.teamcode.dependencies;

import com.arcrobotics.ftclib.geometry.Vector2d;

public class Waypoints {
    // center of board is at 0,0. x and y are from audience perspective, and measured in inches.

    // all of these are set to center of field right now, need testing
    public static enum position {
        STACK(new Vector2d(0,0)),
        SUBSTATION(new Vector2d(0,0)),
        CLOSEPARK(new Vector2d(0,0)),
        FARPARK(new Vector2d(0,0));
        private final Vector2d pos;
        position(Vector2d i){
            this.pos = i;
        }

    }

}
