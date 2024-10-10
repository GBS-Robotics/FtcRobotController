package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.matrices.MatrixF;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;

public class RotateRobot
{
    public RotateRobot (float yaw, VectorF relativePosition)
    {
        MatrixF rotation = new MatrixF(3, 3) {
            @Override
            public MatrixF emptyMatrix(int numRows, int numCols) {
                return null;
            }

            @Override
            public float get(int row, int col) {
                return 0;
            }

            @Override
            public void put(int row, int col, float value) {
                
            }
        };
    }
}
