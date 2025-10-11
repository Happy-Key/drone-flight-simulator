using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class GaussianElimination : MonoBehaviour
{
    public static void Solve(float[,] A, float[] b)
    {
        int n = b.Length;

        // Augment matrix A with vector b
        float[,] augmentedMatrix = new float[n, n + 1];
        for (int i = 0; i < n; i++)
        {
            for (int j = 0; j < n; j++)
            {
                augmentedMatrix[i, j] = A[i, j];
            }
            augmentedMatrix[i, n] = b[i];
        }

        // Forward elimination
        for (int col = 0; col < n; col++)
        {
            // Find pivot row
            int pivotRow = col;
            for (int row = col + 1; row < n; row++)
            {
                if (Mathf.Abs(augmentedMatrix[row, col]) > Mathf.Abs(augmentedMatrix[pivotRow, col]))
                {
                    pivotRow = row;
                }
            }

            // Swap rows if necessary
            if (pivotRow != col)
            {
                for (int j = col; j <= n; j++)
                {
                    float temp = augmentedMatrix[col, j];
                    augmentedMatrix[col, j] = augmentedMatrix[pivotRow, j];
                    augmentedMatrix[pivotRow, j] = temp;
                }
            }

            // Eliminate below the pivot
            for (int row = col + 1; row < n; row++)
            {
                float factor = augmentedMatrix[row, col] / augmentedMatrix[col, col];
                for (int j = col; j <= n; j++)
                {
                    augmentedMatrix[row, j] -= factor * augmentedMatrix[col, j];
                }
            }
        }

        // Back substitution
        float[] x = new float[n];
        for (int row = n - 1; row >= 0; row--)
        {
            float sum = 0;
            for (int j = row + 1; j < n; j++)
            {
                sum += augmentedMatrix[row, j] * x[j];
            }
            x[row] = (augmentedMatrix[row, n] - sum) / augmentedMatrix[row, row];
        }

        // Copy the solution back to the original vector b
        for (int i = 0; i < n; i++)
        {
            b[i] = x[i];
        }
    }
}
