package de.fhg.ivi.crowdsimulation.simulation.tools;

/**
 * Icecore's atan2
 *
 * http://www.java-gaming.org/index.php?topic=36467.0
 * http://www.java-gaming.org/topics/extremely-fast-atan2/36467/msg/346145/view.html#msg346145
 *
 * @author hahmann
 *
 */
public final class Icecore
{

    private static final int   Size_Ac     = 100000;

    private static final int   Size_Ar     = Size_Ac + 1;

    private static final float Pi          = (float) Math.PI;

    private static final float Pi_H        = Pi / 2;

    private static final float Atan2[]     = new float[Size_Ar];

    private static final float Atan2_PM[]  = new float[Size_Ar];

    private static final float Atan2_MP[]  = new float[Size_Ar];

    private static final float Atan2_MM[]  = new float[Size_Ar];

    private static final float Atan2_R[]   = new float[Size_Ar];

    private static final float Atan2_RPM[] = new float[Size_Ar];

    private static final float Atan2_RMP[] = new float[Size_Ar];

    private static final float Atan2_RMM[] = new float[Size_Ar];

    static
    {
        for (int i = 0; i <= Size_Ac; i++ )
        {
            double d = (double) i / Size_Ac;
            double x = 1;
            double y = x * d;
            float v = (float) Math.atan2(y, x);
            Atan2[i] = v;
            Atan2_PM[i] = Pi - v;
            Atan2_MP[i] = -v;
            Atan2_MM[i] = -Pi + v;

            Atan2_R[i] = Pi_H - v;
            Atan2_RPM[i] = Pi_H + v;
            Atan2_RMP[i] = -Pi_H + v;
            Atan2_RMM[i] = -Pi_H - v;
        }
    }

    /**
     * Returns the angle <i>theta</i> from the conversion of rectangular coordinates
     * ({@code x},&nbsp;{@code y}) to polar coordinates (r,&nbsp;<i>theta</i>). This method computes
     * the phase <i>theta</i> by computing an arc tangent of {@code y/x} in the range of -<i>pi</i>
     * to <i>pi</i>. Special cases:
     * <ul>
     * <li>If either argument is NaN, then the result is NaN.
     * <li>If the first argument is positive zero and the second argument is positive, or the first
     * argument is positive and finite and the second argument is positive infinity, then the result
     * is positive zero.
     * <li>If the first argument is negative zero and the second argument is positive, or the first
     * argument is negative and finite and the second argument is positive infinity, then the result
     * is negative zero.
     * <li>If the first argument is positive zero and the second argument is negative, or the first
     * argument is positive and finite and the second argument is negative infinity, then the result
     * is the {@code double} value closest to <i>pi</i>.
     * <li>If the first argument is negative zero and the second argument is negative, or the first
     * argument is negative and finite and the second argument is negative infinity, then the result
     * is the {@code double} value closest to -<i>pi</i>.
     * <li>If the first argument is positive and the second argument is positive zero or negative
     * zero, or the first argument is positive infinity and the second argument is finite, then the
     * result is the {@code double} value closest to <i>pi</i>/2.
     * <li>If the first argument is negative and the second argument is positive zero or negative
     * zero, or the first argument is negative infinity and the second argument is finite, then the
     * result is the {@code double} value closest to -<i>pi</i>/2.
     * <li>If both arguments are positive infinity, then the result is the {@code double} value
     * closest to <i>pi</i>/4.
     * <li>If the first argument is positive infinity and the second argument is negative infinity,
     * then the result is the {@code double} value closest to 3*<i>pi</i>/4.
     * <li>If the first argument is negative infinity and the second argument is positive infinity,
     * then the result is the {@code double} value closest to -<i>pi</i>/4.
     * <li>If both arguments are negative infinity, then the result is the {@code double} value
     * closest to -3*<i>pi</i>/4.
     * </ul>
     *
     * <p>
     * The computed result must be within 2 ulps of the exact result. Results must be
     * semi-monotonic.
     *
     * @param y the ordinate coordinate
     * @param x the abscissa coordinate
     * @return the <i>theta</i> component in radians of the point (<i>r</i>,&nbsp;<i>theta</i>) in
     *         polar coordinates that corresponds to the point (<i>x</i>,&nbsp;<i>y</i>) in
     *         Cartesian coordinates.
     */
    public static final float atan2(float y, float x)
    {
        if (y < 0)
        {
            if (x < 0)
            {
                // (y < x) because == (-y > -x)
                if (y < x)
                {
                    return Atan2_RMM[(int) (x / y * Size_Ac)];
                }
                return Atan2_MM[(int) (y / x * Size_Ac)];
            }
            y = -y;
            if (y > x)
            {
                return Atan2_RMP[(int) (x / y * Size_Ac)];
            }
            return Atan2_MP[(int) (y / x * Size_Ac)];
        }
        if (x < 0)
        {
            x = -x;
            if (y > x)
            {
                return Atan2_RPM[(int) (x / y * Size_Ac)];
            }
            return Atan2_PM[(int) (y / x * Size_Ac)];
        }
        if (y > x)
        {
            return Atan2_R[(int) (x / y * Size_Ac)];
        }
        return Atan2[(int) (y / x * Size_Ac)];
    }
}
