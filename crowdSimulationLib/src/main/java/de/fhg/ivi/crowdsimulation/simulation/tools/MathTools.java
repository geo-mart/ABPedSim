package de.fhg.ivi.crowdsimulation.simulation.tools;

import java.util.concurrent.ThreadLocalRandom;

import org.apache.commons.math3.util.FastMath;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import com.vividsolutions.jts.geom.Coordinate;
import com.vividsolutions.jts.geom.Point;

import math.geom2d.Vector2D;

/**
 * Class for providing helping methods in case of mathematical functions. There are several
 * functions implemented in this class:
 *
 * <li>Calculation of a gaussian distribution</li>
 * <li>Rounding of a {@link Double} value after a specific number of digits</li>
 * <li>Convert from m/s to km/h</li>
 * <li>Convert from km/h to m/s</li>
 * <li>Calculation of a normalized version of a {@link Vector2D}</li>
 *
 * <p>
 *
 * @author hahmann/meinert
 */
public class MathTools
{

    /**
     * Uses the object logger for printing specific messages in the console.
     */
    private static final Logger logger       = LoggerFactory.getLogger(GeometryTools.class);

    /**
     * Defines the {@code byte} or mode of the {@link #norm(Vector2D, byte)}. In this case the
     * calculation will be executed with the highest accuracy of the {@link Math} library.
     */
    private final static byte   STRICT_MATH  = 0;

    /**
     * Defines the {@code byte} or mode of the {@link #norm(Vector2D, byte)}. In this case the
     * calculation will be executed with the medium accuracy of the {@link FastMath} library
     */
    private final static byte   FAST_MATH    = 1;

    /**
     * Defines the {@code byte} or mode of the {@link #norm(Vector2D, byte)}. In this case the
     * calculation will be executed with the lower accuracy of a lookup table.
     */
    private final static byte   LUT          = 2;

    /**
     * The magnitude of the length of the {@link #sqrtLUT}. A magnitude of 2 leads to length of 100
     * values of the LUT
     */
    private static final int    LUTMagnitude = 4;

    /**
     * The length of the {@link #sqrtLUT}. The longer, the better the resolution of the square root
     * LUT. Depends on {@link #LUTMagnitude}.
     */
    private static int          LUTLength    = 0;

    /**
     * The square root Lookup table
     */
    private static double[]     sqrtLUT      = null;

    /** The constant for 2*PI, equivalent to 360 degrees. */
    public final static double  M_2PI        = Math.PI * 2;

    static
    {
        // TODO move to separate class
        precomputeSqrtLUT();
    }

    /**
     * Computes a random value depending on the given {@code mean} value and the given
     * {@code standardDeviation}, guaranteed to be in the 95% quantile of the normal distribution
     * (aka Gaussian Distribution) to avoid extreme outliers.
     *
     * @param mean the mean value of the distribution
     * @param standardDeviation the standard Deviation
     *
     * @return a value which is guaranteed to be within the 95% quantile of the normal distribution
     *         to avoid extreme outliers
     */
    public static float getRandomGaussianValue(float mean, float standardDeviation)
    {
        float randomGaussianValue = (float) ThreadLocalRandom.current().nextGaussian()
            * standardDeviation + mean;

        // avoid values greater than or lower than mean +/- 2 * standardDeviation
        float twoSigma = 1.960f * standardDeviation;
        if (randomGaussianValue > mean + twoSigma)
        {
            randomGaussianValue = mean + twoSigma;
        }
        else if (randomGaussianValue < mean - twoSigma)
        {
            randomGaussianValue = mean - twoSigma;
        }
        return randomGaussianValue;
    }

    /**
     * Method for rounding {@link Double} numbers.
     *
     * @param value which is to be rounded
     * @param digits determines the number of digits of the to be rounded {@code value}
     *
     * @return a {@link Double} number of the {@code value} which is rounded with {@code digits}
     *         numbers of digits
     */
    public static double round(double value, int digits)
    {
        return Math.round(value * Math.pow(10d, digits)) / Math.pow(10d, digits);
    }

    /**
     * Converts a {@link Double} from meter per second to kilometer per hour.
     *
     * @param inputValue describes the velocity in meter per second
     *
     * @return the {@code inputValue} converted in kilometer per hour
     */
    public static float convertMsToKmh(float inputValue)
    {
        return inputValue * 3.6f;
    }

    /**
     * Converts a {@link Double} from kilometer per hour to meter per second.
     *
     * @param inputValue describes the velocity in kilometer per hour
     *
     * @return the {@code inputValue} converted in meter per second
     */
    public static float convertKmhToMs(float inputValue)
    {
        return inputValue / 3.6f;
    }

    /**
     * Computes the 2-dimensional Euclidean distance between 2 {@link Coordinate} objects. The
     * Z-ordinate is ignored.
     *
     * @param c1 a point
     * @param c2 a point
     *
     * @return the 2-dimensional Euclidean distance between the locations
     */
    public static double distance(Coordinate c1, Coordinate c2)
    {
        double dx = c1.x - c2.x;
        double dy = c1.y - c2.y;
        return hypot(dx, dy, FAST_MATH);
    }

    /**
     * Computes the 2-dimensional Euclidean distance between 2 {@link Point} objects. The Z-ordinate
     * is ignored.
     *
     * @param c1 a point
     * @param c2 a point
     *
     * @return the 2-dimensional Euclidean distance between the locations
     */
    public static double distance(Point c1, Point c2)
    {
        double dx = c1.getX() - c2.getX();
        double dy = c1.getY() - c2.getY();
        return hypot(dx, dy, FAST_MATH);
    }

    /**
     * Computes the 2-dimensional Euclidean distance (squared) between 2 {@link Coordinate} objects.
     * The Z-ordinate is ignored.
     *
     * @param c1 a point
     * @param c2 a point
     *
     * @return the 2-dimensional Euclidean distance (squared) between the locations
     */
    public static double distanceSquared(Coordinate c1, Coordinate c2)
    {
        return (c2.x - c1.x) * (c2.x - c1.x) + (c2.y - c1.y) * (c2.y - c1.y);
    }

    /**
     * Returns the {@link Vector2D} with same direction as the input x and y values of a
     * {@link Vector2D}, but with norm equal to 1.
     *
     * @param x the x value of a {@link Vector2D}
     * @param y the y value of a {@link Vector2D}
     *
     * @return normalized {@link Vector2D}
     */
    public static Vector2D normalize(double x, double y)
    {
        double r = hypot(x, y, FAST_MATH);
        return new Vector2D((x / r), (y / r));
    }

    /**
     * Computes the norm (squared) of the given {@link Vector2D} as a double value.
     *
     * @param vector {@link Vector2D}
     *
     * @return the euclidean norm (squared) of the {@link Vector2D} as a double value
     */
    public static double normSquared(Vector2D vector)
    {
        return vector.x() * vector.x() + vector.y() * vector.y();
    }

    /**
     * Computes the norm of the given {@link Vector2D} as a double value. For this calculation
     * exists three different approaches, which differ in their accuracy, and can be invoked in this
     * method.
     *
     * @param vector {@link Vector2D}
     *
     * @return the euclidean norm of the {@link Vector2D} as a double value
     */
    public static double norm(Vector2D vector)
    {
        // return normalized(vector, STRICT_MATH);
        return norm(vector, FAST_MATH);
        // return normalized(vector, LUT);
    }

    /**
     * Computes the norm of the given {@link Vector2D} as a double value. For this calculation
     * exists three different approaches, which differ in their accuracy, and can be invoked in this
     * method. The highest accuracy method based on the {@link StrictMath} library, the medium one
     * based on the {@link FastMath} library and the lower accuracy method on a lookup table.
     *
     * @param vector {@link Vector2D}
     * @param mode defines the mode, which should be executed in this method
     *
     * @return the euclidean norm of the {@link Vector2D} as a double value
     */
    private static double norm(Vector2D vector, byte mode)
    {
        return hypot(vector.x(), vector.y(), mode);
    }

    /**
     * Returns the angle of the given {@code vector} with the horizontal axis, in radians.
     *
     * @return the horizontal angle of the vector
     */
    public static double angle(Vector2D vector)
    {
        return (Icecore.atan2((float) vector.y(), (float) vector.x()) + M_2PI) % (M_2PI);
    }

    /**
     * Tests, if the given vector is a zero vector (x==0 AND y==0)
     *
     * @param vector the vector to test
     * @return {@code true} if the given vector is a zero vector {@code false} otherwise.
     */
    public static boolean isZeroVector(Vector2D vector)
    {
        return vector.x() == 0 && vector.y() == 0;
    }

    /**
     * Returns the hypotenuse of a triangle with sides {@code x} and {@code y} -
     * sqrt(<i>x</i><sup>2</sup>&nbsp;+<i>y</i><sup>2</sup>)<br/>
     * .
     *
     * <ul>
     * <li>If either argument is infinite, then the result is positive infinity.</li>
     * <li>else, if either argument is NaN then the result is NaN.</li>
     * </ul>
     *
     * In case of 0 <= <i>x</i><sup>2</sup>&nbsp;+<i>y</i><sup>2</sup> <= 1 the result is not
     * computed but only looked up in {@link #sqrtLUT}. In all other cases
     * {@link FastMath#hypot(double, double)} is used for computation
     *
     * @param x a value
     * @param y a value
     * @return sqrt(<i>x</i><sup>2</sup>&nbsp;+<i>y</i><sup>2</sup>)
     */
    private static double hypot(double x, double y, byte mode)
    {
        double value = Double.NaN;
        switch (mode)
        {
            // highest accuracy
            case STRICT_MATH:
                value = Math.hypot(x, y);
                break;
            // medium accuracy
            case FAST_MATH:
                value = FastMath.hypot(x, y);
                break;
            // low accuracy
            case LUT:
                value = hypotLUT(x, y);
                break;

            default:
                break;
        }
        // for testing the actual range of input values to this function
        logger.trace("MathTools.normalized(), result=" + value + ", input=" + Math.pow(value, 2));
        return value;
    }

    /**
     * Precomputes the square root Lookup table
     */
    private static void precomputeSqrtLUT()
    {
        LUTLength = (int) Math.pow(10, LUTMagnitude);
        sqrtLUT = new double[LUTLength + 1];
        for (int i = 0; i < LUTLength + 1; i++ )
        {
            sqrtLUT[i] = Math.sqrt(i * Math.pow(10, -LUTMagnitude));
        }
    }

    /**
     * Returns the hypotenuse of a triangle with sides {@code x} and {@code y} -
     * sqrt(<i>x</i><sup>2</sup>&nbsp;+<i>y</i><sup>2</sup>)<br/>
     * .
     *
     * <ul>
     * <li>If either argument is infinite, then the result is positive infinity.</li>
     * <li>else, if either argument is NaN then the result is NaN.</li>
     * </ul>
     *
     * In case of 0 <= <i>x</i><sup>2</sup>&nbsp;+<i>y</i><sup>2</sup> <= 1 the result is not
     * computed but only looked up in {@link #sqrtLUT}. In all other cases
     * {@link FastMath#hypot(double, double)} is used for computation
     *
     * @param x a value
     * @param y a value
     * @return sqrt(<i>x</i><sup>2</sup>&nbsp;+<i>y</i><sup>2</sup>)
     */
    private static double hypotLUT(double x, double y)
    {
        double radicand = x * x + y * y;
        if (radicand > 1 || radicand < 0)
            return FastMath.hypot(x, y);
        double roundedRadicand = round(radicand, LUTMagnitude);
        return sqrtLUT[(int) (roundedRadicand * LUTLength)];
    }
}
