package de.fhg.ivi.crowdsimulation.ui.gui.visualisation;

import java.awt.BasicStroke;
import java.awt.Color;
import java.awt.Component;
import java.awt.Font;
import java.awt.Graphics;
import java.awt.Graphics2D;
import java.awt.Panel;
import java.awt.RenderingHints;
import java.awt.Shape;
import java.awt.Stroke;
import java.awt.event.ComponentEvent;
import java.awt.event.ComponentListener;
import java.awt.event.FocusEvent;
import java.awt.event.FocusListener;
import java.awt.event.KeyEvent;
import java.awt.event.KeyListener;
import java.awt.event.MouseEvent;
import java.awt.event.MouseWheelEvent;
import java.awt.event.MouseWheelListener;
import java.awt.geom.Line2D;
import java.awt.geom.Rectangle2D;
import java.awt.image.BufferedImage;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.concurrent.TimeUnit;

import javax.swing.JPanel;

import org.geotools.brewer.color.ColorBrewer;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import com.vividsolutions.jts.awt.ShapeWriter;
import com.vividsolutions.jts.geom.Coordinate;
import com.vividsolutions.jts.geom.Envelope;
import com.vividsolutions.jts.geom.Geometry;

import de.fhg.ivi.crowdsimulation.simulation.CrowdSimulator;
import de.fhg.ivi.crowdsimulation.simulation.mentalmodel.FollowWayPointsMentalModel;
import de.fhg.ivi.crowdsimulation.simulation.mentalmodel.WayFindingModel;
import de.fhg.ivi.crowdsimulation.simulation.objects.Boundary;
import de.fhg.ivi.crowdsimulation.simulation.objects.Crowd;
import de.fhg.ivi.crowdsimulation.simulation.objects.Grid;
import de.fhg.ivi.crowdsimulation.simulation.objects.Pedestrian;
import de.fhg.ivi.crowdsimulation.simulation.objects.WayPoint;
import de.fhg.ivi.crowdsimulation.simulation.tools.MathTools;
import math.geom2d.Vector2D;

/**
 * This class is a {@link JPanel}, which represents the actual "Map", i.e. where {@link Boundary},
 * {@link WayPoint}, {@link Crowd}, {@link Pedestrian} and {@link Grid} objects are painted and
 * which thereby visualizes the progress of the simulation.
 *
 * @author hahmann/meinert
 */
public class MapPanel extends JPanel
    implements Runnable, ComponentListener, FocusListener, MouseWheelListener, KeyListener
{
    /**
     * Uses the object logger for printing specific messages in the console.
     */
    private static final Logger                       logger           = LoggerFactory
        .getLogger(MapPanel.class);

    /**
     * default serial version ID
     */
    private static final long                         serialVersionUID = 1L;

    /**
     * The {@link CrowdSimulator} object that is visualized in this {@link MapPanel}.
     */
    private final CrowdSimulator                      crowdSimulator;

    /**
     * A {@link MapPanel#currentScale} of 1 means that 1 pixel equals 1 meter in reality. A scale of
     * 0.5 mean that 1 pixel equals 2 meters in reality. Default is 1.
     */
    private double                                    currentScale     = 0;

    /**
     * A rectangle, which defines the bounding box of the CrowdSimulator.
     */
    private Envelope                                  currentMapExtent;

    /**
     * A coordinate, which defines the map origin (i.e. upper left corner of the map).
     */
    private Coordinate                                currentMapOrigin;

    /**
     * Translator for GeoTools objects to java.awt objects
     */
    private ShapeWriter                               shapeWriter;

    /**
     * Variable to store if {@code wayPoints} are visible (true) or not (false).
     */
    private boolean                                   wayPointsVisible;

    /**
     * Variable to store if {@code wayPointVerticals} are visible (true) or not (false).
     */
    private boolean                                   wayPointVerticalsVisible;

    /**
     * Variable to store if {@code wayPointAxis} are visible (true) or not (false).
     */
    private boolean                                   wayPointAxisVisible;

    /**
     * Variable to store if {@code wayPointLabels} are visible (true) or not (false).
     */
    private boolean                                   wayPointLabelsVisible;

    /**
     * Variable to store if {@code boundaries} are visible (true) or not (false).
     */
    private boolean                                   boundariesVisible;

    /**
     * Variable to store if {@code pedestrians} are visible (true) or not (false).
     */
    private boolean                                   pedestriansVisible;

    /**
     * Variable to store the offset, which is used for map navigation.
     */
    private double                                    offset           = 50;

    /**
     * The minimum interval between two consecutive refreshs in nanoseconds
     */
    private int                                       refreshInterval  = 20_000_000;

    /**
     * Color Map containing 8 colors from {@link ColorBrewer} "RdYlGn" color scale and thresholds
     * assigning different colors to different classes.
     *
     * @see <a href=
     *      "http://colorbrewer2.org/#type=diverging&scheme=RdYlGn&n=8">http://colorbrewer2.org/#type=diverging&scheme=RdYlGn&n=8</a>
     */
    private static final LinkedHashMap<Double, Color> pedestrianColorMap;
    static
    {
        pedestrianColorMap = new LinkedHashMap<>();
        pedestrianColorMap.put(0.001d, ColorBrewer.instance().getPalette("RdYlGn").getColors(8)[7]);
        pedestrianColorMap.put(0.01d, ColorBrewer.instance().getPalette("RdYlGn").getColors(8)[6]);
        pedestrianColorMap.put(0.02d, ColorBrewer.instance().getPalette("RdYlGn").getColors(8)[5]);
        pedestrianColorMap.put(0.05d, ColorBrewer.instance().getPalette("RdYlGn").getColors(8)[4]);
        pedestrianColorMap.put(0.1d, ColorBrewer.instance().getPalette("RdYlGn").getColors(8)[3]);
        pedestrianColorMap.put(0.2d, ColorBrewer.instance().getPalette("RdYlGn").getColors(8)[2]);
        pedestrianColorMap.put(0.5d, ColorBrewer.instance().getPalette("RdYlGn").getColors(8)[1]);
        pedestrianColorMap.put(1d, ColorBrewer.instance().getPalette("RdYlGn").getColors(8)[0]);
    }

    /**
     * Color Map containing 8 colors from {@link ColorBrewer} "RdYlGn" color scale and thresholds
     * assigning different colors to different classes.
     *
     * @see <a href=
     *      "http://colorbrewer2.org/#type=diverging&scheme=RdYlGn&n=8">http://colorbrewer2.org/#type=diverging&scheme=RdYlGn&n=8</a>
     */
    private static final LinkedHashMap<Double, Color> gridCellColorMap;
    static
    {
        gridCellColorMap = new LinkedHashMap<>();
        gridCellColorMap.put(1d, ColorBrewer.instance().getPalette("RdYlGn").getColors(8)[0]);
        gridCellColorMap.put(0.5d, ColorBrewer.instance().getPalette("RdYlGn").getColors(8)[1]);
        gridCellColorMap.put(0.2d, ColorBrewer.instance().getPalette("RdYlGn").getColors(8)[2]);
        gridCellColorMap.put(0.1d, ColorBrewer.instance().getPalette("RdYlGn").getColors(8)[3]);
        gridCellColorMap.put(0.05d, ColorBrewer.instance().getPalette("RdYlGn").getColors(8)[4]);
        gridCellColorMap.put(0.02d, ColorBrewer.instance().getPalette("RdYlGn").getColors(8)[5]);
        gridCellColorMap.put(0.01d, ColorBrewer.instance().getPalette("RdYlGn").getColors(8)[6]);
        gridCellColorMap.put(0.005d, ColorBrewer.instance().getPalette("RdYlGn").getColors(8)[7]);
    }

    /**
     * Can be one of {@link MapPanel#PEDESTRIAN_FILL_MODE_DEFAULT},
     * {@link MapPanel#PEDESTRIAN_FILL_MODE_EXTRINSIC_FORCES_QUANTITATIVE},
     * {@link MapPanel#PEDESTRIAN_FILL_MODE_FORCES_AND_ORIENTATION}
     *
     **/
    private int             pedestriansFillMode                                = PEDESTRIAN_FILL_MODE_EXTRINSIC_FORCES_QUANTITATIVE;

    /**
     * Fills all pedestrians with green color
     */
    public static final int PEDESTRIAN_FILL_MODE_DEFAULT                       = 0;

    /**
     * Fills the upper half of the Pedestrian with blue color, if the Pedestrian needs orientation
     * according to {@link WayFindingModel#needsOrientation()} and the lower half with red color if
     * the Pedestrian has extrinsic forces (boundary or other pedestrian) according to
     * {@link WayFindingModel#hasCourseDeviation()}. If the respective condition is not {@code true}
     * then the respective part is filled with green color.
     */
    public static final int PEDESTRIAN_FILL_MODE_FORCES_AND_ORIENTATION        = 1;

    /**
     * Fills the pedestrian with a color the relates to the total amount of extrinsic forces. Green
     * means no extrinsic forces, red means high extrinsic forces.
     */
    public static final int PEDESTRIAN_FILL_MODE_EXTRINSIC_FORCES_QUANTITATIVE = 2;

    /**
     * Variable to store if the {@code currentVelocity#Pedestrian} are visible (true) or not
     * (false).
     */
    private boolean         velocityVisible;

    /**
     * Indicates the visibility of the current velocity vectors of all {@link Pedestrian}
     */
    private boolean         currentVelocityVectorVisible;

    /**
     * Indicates the visibility of the current target vector (i.e. vector normalize to length of 1
     * to target point) of all {@link Pedestrian}
     */
    private boolean         currentTargetVectorVisible;

    /**
     * Indicates the visibility of the current target points of all {@link Pedestrian}
     */
    private boolean         currentTargetPointVisible;

    /**
     * Indicates the visibility of the vector of current total extrinsic forces (i.e. forces caused
     * by other {@link Pedestrian} and Boundaries) of all {@link Pedestrian}
     */
    private boolean         currentExtrinsicForcesVisible;

    /**
     * Indicates whether force vector that results from the interaction of this {@link Pedestrian}
     * with all surrounding {@link Pedestrian} objects is visible
     */
    private boolean         currentPedestrianForceVisible;

    /**
     * Indicates whether force vector that results from the interaction of this {@link Pedestrian}
     * with all surrounding {@link Boundary} objects is visible
     */
    private boolean         currentBoundaryForceVisible;

    /**
     * Indicates if the {@link CrowdSimulator#getCrowds()} outlines are visible (true) or not
     * (false).
     */
    private boolean         crowdOutlineVisible;

    /**
     * Indicates if the {@link CrowdSimulator#getGrid()} cells are visible (true) or not (false).
     */
    private boolean         gridVisible;

    /**
     * Indicates if the labels of {@link Grid} (i.e. local crowd density values in pedestrians/m²)
     * visible or not
     */
    private boolean         gridLabelsVisible;

    /**
     * Checks out if threads are running or not.
     */
    private boolean         graphicsThreadRunning;

    /**
     * {@code true}, if rendering is currently in progress, {@code false} otherwise
     */
    private boolean         isRenderingInProgress;

    /**
     * Storage variable for the last time than the graphic was refreshed. Given in milliseconds.
     */
    private long            lastGraphicsRefreshTime                            = 0;

    /**
     * Stores the number of updates per second of the graphic frame
     */
    private double          graphicRefreshsPerSecond                           = 0;

    /**
     * The {@link Graphics} object provides diverse methods for drawing and filling. This is
     * necessary for all things which should be visualized in the application window.
     */
    private Graphics2D      backbufferGraphics;

    /**
     * An object, with the expansion of {@code widthMap} and {@code heightMap}, for drawing elements
     * in the application window.
     */
    private BufferedImage   backbufferImage;

    /**
     * Constructor.
     * <p>
     * Adds {@link Component} elements to this class for map navigation purposes. Also initialize
     * objects which should be drawn at the start of the simulation.
     *
     * @param crowdSimulator the {@link CrowdSimulator} object that is visualized in this
     *            {@link MapPanel}
     */
    public MapPanel(CrowdSimulator crowdSimulator)
    {
        super();
        this.crowdSimulator = crowdSimulator;
        shapeWriter = new ShapeWriter();
        addComponentListener(this);

        setBackground(Color.WHITE);
        addMouseWheelListener(this);
        addKeyListener(this);
        addFocusListener(this);

        // init objects which should be drawn at the start of the simulation
        setBoundariesVisible(true);
        setWayPointsVisible(true);
        setWayPointVerticalsVisible(true);
        setWayPointLabelsVisible(true);
        setPedestriansVisible(true);
        setCrowdOutlineVisible(true);
    }

    /**
     * Creates the {@code backbufferImage} as {@link BufferedImage} in which all objects of this
     * class can be drawn.
     */
    private void createBackBuffer()
    {
        try
        {
            backbufferImage = new BufferedImage(getWidth(), getHeight(),
                BufferedImage.TYPE_INT_RGB);
            backbufferGraphics = backbufferImage.createGraphics();
            backbufferGraphics.setBackground(Color.white);
            backbufferGraphics.clearRect(0, 0, getWidth(), getHeight());
        }
        catch (IllegalArgumentException e)
        {
            // do nothing if getHeight <= 0 or getWidth <= 0
            logger.debug("MapPanel.createBackBuffer(), ", e);
        }
    }

    /**
     * Sets {@link WayPoint} objects are visible or not.
     *
     * @param wayPointsVisible indicates if {@link WayPoint} are visible (true) or not (false)
     */
    public void setWayPointsVisible(boolean wayPointsVisible)
    {
        this.wayPointsVisible = wayPointsVisible;
    }

    /**
     * Tests if {@link WayPoint} objects are visible or not.
     *
     * @return {@code true} if {@link WayPoint} are visible, {@code false} otherwise
     */
    public boolean isWayPointsVisible()
    {
        return wayPointsVisible;
    }

    /**
     * Sets {@link WayPoint#getWayPointVertical()} visible or not.
     *
     * @param wayPointVerticalsVisible indicates if {@link WayPoint#getWayPointVertical()} are
     *            visible (true) or not (false)
     */
    public void setWayPointVerticalsVisible(boolean wayPointVerticalsVisible)
    {
        this.wayPointVerticalsVisible = wayPointVerticalsVisible;
    }

    /**
     * Tests if {@link WayPoint#getWayPointVertical()} are visible or not.
     *
     * @return {@code true} if {@link WayPoint#getWayPointVertical()} are visible, {@code false}
     *         otherwise
     */
    public boolean isWayPointVerticalsVisible()
    {
        return wayPointVerticalsVisible;
    }

    /**
     * Sets {@link WayPoint#getAxisBetweenWayPoints()} visible or not
     *
     * @param wayPointAxisVisible indicates {@link WayPoint#getAxisBetweenWayPoints()} are visible
     *            or not
     */
    public void setWayPointAxisVisible(boolean wayPointAxisVisible)
    {
        this.wayPointAxisVisible = wayPointAxisVisible;
    }

    /**
     * Tests if {@link WayPoint#getAxisBetweenWayPoints()} are visible or not.
     *
     * @return {@code true} if {@link WayPoint#getAxisBetweenWayPoints()} are visible, {@code false}
     *         otherwise
     */
    public boolean isWayPointAxisVisible()
    {
        return wayPointAxisVisible;
    }

    /**
     * Sets {@link WayPoint} labels (i.e. WayPoint id) visible or not
     *
     * @param wayPointLabelsVisible indicates if labels (i.e. WayPoint id) are visible or not
     */
    public void setWayPointLabelsVisible(boolean wayPointLabelsVisible)
    {
        this.wayPointLabelsVisible = wayPointLabelsVisible;
    }

    /**
     * Tests if {@link WayPoint} labels are (i.e. WayPoint id) visible or not
     *
     * @return {@code true} if {@link WayPoint} labels are (i.e. WayPoint id) are visible,
     *         {@code false} otherwise
     */
    public boolean isWayPointLabelsVisible()
    {
        return wayPointLabelsVisible;
    }

    /**
     * Sets {@link Boundary} objects visible or not.
     *
     * @param boundariesVisible indicates if {@link Boundary} objects are visible ({@code true} or
     *            not {@code false}
     */
    public void setBoundariesVisible(boolean boundariesVisible)
    {
        this.boundariesVisible = boundariesVisible;
    }

    /**
     * Tests if {@link Boundary} objects are visible or not
     *
     * @return {@code true} if {@link Boundary} objects are visible, {@code false} otherwise
     */
    public boolean isBoundariesVisible()
    {
        return boundariesVisible;
    }

    /**
     * Sets all {@link Pedestrian}s visible or not.
     *
     * @param visible indicates if {@link Pedestrian} will be visible (true) or not (false) after
     *            calling this method
     */
    public void setPedestriansVisible(boolean visible)
    {
        this.pedestriansVisible = visible;
    }

    /**
     * Tests if all {@link Pedestrian}s are visible or not.
     *
     * @return indicates if all {@link Pedestrian} are visible {@code true} or not {@code false}
     */
    public boolean isPedestriansVisible()
    {
        return pedestriansVisible;
    }

    /**
     * Sets the fill mode of the {@link Pedestrian} objects. Can be one of
     * {@link MapPanel#PEDESTRIAN_FILL_MODE_DEFAULT},
     * {@link MapPanel#PEDESTRIAN_FILL_MODE_FORCES_AND_ORIENTATION},
     * {@link MapPanel#PEDESTRIAN_FILL_MODE_EXTRINSIC_FORCES_QUANTITATIVE}
     *
     * @param pedestriansFillMode can be one of {@link MapPanel#PEDESTRIAN_FILL_MODE_DEFAULT},
     *            {@link MapPanel#PEDESTRIAN_FILL_MODE_FORCES_AND_ORIENTATION},
     *            {@link MapPanel#PEDESTRIAN_FILL_MODE_EXTRINSIC_FORCES_QUANTITATIVE}
     */
    public void setPedestriansFillMode(int pedestriansFillMode)
    {
        this.pedestriansFillMode = pedestriansFillMode;
    }

    /**
     * Gets the fill mode of the Pedestrians.
     *
     * @return the fill mode. can be one of {@link MapPanel#PEDESTRIAN_FILL_MODE_DEFAULT},
     *         {@link MapPanel#PEDESTRIAN_FILL_MODE_FORCES_AND_ORIENTATION},
     *         {@link MapPanel#PEDESTRIAN_FILL_MODE_EXTRINSIC_FORCES_QUANTITATIVE}
     */
    public int getPedestriansFillMode()
    {
        return this.pedestriansFillMode;
    }

    /**
     * Sets the velocity labels of the {@link Pedestrian} visible or not
     *
     * @param velocityVisible indicates if {@link Pedestrian} velocities will be visible (true) or
     *            not (false) after calling this method
     */
    public void setVelocityVisible(boolean velocityVisible)
    {
        this.velocityVisible = velocityVisible;
    }

    /**
     * Tests if the velocity labels of all {@link Pedestrian} are visible or not
     *
     * @return indicates if all {@link Pedestrian} velocity labels are visible {@code true} or not
     *         {@code false}
     */
    public boolean isVelocityVisible()
    {
        return velocityVisible;
    }

    /**
     * Sets the current velocity vectors of all {@link Pedestrian} visible or not.
     *
     * @param visible indicates if the current velocity vectors of all {@link Pedestrian} are
     *            visible (true) or not (false).
     */
    public void setCurrentVelocityVectorVisible(boolean visible)
    {
        this.currentVelocityVectorVisible = visible;
    }

    /**
     * Tests if the current velocity vectors of all {@link Pedestrian} are visible or not.
     *
     * @return indicates if the current velocity vectors of all {@link Pedestrian} are visible
     *         {@code true} or not {@code false}
     */
    public boolean isCurrentVelocityVectorVisible()
    {
        return currentVelocityVectorVisible;
    }

    /**
     * Sets the current target vector (i.e. vector normalized to length of 1 to target point) of all
     * {@link Pedestrian} visible or not.
     *
     * @param visible indicates if the current target points of all {@link Pedestrian} are visible
     *            (true) or not (false).
     */
    public void setCurrentTargetVectorVisible(boolean visible)
    {
        this.currentTargetVectorVisible = visible;
    }

    /**
     * Tests, if the current target vector (i.e. vector normalized to length of 1 to target point)
     * of all {@link Pedestrian} visible or not.
     *
     * @return indicates if the current target vectors of all {@link Pedestrian} are visible (true)
     *         or not (false).
     */
    public boolean isCurrentTargetVectorVisible()
    {
        return currentTargetVectorVisible;
    }

    /**
     * Sets the current target points of all {@link Pedestrian} visible or not.
     *
     * @param visible indicates if the current target points of all {@link Pedestrian} are visible
     *            (true) or not (false).
     */
    public void setCurrentTargetPointVisible(boolean visible)
    {
        this.currentTargetPointVisible = visible;
    }

    /**
     * Tests if the current target points of all {@link Pedestrian} are visible or not
     *
     * @return indicates if the current target points of all {@link Pedestrian} are visible
     *         {@code true} or not {@code false}
     */
    public boolean isCurrentTargetPointVisible()
    {
        return currentTargetPointVisible;
    }

    /**
     * Sets the current the vector of current total extrinsic forces (i.e. forces caused by other
     * {@link Pedestrian} and Boundaries) of all {@link Pedestrian} visible or not.
     *
     * @param visible indicates if the current target points of all {@link Pedestrian} are visible
     *            (true) or not (false).
     */
    public void setCurrentExtrinsicForcesVisible(boolean visible)
    {
        this.currentExtrinsicForcesVisible = visible;
    }

    /**
     * Tests if the current the vector of current total extrinsic forces (i.e. forces caused by
     * other {@link Pedestrian} and Boundaries) of all {@link Pedestrian} visible or not.
     *
     * @return indicates if the current target points of all {@link Pedestrian} are visible (true)
     *         or not (false).
     */
    public boolean isCurrentExtrinsicForcesVisible()
    {
        return currentExtrinsicForcesVisible;
    }

    /**
     * Sets the force vectors that results from the interaction a {@link Pedestrian} with all
     * surrounding {@link Pedestrian} objects visible
     *
     * @param currentPedestrianForceVisible {@code true} for visible, {@code false} for not visible
     */
    public void setCurrentPedestrianForceVisible(boolean currentPedestrianForceVisible)
    {
        this.currentPedestrianForceVisible = currentPedestrianForceVisible;
    }

    /**
     * Tests, if the force vectors that results from the interaction a {@link Pedestrian} with all
     * surrounding {@link Pedestrian} objects is visible
     *
     * @return {@code true} for visible, {@code false} for not visible
     */
    public boolean isCurrentPedestrianForceVisible()
    {
        return currentPedestrianForceVisible;
    }

    /**
     * Sets the force vector that results from the interaction of this {@link Pedestrian} with all
     * surrounding {@link Boundary} objects is visible
     *
     * @param currentBoundaryForceVisible {@code true} for visible, {@code false} for not visible
     */
    public void setCurrentBoundaryForceVisible(boolean currentBoundaryForceVisible)
    {
        this.currentBoundaryForceVisible = currentBoundaryForceVisible;
    }

    /**
     * Tests if the force vector that results from the interaction of this {@link Pedestrian} with
     * all surrounding {@link Boundary} objects is visible
     *
     * @return {@code true} for visible, {@code false} for not visible
     */
    public boolean isCurrentBoundaryForceVisible()
    {
        return currentBoundaryForceVisible;
    }

    /**
     * Sets {@link Crowd} visible or not.
     *
     * @param visible indicates if {@link Crowd} will be visible (true) or not (false) after calling
     *            this method
     */
    public void setCrowdOutlineVisible(boolean visible)
    {
        this.crowdOutlineVisible = visible;
        for (Crowd crowd : crowdSimulator.getCrowds())
        {
            crowd.setUpdatingCrowdOutline(this.crowdOutlineVisible);
        }
    }

    /**
     * Tests if {@link Crowd} is visible or not.
     *
     * @return indicates if {@link Crowd} is visible (true) or not (false)
     */
    public boolean isCrowdOutlineVisible()
    {
        return crowdOutlineVisible;
    }

    /**
     * Sets the {@link Grid} visible or not.
     *
     * @param visible indicates if {@link Grid} will be visible (true) or not (false) after calling
     *            this method
     */
    public void setGridVisible(boolean visible)
    {
        this.gridVisible = visible;
        Grid grid = crowdSimulator.getGrid();
        grid.setUpdating(this.gridVisible || this.gridLabelsVisible);
    }

    /**
     * Tests if the {@link Grid} is visible or not.
     *
     * @return indicates if {@link Grid} is visible (true) or not (false)
     */
    public boolean isGridVisible()
    {
        return gridVisible;
    }

    /**
     * Sets the labels of {@link Grid} (i.e. local crowd density values in pedestrians/m²) visible
     * or not.
     *
     * @param visible the labels of {@link Grid} (i.e. local crowd density values in pedestrians/m²)
     *            will be visible (true) or not (false) after calling this method
     */
    public void setGridLabelsVisible(boolean visible)
    {
        this.gridLabelsVisible = visible;
        Grid grid = crowdSimulator.getGrid();
        grid.setUpdating(this.gridVisible || this.gridLabelsVisible);
    }

    /**
     * Tests if the labels of {@link Grid} (i.e. local crowd density values in pedestrians/m²) are
     * visible or not.
     *
     * @return indicates if the labels of {@link Grid} (i.e. local crowd density values in
     *         pedestrians/m²) are visible (true) or not (false)
     */
    public boolean isGridLabelsVisible()
    {
        return gridLabelsVisible;
    }

    /**
     * Checks if the graphicThread is currently running.
     *
     * @return true or false in case the {@code graphicsThreadRunning} is running or not
     */
    public boolean isGraphicsThreadRunning()
    {
        return graphicsThreadRunning;
    }

    /**
     * Sets the {@code graphicsThreadRunning} as running (true) or not (false).
     *
     * @param graphicsThreadRunning could be true or false
     */
    public void setGraphicsThreadRunning(boolean graphicsThreadRunning)
    {
        this.graphicsThreadRunning = graphicsThreadRunning;
    }

    /**
     * Checks, if rendering is currently in progress.
     *
     * @return {@code true}, if rendering is currently in progress, {@code false} otherwise
     */
    public boolean isRenderingInProgress()
    {
        return isRenderingInProgress;
    }

    /**
     * Gets the current rate of graphic refresh per seconds.
     *
     * @return the current rate of graphic refresh per second
     */
    public double getGraphicRefreshsPerSecond()
    {
        return graphicRefreshsPerSecond;
    }

    /**
     * Paints the {@code backbufferImage} in the {@link MapPanel} {@link JPanel}.
     *
     * @see javax.swing.JComponent#paintComponent(java.awt.Graphics)
     */
    @Override
    protected void paintComponent(Graphics g)
    {
        super.paintComponent(g);
        paintBackBuffer();
        g.drawImage(backbufferImage, 0, 0, this);
    }

    /**
     * Draws, scales and transforms the application window in dependence to the {@link Envelope}
     * object of the loaded geodata. Farther initiation of all objects, which should be drawn in the
     * application, is executed.
     *
     */
    private void paintBackBuffer()
    {
        Graphics2D g2 = backbufferGraphics;
        // skip, if the graphics context is unknown
        if (g2 == null)
            return;
        // skip, if rendering is currently in progress to avoid parallel/double rendering
        if (isRenderingInProgress)
            return;
        // try to set the currentMapExtent from the bounding box of the crowd simulator, if unknown
        if (currentMapExtent == null)
        {
            try
            {
                currentMapExtent = crowdSimulator.getBoundingBox();
                currentMapOrigin = new Coordinate(
                    crowdSimulator.getBoundingBox().getMinX()
                        - (crowdSimulator.getBoundingBox().getMaxX()
                            - crowdSimulator.getBoundingBox().getMinX()) / 2,
                    crowdSimulator.getBoundingBox().getMinY());
            }
            catch (NullPointerException e)
            {
                logger.debug("MapPanel.paintBackBuffer(), bounding box=null", e);
            }
        }
        // skip, if the current map extent is unknown
        if (currentMapExtent == null)
            return;
        // skip, if the current map origin is unknown
        if (currentMapOrigin == null)
            return;
        // try to set currentScale from currentMapExtent (i.e. all geometries will be visible)
        if (currentScale == 0)
        {
            try
            {
                double scaleX = getSize().getWidth() / crowdSimulator.getBoundingBox().getWidth();
                double scaleY = getSize().getHeight() / crowdSimulator.getBoundingBox().getHeight();
                currentScale = scaleX < scaleY ? scaleX : scaleY;
            }
            catch (NullPointerException e)
            {
                logger.debug("MapPanel.paintBackBuffer(), bounding box=null", e);
            }
        }
        // skip, if the current scale is unknown
        if (currentScale == 0)
            return;
        // indicate that rendering is currently in progress
        isRenderingInProgress = true;

        // clear map
        g2.clearRect(0, 0, getWidth(), getHeight());

        logger.trace("currentMapExtent.getMinX()=" + currentMapExtent.getMinX()
            + ", currentMapExtent.getMinY()=" + currentMapExtent.getMinY());
        logger.trace("mapOrigin.x               =" + currentMapOrigin.x
            + ",                mapOrigin.y=" + currentMapOrigin.y);

        double translateX = currentMapOrigin.x;
        double translateY = currentMapOrigin.y;
        double scale = currentScale;

        // remember original transform
        // AffineTransform at = g2.getTransform();

        // set scale and offset
        g2.scale(scale, scale);
        g2.translate( -translateX, -translateY);

        // paints a grid that visualizes local crowd densities
        paintGrid(g2);

        // draws boundaries and waypoints
        paintBackground(g2);

        // paint crowdOutline
        paintCrowd(g2);

        // draw pedestrians
        paintPedestrians(g2);

        // reset scale and translate
        // g2.setTransform(at);
        g2.translate(translateX, translateY);
        g2.scale(1 / scale, 1 / scale);

        isRenderingInProgress = false;
    }

    /**
     * Resets the {@link #currentMapExtent}, {@link #currentMapOrigin} and {@link #currentScale}.
     * These values will be set again during the next call of {@link #paintBackBuffer()}
     */
    public void resetMapExtent()
    {
        currentMapExtent = null;
        currentMapOrigin = null;
        currentScale = 0;
    }

    /**
     * Paints {@link CrowdSimulator#getBoundaries()}, {@link CrowdSimulator#getWayPoints()} as
     * {@link Shape} objects into this {@link Panel}
     *
     * @param g2 the {@link Graphics2D} graphics context
     */
    private void paintBackground(Graphics2D g2)
    {
        // get defaults
        Stroke defaultStroke = g2.getStroke();
        Color defaultColor = g2.getColor();

        // draw boundaries in GUI
        if (boundariesVisible)
        {
            List<Boundary> boundaries = crowdSimulator.getBoundaries();
            for (Boundary boundary : boundaries)
            {
                paintBoundary(g2, boundary);
            }
        }

        // // draw waypoints in GUI
        // for (WayPoint wayPoint : crowdSimulator.getWayPoints())
        // {
        // paintWayPoint(g2, wayPoint);
        // }

        // reset defaults
        g2.setStroke(defaultStroke);
        g2.setColor(defaultColor);
    }

    /**
     * Paints a single {@link Boundary} object into the given {@link Graphics2D} context.
     *
     * @param g2 the graphics context
     * @param boundary the boundary object to be painted
     */
    private void paintBoundary(Graphics2D g2, Boundary boundary)
    {
        g2.setStroke(new BasicStroke(0.2f));
        g2.setColor(Color.BLACK);
        g2.draw(shapeWriter.toShape(boundary.getGeometry()));
    }

    /**
     * Method for drawing a {@link WayPoint} object on the {@link Graphics2D} context g2
     *
     * @param g2 denotes the screen of the application in which drawing is possible
     */
    // private void paintWayPoint(Graphics2D g2, WayPoint wayPoint)

    /**
     * Draws the outline(s) of the {@link Crowd} object into the {@link Graphics2D} context g2.
     *
     * @param g2 parameter which contains the {@link Graphics2D} element
     */
    private void paintCrowd(Graphics2D g2)
    {
        if ( !crowdOutlineVisible)
            return;
        for (Crowd crowd : crowdSimulator.getCrowds())
        {
            List<Geometry> tempCrowdOutlines = crowd.getCrowdOutlines();
            if (tempCrowdOutlines != null)
            {
                // get defaults
                Color defaultColor = g2.getColor();
                Stroke defaultStroke = g2.getStroke();

                g2.setColor(Color.BLUE);
                g2.setStroke(new BasicStroke(0.5f));

                for (Geometry crowdOutline : tempCrowdOutlines)
                {
                    try
                    {
                        g2.draw(shapeWriter.toShape(crowdOutline));
                    }
                    catch (Exception e)
                    {
                        logger.error("Crowd.paint(), crowdOutline=" + crowdOutline);
                    }
                }

                // reset defaults
                g2.setColor(defaultColor);
                g2.setStroke(defaultStroke);
            }
            else
                logger.error("Crowd.paint(), " + "tempCrowdOutlines=null");
        }
    }

    /**
     * Draws {@code pedestrians} into the application window.
     *
     * @param g2 parameter which contains the {@link Graphics2D} element
     */
    private void paintPedestrians(Graphics2D g2)
    {
        // get defaults
        RenderingHints defaultRenderingHints = g2.getRenderingHints();
        Stroke defaultStroke = g2.getStroke();
        Color defaultColor = g2.getColor();
        Font defaultFont = g2.getFont();

        // set pedestrian specific parameter
        // /* Enable anti-aliasing and pure stroke */
        g2.setRenderingHint(RenderingHints.KEY_ANTIALIASING, RenderingHints.VALUE_ANTIALIAS_ON);
        g2.setRenderingHint(RenderingHints.KEY_STROKE_CONTROL, RenderingHints.VALUE_STROKE_PURE);
        float strokeWidth = 0.1f;
        g2.setStroke(new BasicStroke(strokeWidth));
        g2.setColor(Color.BLUE);

        // paint pedestrians
        for (Crowd crowd : crowdSimulator.getCrowds())
        {
            if (pedestriansVisible)
            {
                for (Pedestrian pedestrian : crowd.getPedestrians())
                {
                    paintPedestrian(g2, pedestrian,
                        crowdSimulator.getForceModel().getPedestrianRadius());
                }
            }
        }

        // reset defaults
        g2.setRenderingHints(defaultRenderingHints);
        g2.setStroke(defaultStroke);
        g2.setColor(defaultColor);
        g2.setFont(defaultFont);
    }

    /**
     * Gets a color that matches the class given in {@link #pedestrianColorMap} for the given
     * {@code value}.
     *
     * @param value the value to be used for finding a color in {@link #pedestrianColorMap}
     *
     * @return the appropriate color for the given {@code value}
     */
    private Color getPedestrianColor(double value)
    {
        // if value equals zero the return green
        if (value == 0d)
        {
            return pedestrianColorMap.entrySet().iterator().next().getValue();
        }
        // default value = green
        Color color = pedestrianColorMap.entrySet().iterator().next().getValue();

        // look for an appropriate class value depending on the value
        for (Map.Entry<Double, Color> entry : pedestrianColorMap.entrySet())
        {
            if (value > entry.getKey())
            {
                color = entry.getValue();
            }
        }
        return color;
    }

    /**
     * Paints the given {@link Pedestrian} {@code pedestrian} on the given {@link Graphics2D}
     * context {@code g2} depending on some visualization settings given by the further parameters.
     *
     * @param g2 denotes the screen of the application in which drawing is possible
     * @param pedestrian the {@link Pedestrian} object to be drawn.
     * @param radius the radius to be used for drawing
     */
    private void paintPedestrian(Graphics2D g2, Pedestrian pedestrian, double radius)
    {
        double x = pedestrian.getCurrentPosition().x();
        double y = pedestrian.getCurrentPosition().y();

        double fillRadius = radius;

        // fills pedestrians according to selected fill mode
        if (pedestriansFillMode == PEDESTRIAN_FILL_MODE_DEFAULT
            || pedestriansFillMode == PEDESTRIAN_FILL_MODE_FORCES_AND_ORIENTATION)
        {
            g2.setColor(Color.GREEN);
            Rectangle2D.Double square = new Rectangle2D.Double(x - fillRadius, y - fillRadius,
                2 * fillRadius, 2 * fillRadius);
            g2.draw(square);
        }
        if (pedestriansFillMode == PEDESTRIAN_FILL_MODE_FORCES_AND_ORIENTATION)
        {
            // indicates need for orientation
            if (pedestrian.getMentalModel().needsOrientation())
            {
                g2.setColor(Color.BLUE);
                Rectangle2D.Double square = new Rectangle2D.Double(x - fillRadius, y - fillRadius,
                    2 * fillRadius, 1 * fillRadius);
                g2.fill(square);
            }
            // indicates course deviation
            if (pedestrian.getMentalModel().hasCourseDeviation())
            {
                g2.setColor(Color.RED);
                Rectangle2D.Double square = new Rectangle2D.Double(x - fillRadius, y,
                    2 * fillRadius, 1 * fillRadius);
                g2.fill(square);
            }
        }
        if (pedestriansFillMode == PEDESTRIAN_FILL_MODE_EXTRINSIC_FORCES_QUANTITATIVE)
        {
            Vector2D totalExtrinsicForces = pedestrian.getTotalExtrinsicForces();
            if (totalExtrinsicForces != null)
            {
                g2.setColor(getPedestrianColor(MathTools.norm(totalExtrinsicForces)));
            }
            else
            {
                g2.setColor(Color.GREEN);
            }
            Rectangle2D.Double square = new Rectangle2D.Double(x - fillRadius, y - fillRadius,
                2 * fillRadius, 2 * fillRadius);
            // g2.fill(square);
            g2.draw(square);
        }

        // draws resulting velocity
        if (velocityVisible)
        {
            g2.setColor(Color.GREEN);
            float resultingVelocity = (float) MathTools.norm(pedestrian.getCurrentVelocity());
            int fontSize = 2;
            g2.setFont(new Font(Font.SANS_SERIF, Font.PLAIN, fontSize));
            // id for debugging
            g2.drawString("id=" + pedestrian.getId(), (float) x - 3, (float) y - 3);
            g2.drawString(
                "v=" + Math.round(MathTools.convertMsToKmh(resultingVelocity)) + "km/h" + ", f="
                    + MathTools.round(MathTools.norm(pedestrian.getTotalExtrinsicForces()), 2),
                (float) x - 1, (float) y - 1);
        }

        // draws velocity vector (where the pedestrian actually goes)
        if (currentVelocityVectorVisible)
        {
            g2.setColor(Color.GREEN);
            Line2D.Double velocityVector = new Line2D.Double(pedestrian.getCurrentPosition().x(),
                pedestrian.getCurrentPosition().y(),
                pedestrian.getCurrentPosition().x() + pedestrian.getCurrentVelocity().x() * 10,
                pedestrian.getCurrentPosition().y() + pedestrian.getCurrentVelocity().y() * 10);
            g2.draw(velocityVector);
        }

        // draw line from pedestrian to current target point
        if (currentTargetPointVisible)
        {
            if (pedestrian.getMentalModel() instanceof FollowWayPointsMentalModel)
            {
                Vector2D targetCoordinateVector = ((FollowWayPointsMentalModel) pedestrian
                    .getMentalModel()).getTargetCoordinateVector();
                if (targetCoordinateVector != null)
                {
                    g2.setColor(Color.LIGHT_GRAY);
                    Line2D.Double velocityVector = new Line2D.Double(x, y,
                        targetCoordinateVector.x(), targetCoordinateVector.y());
                    g2.draw(velocityVector);
                }
            }
        }

        // draws direction vector (where the pedestrian wants to go)
        if (currentTargetVectorVisible)
        {
            g2.setColor(Color.BLUE);
            Line2D.Double velocityVector = new Line2D.Double(pedestrian.getCurrentPosition().x(),
                pedestrian.getCurrentPosition().y(),
                pedestrian.getCurrentPosition().x()
                    + pedestrian.getMentalModel().getNormalizedDirectionVector().x() * 5,
                pedestrian.getCurrentPosition().y()
                    + pedestrian.getMentalModel().getNormalizedDirectionVector().y() * 5);
            g2.draw(velocityVector);
        }

        // draws extrinsic forces vector (where the pedestrian is accelerated to)
        if (currentExtrinsicForcesVisible)
        {
            Vector2D totalExtrinsicForces = pedestrian.getTotalExtrinsicForces();
            if (totalExtrinsicForces != null)
            {
                g2.setColor(Color.RED);
                Line2D.Double forceVector = new Line2D.Double(pedestrian.getCurrentPosition().x(),
                    pedestrian.getCurrentPosition().y(),
                    pedestrian.getCurrentPosition().x() + totalExtrinsicForces.x() * 5,
                    pedestrian.getCurrentPosition().y() + totalExtrinsicForces.y() * 5);
                g2.draw(forceVector);
            }
        }

        // draws pedestrian forces vector
        if (currentPedestrianForceVisible)
        {
            Vector2D forceInteractionWithPedestrians = pedestrian
                .getForceInteractionWithPedestrians();
            if (forceInteractionWithPedestrians != null)
            {
                g2.setColor(new Color(0, 120, 0));
                Line2D.Double forceVector = new Line2D.Double(pedestrian.getCurrentPosition().x(),
                    pedestrian.getCurrentPosition().y(),
                    pedestrian.getCurrentPosition().x() + forceInteractionWithPedestrians.x() * 5,
                    pedestrian.getCurrentPosition().y() + forceInteractionWithPedestrians.y() * 5);
                g2.draw(forceVector);
            }
        }

        // draws boundary forces vector
        if (currentBoundaryForceVisible)
        {
            Vector2D forceInteractionWithBoundaries = pedestrian
                .getForceInteractionWithBoundaries();
            if (forceInteractionWithBoundaries != null)
            {
                g2.setColor(new Color(120, 0, 0));
                Line2D.Double forceVector = new Line2D.Double(pedestrian.getCurrentPosition().x(),
                    pedestrian.getCurrentPosition().y(),
                    pedestrian.getCurrentPosition().x() + forceInteractionWithBoundaries.x() * 5,
                    pedestrian.getCurrentPosition().y() + forceInteractionWithBoundaries.y() * 5);
                g2.draw(forceVector);
            }
        }
    }

    /**
     * Gets a color that matches the class given in {@link #gridCellColorMap} for the given
     * {@code value}.
     *
     * @param value the value to be used for finding a color in {@link #gridCellColorMap}
     *
     * @return the appropriate color for the given {@code value}
     */
    private Color getGridCellColor(double value)
    {
        // if value equals zero the return white
        if (value == 0d)
        {
            return Color.white;
        }
        // default value = highest possible value
        Color color = gridCellColorMap.entrySet().iterator().next().getValue();

        // look for an appropriate class value depending on the value
        for (Map.Entry<Double, Color> entry : gridCellColorMap.entrySet())
        {
            if (value < entry.getKey())
            {
                color = entry.getValue();
            }
        }
        return color;
    }

    /**
     * Paints the given {@link Grid} object into the given {@link Graphics2D} context.
     *
     * @param g2 the {@link Graphics2D} context to paint in
     */
    private void paintGrid(Graphics2D g2)
    {
        if ( !gridVisible && !gridLabelsVisible)
            return;
        Grid grid = crowdSimulator.getGrid();
        Map<Envelope, Integer> gridCells = grid.getGridCells();
        if (gridCells == null || gridCells.isEmpty())
            return;

        // get defaults
        Stroke defaultStroke = g2.getStroke();
        Color defaultColor = g2.getColor();
        Font defaultFont = g2.getFont();

        for (Map.Entry<Envelope, Integer> gridCell : gridCells.entrySet())
        {
            // skip empty cells
            if (gridCell.getValue() == 0)
                continue;

            // create rectangle for painting
            Rectangle2D.Double cell = new Rectangle2D.Double(gridCell.getKey().getMinX(),
                gridCell.getKey().getMinY(), grid.getCellSize(), grid.getCellSize());

            // crowd density in grid cell (pedestrian / m²)
            double gridCellCrowdDensity = (double) gridCell.getValue()
                / (grid.getCellSize() * grid.getCellSize());

            // paint cells
            if (gridVisible)
            {
                // set color depending on value and paint
                g2.setColor(getGridCellColor(gridCellCrowdDensity));
                g2.fill(cell);
            }

            // draw cell value as String
            if (gridLabelsVisible)
            {
                g2.setColor(Color.BLACK);
                g2.setFont(new Font(Font.SANS_SERIF, Font.PLAIN, 3));
                g2.drawString(String.valueOf(MathTools.round(gridCellCrowdDensity, 2)),
                    (float) gridCell.getKey().getMinX(), (float) gridCell.getKey().getMaxY());
            }
        }

        // reset defaults
        g2.setStroke(defaultStroke);
        g2.setColor(defaultColor);
        g2.setFont(defaultFont);
    }

    /**
     * Implements run() of the interface {@link Runnable}. The Thread processed permanently, that
     * means that it's possible to send instructions to the thread (if the thread is running).
     *
     * @see java.lang.Runnable#run()
     */
    @Override
    public void run()
    {
        // infinite loop runs until application is closed or paused
        while (graphicsThreadRunning)
        {
            // don't refresh more than 50 times per second
            long timeSinceLastRefresh = System.nanoTime() - lastGraphicsRefreshTime;
            if (timeSinceLastRefresh < refreshInterval)
            {
                try
                {
                    long sleepInterval = refreshInterval - timeSinceLastRefresh;
                    long sleepIntervalMs = TimeUnit.NANOSECONDS.toMillis(sleepInterval);
                    long sleepIntervalremainingNanos = (sleepInterval) % 1000000;
                    Thread.sleep(sleepIntervalMs, (int) sleepIntervalremainingNanos);
                }
                catch (InterruptedException e)
                {
                    logger.debug("MapPanel.run(), sleep interrupted", e);
                }
            }

            long currentTime = System.nanoTime();

            // update paintBackBuffer
            repaint();

            // update frame per second rate
            graphicRefreshsPerSecond = 1
                / ((currentTime - (double) lastGraphicsRefreshTime) / 1000_000_000d);

            // set last graphics refresh time
            lastGraphicsRefreshTime = currentTime;
        }
    }

    /**
     * Clears all elements which are inside the {@link MapPanel}
     */
    public void clear()
    {
        backbufferGraphics.clearRect(0, 0, getWidth(), getHeight());
    }

    /**
     * No implementation (i.e. nothing happens).
     *
     * @see java.awt.event.ComponentListener#componentResized(java.awt.event.ComponentEvent)
     */
    @Override
    public void componentResized(ComponentEvent e)
    {
        logger.debug("CrowdSimulation.MapPanel.componentResized(), ");
        createBackBuffer();
        repaint();
    }

    /**
     * No implementation (i.e. nothing happens).
     *
     * @see java.awt.event.ComponentListener#componentMoved(java.awt.event.ComponentEvent)
     */
    @Override
    public void componentMoved(ComponentEvent e)
    {

    }

    /**
     * No implementation (i.e. nothing happens).
     *
     * @see java.awt.event.ComponentListener#componentShown(java.awt.event.ComponentEvent)
     */
    @Override
    public void componentShown(ComponentEvent e)
    {

    }

    /**
     * No implementation (i.e. nothing happens).
     *
     * @see java.awt.event.ComponentListener#componentHidden(java.awt.event.ComponentEvent)
     */
    @Override
    public void componentHidden(ComponentEvent e)
    {

    }

    /**
     * No implementation (i.e. nothing happens).
     *
     * @see java.awt.event.FocusListener#focusGained(java.awt.event.FocusEvent)
     */
    @Override
    public void focusGained(FocusEvent e)
    {

    }

    /**
     * No implementation (i.e. nothing happens).
     *
     * @see java.awt.event.FocusListener#focusLost(java.awt.event.FocusEvent)
     */
    @Override
    public void focusLost(FocusEvent e)
    {
        this.requestFocus();
    }

    /**
     * No implementation (i.e. nothing happens).
     *
     * @see javax.swing.JComponent#processMouseEvent(java.awt.event.MouseEvent)
     */
    @Override
    public void processMouseEvent(MouseEvent e)
    {

    }

    /**
     * Implements mouseWheelMoved() of the interface {@link MouseWheelListener}. Changes the scale.
     *
     * @see MouseWheelListener#mouseWheelMoved(MouseWheelEvent)
     */
    @Override
    public void mouseWheelMoved(MouseWheelEvent event)
    {
        Envelope oldBoundingBox = currentMapExtent;
        Coordinate oldMapOrigin = currentMapOrigin;

        int wheelRotation = event.getWheelRotation();

        // screen coordinates
        double xEventPosition = event.getX();
        double yEventPosition = event.getY();

        logger.trace("mouseWheelMoved(), " + wheelRotation);
        logger.trace("mouseWheelMoved(), " + xEventPosition);
        logger.trace("mouseWheelMoved(), " + yEventPosition);

        double zoomFactor = 2;
        currentScale *= Math.pow(zoomFactor, -1 * wheelRotation);

        // meters
        double deltaX = xEventPosition / currentScale;
        double deltaY = yEventPosition / currentScale;

        // meters
        double mapWidth = currentMapExtent.getMaxX() - currentMapExtent.getMinX();
        double mapHeight = currentMapExtent.getMaxY() - currentMapExtent.getMinY();

        Envelope updatedBoundingBox = null;
        Coordinate updatedMapOrigin = null;

        if (wheelRotation < 0)
        {
            logger.trace("MapPanel.mouseWheelMoved(), zoom in");
            updatedMapOrigin = new Coordinate(oldMapOrigin.x + deltaX / 2 * zoomFactor,
                oldMapOrigin.y + deltaY / 2 * zoomFactor);

            updatedBoundingBox = new Envelope(updatedMapOrigin.x,
                updatedMapOrigin.x + mapWidth / zoomFactor, updatedMapOrigin.y,
                updatedMapOrigin.y + mapHeight / zoomFactor);
        }
        else
        {
            logger.trace("MapPanel.mouseWheelMoved(), zoom out");
            updatedMapOrigin = new Coordinate(oldMapOrigin.x - deltaX / zoomFactor,
                oldMapOrigin.y - deltaY / zoomFactor);

            updatedBoundingBox = new Envelope(updatedMapOrigin.x,
                updatedMapOrigin.x + mapWidth * zoomFactor, updatedMapOrigin.y,
                updatedMapOrigin.y + mapHeight * zoomFactor);
        }

        logger.trace("MapPanel.mouseWheelMoved(), oldBoundingBox=" + oldBoundingBox + ", area="
            + oldBoundingBox.getArea());
        logger.trace("MapPanel.mouseWheelMoved(), updatedBoundingBox=" + updatedBoundingBox
            + ", area=" + updatedBoundingBox.getArea());

        currentMapExtent = updatedBoundingBox;
        currentMapOrigin = updatedMapOrigin;

        repaint();
    }

    /**
     * No implementation (i.e. nothing happens).
     *
     * @param e object is the {@link KeyEvent} which occurs if a button is pressed
     *
     * @see java.awt.event.KeyListener#keyPressed(java.awt.event.KeyEvent)
     */
    @Override
    public void keyTyped(KeyEvent e)
    {

    }

    /**
     * No implementation (i.e. nothing happens).
     *
     * @param e object is the {@link KeyEvent} which occurs if a button is pressed
     *
     * @see java.awt.event.KeyListener#keyPressed(java.awt.event.KeyEvent)
     */
    @Override
    public void keyPressed(KeyEvent e)
    {

    }

    /**
     * Shifts the Map if one of the arrow keys is pressed and also released.
     *
     * @param e object is the {@link KeyEvent} which occurs if a button is released
     *
     * @see java.awt.event.KeyListener#keyReleased(java.awt.event.KeyEvent)
     */
    @Override
    public void keyReleased(KeyEvent e)
    {
        double moveDistance = offset / currentScale;
        Envelope oldBoundingBox = currentMapExtent;
        Envelope updatedBoundingBox = null;
        Coordinate oldMapOrigin = currentMapOrigin;
        Coordinate updatedMapOrigin = null;
        if (e.getKeyCode() == KeyEvent.VK_UP)
        {
            updatedMapOrigin = new Coordinate(oldMapOrigin.x, oldMapOrigin.y - moveDistance);
            updatedBoundingBox = new Envelope(oldBoundingBox.getMinX(), oldBoundingBox.getMaxX(),
                oldBoundingBox.getMinY() - moveDistance, oldBoundingBox.getMaxY() - moveDistance);
        }
        if (e.getKeyCode() == KeyEvent.VK_DOWN)
        {
            updatedMapOrigin = new Coordinate(oldMapOrigin.x, oldMapOrigin.y + moveDistance);
            updatedBoundingBox = new Envelope(oldBoundingBox.getMinX(), oldBoundingBox.getMaxX(),
                oldBoundingBox.getMinY() + moveDistance, oldBoundingBox.getMaxY() + moveDistance);
        }
        if (e.getKeyCode() == KeyEvent.VK_RIGHT)
        {
            updatedMapOrigin = new Coordinate(oldMapOrigin.x + moveDistance, oldMapOrigin.y);
            updatedBoundingBox = new Envelope(oldBoundingBox.getMinX() + moveDistance,
                oldBoundingBox.getMaxX() + moveDistance, oldBoundingBox.getMinY(),
                oldBoundingBox.getMaxY());
        }
        if (e.getKeyCode() == KeyEvent.VK_LEFT)
        {
            updatedMapOrigin = new Coordinate(oldMapOrigin.x - moveDistance, oldMapOrigin.y);
            updatedBoundingBox = new Envelope(oldBoundingBox.getMinX() - moveDistance,
                oldBoundingBox.getMaxX() - moveDistance, oldBoundingBox.getMinY(),
                oldBoundingBox.getMaxY());
        }
        if (updatedMapOrigin != null)
            currentMapOrigin = updatedMapOrigin;
        if (updatedBoundingBox != null)
            currentMapExtent = updatedBoundingBox;
        repaint();
    }
}
