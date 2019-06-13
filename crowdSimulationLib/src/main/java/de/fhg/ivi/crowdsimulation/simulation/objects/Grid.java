package de.fhg.ivi.crowdsimulation.simulation.objects;

import java.io.IOException;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.NoSuchElementException;

import org.apache.commons.math3.util.FastMath;
import org.geotools.data.simple.SimpleFeatureIterator;
import org.geotools.data.simple.SimpleFeatureSource;
import org.geotools.geometry.jts.ReferencedEnvelope;
import org.geotools.grid.Grids;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import com.vividsolutions.jts.geom.Envelope;
import com.vividsolutions.jts.geom.Geometry;

import de.fhg.ivi.crowdsimulation.simulation.tools.GeometryTools;

/**
 * A {@link Grid} is a raster of cells with specifiable {@link #cellSize}. It can be used to
 * aggregate the number of {@link Pedestrian} objects within each cell at a given point of time
 *
 * @author hahmann/meinert
 */
public class Grid
{

    /**
     * Uses the object logger for printing specific messages in the console.
     */
    private static final Logger    logger          = LoggerFactory.getLogger(Grid.class);

    /**
     * A map of {@link Envelope} objects (i.e. representing the cells of this Grid) that stores an
     * {@link Integer} value for each grid cell.
     */
    private Map<Envelope, Integer> gridCells;

    /**
     * Unix time stamp of the last full update of the aggregate value associated to each grid cell.
     */
    private long                   lastUpdateTime;

    /**
     * The cell size of the grid. Given in meters.
     */
    private double                 cellSize;

    /**
     * The default cell size of the grid. Given in meters.
     */
    private double                 defaultCellSize = 10d;

    /**
     * The maximum amount of cells this grid should have.
     */
    private double                 maxCells        = 1000000;

    /**
     * Indicates, if the {@link #gridCells} should be updated each time {@link #update(List, long)}
     * is called. Usually, it is unnecessary to do so, if the {@link Grid} is not visible in the
     * Graphical User Interface, since it is currently unused elsewhere.
     */
    private boolean                isUpdating      = false;

    /**
     * Creates a new grid with a cell size of {@link #defaultCellSize} that fits into
     * {@code gridBounds}.
     *
     * @param gridBounds the bounding box to fit the {@link Grid} in.
     */
    public Grid(ReferencedEnvelope gridBounds)
    {
        gridCells = new HashMap<>();
        lastUpdateTime = 0;
        double dynamicCellSize = FastMath.sqrt(gridBounds.getArea() / maxCells);
        double staticCellSize = defaultCellSize;
        this.cellSize = gridBounds.getArea() / (defaultCellSize * defaultCellSize) < maxCells
            ? staticCellSize : dynamicCellSize;
        SimpleFeatureSource grid = Grids.createSquareGrid(gridBounds, cellSize);
        @SuppressWarnings("resource")
        SimpleFeatureIterator features = null;
        try
        {
            features = grid.getFeatures().features();
            while (features.hasNext())
            {
                Geometry gridCell = (Geometry) features.next().getDefaultGeometry();
                gridCells.put(gridCell.getEnvelopeInternal(), 0);
            }
        }
        catch (NoSuchElementException e)
        {
            logger.error("Grid.createGrid(), ", e);

        }
        catch (IOException e)
        {
            logger.error("Grid.createGrid(), ", e);
        }
        finally
        {
            if (features != null)
                features.close();
        }
    }

    /**
     * Gets the map of {@link Envelope} objects (i.e. representing the cells of the Grid) that
     * stores an {@link Integer} value for each grid cell
     *
     * @return the map of grid cells and associated cell values for this {@link Grid} object
     */
    public Map<Envelope, Integer> getGridCells()
    {
        return gridCells;
    }

    /**
     * The cell size of the grid. Given in meters
     *
     * @return the cell size of the grid.
     */
    public double getCellSize()
    {
        return cellSize;
    }

    /**
     * Indicate, if this {@link Grid} is currently updating all its cell values
     *
     * @param isUpdating {@code true} if this {@link Grid} is currently updating all its cell
     *            values, {@code false} otherwise
     */
    public void setUpdating(boolean isUpdating)
    {
        this.isUpdating = isUpdating;
    }

    /**
     * Updates this {@link Grid} object, i.e. all cell values are updated by the aggregated number
     * of {@link Pedestrian} objects contained in each cell.
     *
     * @param pedestrians a {@link List} of {@link Pedestrian} objects that should be counted per
     *            grid cell
     * @param currentTime the current time stamp (the Grid is only updated if last update is more
     *            than 1000ms ago)
     */
    public void update(List<Pedestrian> pedestrians, long currentTime)
    {
        if ( !isUpdating)
            return;

        boolean pastTime = currentTime - lastUpdateTime > 1000;

        if (pastTime)
        {
            for (Map.Entry<Envelope, Integer> gridCell : gridCells.entrySet())
            {
                gridCell.setValue(0);
            }
            for (Map.Entry<Envelope, Integer> gridCell : gridCells.entrySet())
            {
                for (Pedestrian pedestrian : pedestrians)
                {
                    if (GeometryTools.isInside(pedestrian.getCurrentPosition(), gridCell.getKey()))
                    {
                        gridCell.setValue(gridCell.getValue() + 1);
                    }
                }
            }
            lastUpdateTime = currentTime;
        }
    }

    /**
     * Sets {@link #gridCells} to {@code null}.
     */
    public void clear()
    {
        gridCells = null;
    }
}
