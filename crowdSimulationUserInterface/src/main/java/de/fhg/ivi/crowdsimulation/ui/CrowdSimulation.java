package de.fhg.ivi.crowdsimulation.ui;

import java.awt.BorderLayout;
import java.awt.Color;
import java.awt.ScrollPane;
import java.awt.event.WindowAdapter;
import java.awt.event.WindowEvent;
import java.io.File;
import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;

import javax.swing.ImageIcon;
import javax.swing.JFileChooser;
import javax.swing.JFrame;
import javax.swing.JOptionPane;
import javax.swing.JPanel;
import javax.swing.JScrollPane;
import javax.swing.ScrollPaneConstants;
import javax.swing.UIManager;
import javax.swing.UnsupportedLookAndFeelException;

import org.opengis.geometry.BoundingBox;
import org.opengis.referencing.crs.CoordinateReferenceSystem;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import com.vividsolutions.jts.geom.Geometry;

import de.fhg.ivi.crowdsimulation.simulation.CrowdSimulator;
import de.fhg.ivi.crowdsimulation.simulation.CrowdSimulatorNotValidException;
import de.fhg.ivi.crowdsimulation.simulation.objects.Boundary;
import de.fhg.ivi.crowdsimulation.simulation.objects.Crowd;
import de.fhg.ivi.crowdsimulation.simulation.objects.Pedestrian;
import de.fhg.ivi.crowdsimulation.simulation.objects.RoutingNetwork;
import de.fhg.ivi.crowdsimulation.simulation.objects.WayPoint;
import de.fhg.ivi.crowdsimulation.ui.gui.PanelManager;
import de.fhg.ivi.crowdsimulation.ui.gui.control.MenuBar;
import de.fhg.ivi.crowdsimulation.ui.gui.control.SettingsPanel;
import de.fhg.ivi.crowdsimulation.ui.gui.control.Toolbar;
import de.fhg.ivi.crowdsimulation.ui.gui.tools.DataTools;
import de.fhg.ivi.crowdsimulation.ui.gui.visualisation.InformationPanel;
import de.fhg.ivi.crowdsimulation.ui.gui.visualisation.MapPanel;
import hcu.csl.agentbasedmodeling.PedestrianAgent;

/**
 * Main class of the simulation.
 * <p>
 * Three {@link Thread}s were invoked in this class for parallel running activities. The
 * {@link #simulationThread} providing the simulation part of the application, see
 * {@link CrowdSimulator}, the {@link #graphicsThread} providing a thread for the display of the
 * graphic, see {@link MapPanel}, and the {@link #infoThread}, which provides a thread for the
 * output of informations on the state of the simulation, see {@link InformationPanel}.
 * <p>
 * Furthermore all parts of the GUI are invoked in this class and added to the {@link PanelManager},
 * which manages these classes.
 * <p>
 * Farther importing and editing of all kinds of necessary geometries in case of this crowd
 * simulation is done implemented here. The necessary geometry are represented through the classes
 * {@link Pedestrian}, {@link Boundary} and {@link WayPoint}.
 *
 * @author hahmann/meinert
 */
public class CrowdSimulation extends JFrame
{
    /**
     * Uses the object logger for printing specific messages in the console.
     */
    private static final Logger logger           = LoggerFactory.getLogger(CrowdSimulation.class);

    /**
     * default serial version ID
     */
    private static final long   serialVersionUID = 1L;

    /**
     * Main simulation object that contains the lists of {@link Pedestrian} objects, boundaries and
     * waypoints and computes the actual simulation.
     */
    private CrowdSimulator      crowdSimulator;

    /**
     * Threading for simulation purposes (for the pedestrian calculations).
     */
    private Thread              simulationThread;

    /**
     * Threading for implementation/drawing of graphics.
     */
    private Thread              graphicsThread;

    /**
     * Threading for informations in the {@link InformationPanel}.
     */
    private Thread              infoThread;

    /**
     * Thread Pool for parallelization of Pedestrian movement computation
     */
    private ExecutorService     threadPool;

    /**
     * This object represents the "Map" class, this means the part in which buildings, waypoints,
     * pedestrians and so on are drawn, and in which the simulation will be visible.
     */
    private MapPanel            mapPanel;

    /**
     * This objects represents the {@link JPanel} for the manipulation of the simulation.
     */
    private SettingsPanel       settingsPanel;

    /**
     * This object represents the {@link JPanel} for logging informations over the simulation.
     */
    private InformationPanel    informationPanel;

    /**
     * Object of this management class, which gets and sets all {@link JPanel} classes.
     */
    private PanelManager        panelManager;

    /**
     * shp File with boundaries.
     */
    private static File         boundaryFile;

    /**
     * shp File with pedestrians.
     */
    private static File         pedestrianFile;

    /**
     * csv File with waypoints (pedMission.csv).
     */
    private static File         waypointFile;

    /**
     * shp File with network.
     */
    private static File         networkFile;

    // private static File missionFile;

    /**
     * Constructor of {@link CrowdSimulation}
     * <p>
     * Calls {@link #loadInitialAgentData()} to load default data into the simulation.
     * <p>
     * Definitions for all parts of the GUI are set here, which includes the size of the screen, the
     * {@link ImageIcon}, a {@link ScrollPane} and so on.
     * <p>
     * Starts the {@link #infoThread}.
     */
    public CrowdSimulation()
    {
        // TODO: Prioritize Threads + give highest priority to ui thread using this (only necessary,
        // if graphics thread seems to get slow due to parallelization)
        // https://funofprograming.wordpress.com/2016/10/08/priorityexecutorservice-for-java/
        int availableProcessors = Runtime.getRuntime().availableProcessors();
        logger
            .trace("CrowdSimulation.CrowdSimulation(), availableProcessors=" + availableProcessors);
        threadPool = Executors.newFixedThreadPool(availableProcessors);

        // initialize crowd simulator
        crowdSimulator = new CrowdSimulator(threadPool);

        // creating geometries for GUI
        // loadInitialData();
        loadInitialAgentData();

        // background, size and layout
        setBackground(Color.WHITE);
        setSize(1280, 1025);
        setLayout(new BorderLayout());

        // the panel manager
        panelManager = new PanelManager();

        // Panel for Toolbar and MenuBar (at the top)
        Toolbar toolbar = new Toolbar(this);
        add(BorderLayout.NORTH, toolbar);
        setJMenuBar(new MenuBar(this));

        // Panel for informations over the simulation
        informationPanel = new InformationPanel(this);
        add(BorderLayout.SOUTH, informationPanel);
        panelManager.addPanel(informationPanel);

        // Panel for the simulation(map itself)
        mapPanel = new MapPanel(crowdSimulator);
        add(BorderLayout.CENTER, mapPanel);
        panelManager.addPanel(mapPanel);

        // Panel for the manipulation of the simulation
        settingsPanel = new SettingsPanel(this);
        // add(BorderLayout.EAST, settingsPanel);
        JScrollPane scrollPane = new JScrollPane(settingsPanel);
        scrollPane.setHorizontalScrollBarPolicy(ScrollPaneConstants.HORIZONTAL_SCROLLBAR_NEVER);
        scrollPane.setVerticalScrollBarPolicy(ScrollPaneConstants.VERTICAL_SCROLLBAR_ALWAYS);
        add(BorderLayout.EAST, scrollPane);
        panelManager.addPanel(settingsPanel);

        // make JFrame visible
        setVisible(true);

        // set window icon and title
        setIconImage(new ImageIcon("src/main/resources/icon/TargetIcon_32.png").getImage());
        setTitle("TARGET Crowd Simulation");

        // start info thread
        startInfoThread();

        // request focus in mapPanel
        mapPanel.requestFocus();

        // stop all thread, when JFrame is closed
        addWindowListener(new WindowAdapter()
            {
                @Override
                public void windowClosing(WindowEvent e)
                {
                    stopGraphicsThread();
                    stopInfoThread();
                    stopSimulationThread();
                    if (threadPool != null)
                        threadPool.shutdown();
                    System.exit(0);
                }
            });
    }

    /**
     * Gets an object of {@link CrowdSimulator}.
     *
     * @return an object of the class {@link CrowdSimulator}
     */
    public CrowdSimulator getCrowdSimulator()
    {
        return crowdSimulator;
    }

    /**
     * Proofs whether all conditions or preconditions, for the start of the simulation, are
     * fulfilled.
     * <p>
     * If conditions are fulfilled the {@link #simulationThread} will started.
     */
    // public void startSimulationThread()

    /**
     * Proofs whether all conditions or preconditions, for the start of the simulation, are
     * fulfilled.
     * <p>
     * If conditions are fulfilled the {@link #simulationThread} will started.
     */
    public void startAgentSimulationThread()
    {
        try
        {
            // checks, if the crowd simulator is valid
            crowdSimulator.validateAgents(crowdSimulator.getCrowds(),
                crowdSimulator.getBoundaries());

            // set current system time as starting time for pedestrians
            for (Crowd crowd : crowdSimulator.getCrowds())
            {
                for (Pedestrian pedestrian : crowd.getPedestrians())
                {
                    pedestrian.getMentalModel()
                        .setStartTime(crowdSimulator.getFastForwardClock().currentTimeMillis());
                }
            }

            // starts the thread for the simulation itself
            simulationThread = new Thread(crowdSimulator);
            simulationThread.start();
            crowdSimulator.setSimulationThreadRunning(true);
        }
        catch (CrowdSimulatorNotValidException e)
        {
            JOptionPane.showMessageDialog(null, e.getMessage(), "Warning",
                JOptionPane.WARNING_MESSAGE);
            logger.debug("CrowdSimulation.startSimulationThread(), ", e);
        }
    }

    /**
     * Stops the {@link #simulationThread}.
     */
    public void stopSimulationThread()
    {
        crowdSimulator.setSimulationThreadRunning(false);
        crowdSimulator.setLastSimulationTime(0);
    }

    /**
     * Starts the {@link #graphicsThread}, but only in the case, that the {@link #simulationThread}
     * is running too.
     */
    public void startGraphicsThread()
    {
        // only start the graphics
        if (crowdSimulator.isSimulationThreadRunning())
        {
            graphicsThread = new Thread(mapPanel);
            graphicsThread.start();
            mapPanel.setGraphicsThreadRunning(true);
        }
    }

    /**
     * Stops the {@link #graphicsThread}.
     */
    public void stopGraphicsThread()
    {
        mapPanel.setGraphicsThreadRunning(false);
    }

    /**
     * Starts the {@link #infoThread}.
     */
    public void startInfoThread()
    {
        infoThread = new Thread(informationPanel);
        infoThread.start();
        informationPanel.setInfoThreadRunning(true);
    }

    /**
     * Stops the {@link #infoThread}.
     */
    public void stopInfoThread()
    {
        informationPanel.setInfoThreadRunning(false);
    }

    /**
     * Gets an object of {@link PanelManager}.
     *
     * @return an object of the {@link PanelManager}
     */
    public PanelManager getPanelManager()
    {
        return panelManager;
    }

    /**
     * Sets the {@link PanelManager} object.
     *
     * @param panelManager object of the {@link PanelManager} which organizes all {@link JPanel}s.
     */
    public void setPanelManager(PanelManager panelManager)
    {
        this.panelManager = panelManager;
    }

    /**
     * Loads the default data for {@link Boundary}s, {@link WayPoint}s and {@link Pedestrian}s
     * objects.
     */
    // private void loadInitialData()

    /**
     * Loads the default data for {@link Boundary}s and {@link PedestrianAgent}s objects.
     */
    private void loadInitialAgentData()
    {
        try
        {

            // data for a "arg" run
            loadBoundaries(boundaryFile, false);
            loadAgents(pedestrianFile, waypointFile, networkFile, false);
            // loadWayPoints(waypointFile, false);
            // loadMission(missionFile, false);
            // loadNetwork(networkFile, false);

        }
        catch (CrowdSimulatorNotValidException e)
        {
            JOptionPane.showMessageDialog(null, e.getMessage(), "Warning",
                JOptionPane.WARNING_MESSAGE);
            logger.debug("CrowdSimulation.loadInitialData(), ", e);
        }
    }

    /**
     * Loads {@link WayPoint}s as a {@link List} of {@link Coordinate} objects from the
     * {@code file}, adds them to the {@link CrowdSimulator} and draws them in the {@link MapPanel}.
     *
     * @param validate if {@code true} the {@link CrowdSimulator} is validated against the new data
     *            and the new data is only used if the validation is successful
     * @param file {@link File} as WKT data or .shp data
     * @throws CrowdSimulatorNotValidException
     */
    // public void loadWayPoints(File file, boolean validate) throws CrowdSimulatorNotValidException

    /**
     * Loads all {@link Boundary} objects as a {@link List} of {@link Geometry} objects from the
     * {@code file}, adds them to the {@link CrowdSimulator} and draws them in the {@link MapPanel}.
     * <p>
     * Furthermore this method sets a {@link BoundingBox} in {@link CrowdSimulator} as some kind of
     * reference region for the whole model. This bounding box based on the extent of the
     * {@link Boundary}s (the extent of the bounding box is also based on the {@link Geometry} of
     * the {@link Pedestrian}s).
     *
     * @param validate if {@code true} the {@link CrowdSimulator} is validated against the new data
     *            and the new data is only used if the validation is successful
     * @param file {@link File} as WKT data or .shp data
     * @throws CrowdSimulatorNotValidException
     */
    public void loadBoundaries(File file, boolean validate) throws CrowdSimulatorNotValidException
    {
        if (file == null)
        {
            file = chooseFile(file);
        }

        // do nothing if no file has been chosen
        if (file == null)
            return;

        List<Geometry> geometries = DataTools.loadGeometriesFromFile(file);

        // only use boundaries if the List<> isn't null
        if (geometries != null && !geometries.isEmpty())
        {
            List<Boundary> boundaries = new ArrayList<>();
            for (Geometry geometry : geometries)
            {
                boundaries.add(new Boundary(geometry,
                    crowdSimulator.getForceModel().getMaxBoundaryInteractionDistance()));
            }

            if (validate)
                crowdSimulator.validateAgents(crowdSimulator.getCrowds(), boundaries);

            // set crs
            CoordinateReferenceSystem crs = DataTools.getCRSFromFile(file);
            crowdSimulator.setCrs(crs);

            // set Boundaries
            crowdSimulator.setBoundaries(boundaries);

            // set bounding box
            crowdSimulator.setBoundingBox(geometries);

            // paint
            if (mapPanel != null)
            {
                mapPanel.resetMapExtent();
                mapPanel.repaint();
            }
        }
    }

    /**
     * Loads {@link Pedestrian} objects as a {@link List} of {@link Coordinate} objects from the
     * {@code file}, adds them to the {@link CrowdSimulator} and draws them in the {@link MapPanel}.
     * <p>
     * Based on these {@link Pedestrian}s a single {@link Crowd} is created and added to the
     * CrowdSimulator.
     *
     * @param validate if {@code true} the {@link CrowdSimulator} is validated against the new data
     *            and the new data is only used if the validation is successful
     * @param file {@link File} as WKT data or .shp data
     * @throws CrowdSimulatorNotValidException
     */
    // public void loadCrowd(File file, boolean validate) throws CrowdSimulatorNotValidException

    /**
     * Loads {@link PedestrianAgents} objects and the network file.
     * <p>
     * Based on these {@link Pedestrian}s a single {@link Crowd} is created and added to the
     * CrowdSimulator.
     *
     * @param validate if {@code true} the {@link CrowdSimulator} is validated against the new data
     *            and the new data is only used if the validation is successful
     * @param fileSHP pedestrians {@link File} as .shp data
     * @param fileCSV waypoints {@link File} as csv data (pedMission.csv)
     * @param fileNetwork network {@link File} as .shp data
     * @throws CrowdSimulatorNotValidException
     */
    public void loadAgents(File fileSHP, File fileCSV, File fileNetwork, boolean validate)
        throws CrowdSimulatorNotValidException
    {
        if (fileSHP == null)
        {
            fileSHP = chooseFile(fileSHP);
        }

        // do nothing if no file has been chosen
        if (fileSHP == null)
            return;

        List<PedestrianAgent> pedestrians = DataTools.loadDataFromCsvFile(fileCSV);

        List<Geometry> pedestriansSHP = DataTools.loadGeometriesFromFile(fileSHP);

        crowdSimulator
            .setNetwork(new RoutingNetwork(DataTools.loadFeaturesFromShapeFile(fileNetwork)));

        // crowdSimulator.setPedestrianPositions(pedestrians);

        // only use pedestrians if the List<> isn't null
        if (pedestriansSHP != null && !pedestriansSHP.isEmpty())
        {

            // set crs
            CoordinateReferenceSystem crs = DataTools.getCRSFromFile(fileSHP);
            crowdSimulator.setCrs(crs);

            // add crowd
            crowdSimulator.addAgentCrowd(pedestrians);

            // set bounding box
            crowdSimulator.setBoundingBox(pedestriansSHP);

            // paint
            if (mapPanel != null)
            {
                mapPanel.resetMapExtent();
                mapPanel.repaint();
            }
        }
    }

    /**
     * @param file network .shp-File
     * @param validate
     * @throws CrowdSimulatorNotValidException
     */
    public void loadNetwork(File file, boolean validate) throws CrowdSimulatorNotValidException
    {
        if (file == null)
        {
            file = chooseFile(file);
        }

        // do nothing if no file has been chosen
        if (file == null)
            return;

        RoutingNetwork r = new RoutingNetwork(DataTools.loadFeaturesFromShapeFile(networkFile));

        crowdSimulator.setNetwork(r);

        // paint
        if (mapPanel != null)
        {
            mapPanel.resetMapExtent();
            mapPanel.repaint();
        }

    }

    /**
     * Imports a {@link File} object, which can be chosen by the user.
     *
     * @param file the WKT/Shape {@link File} object to be parsed.
     *
     * @return file the WKT/Shape {@link File} object to be parsed
     */
    private File chooseFile(File file)
    {
        JFileChooser chooser = new JFileChooser();
        if (chooser.showOpenDialog(this) == JFileChooser.APPROVE_OPTION)
        {
            file = chooser.getSelectedFile();
        }
        return file;
    }

    /**
     * Invokes the constructor of this class.
     *
     * @param args not used
     */
    public static void main(String[] args)
    {

        System.out.println("START ABPedSim");
        String boundaryFileName = args[0];
        boundaryFile = new File(boundaryFileName);
        String pedestrianFileName = args[1];
        pedestrianFile = new File(pedestrianFileName);
        String waypointFileName = args[2];
        waypointFile = new File(waypointFileName);
        String networkFileName = args[3];
        networkFile = new File(networkFileName);

        // set system look and feel (adapts the look of java to the systems default look)
        try
        {
            UIManager.setLookAndFeel(UIManager.getSystemLookAndFeelClassName());
        }
        catch (ClassNotFoundException | InstantiationException | IllegalAccessException
            | UnsupportedLookAndFeelException e)
        {
            logger.debug("Could not set Look and Feel.", e);
        }
        @SuppressWarnings("unused")
        CrowdSimulation ps = new CrowdSimulation();

        // Direktstart
        // ps.startSimulationThread();
        ps.startAgentSimulationThread();

        ps.startGraphicsThread();

        // ExportWKTAction.exportWKT(ps);
    }
}
