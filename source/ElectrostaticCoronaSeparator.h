#ifndef ELECTROSTATICCORONASEPARATOR_H
#define ELECTROSTATICCORONASEPARATOR_H

#include "physics/ChSystem.h"
#include <fstream>
#include "physics/ChBodyEasy.h"
#include "physics/ChConveyor.h"
#include "physics/ChBodyAuxRef.h"
#include "core/ChFileutils.h"
#include "chrono_irrlicht/CHirrApp.h"
#include "core/ChRealtimeStep.h"
#include "core/ChMath.h"
#include "core/ChDistribution.h"
#include "collision/ChCCollisionSystemBullet.h"
#include "particlefactory/ChParticleEmitter.h"
#include "particlefactory/ChParticleProcessor.h"

//#include "chrono_python/ChPython.h"
#include "chrono_postprocess/ChPovRay.h"

//#include "rapidjson/document.h"
//#include "rapidjson/prettywriter.h"
//#include "rapidjson/filereadstream.h"
//#include "rapidjson/filewritestream.h"

#include "UserInterfaceEventReceiver.h"
#include "ProcessFlow.h"
#include "ElectricParticleAssets.h"
//#include "ParserEmitter.h"
//#include "ParserElectricForcesCES.h"
//#include "ProcessFlow.h"


using namespace chrono;
using namespace particlefactory;

double xnozzle = 0;
double ynozzle = 0;


/// Utility function that plots a matrix over a rectangle
static void drawDistribution(irr::video::IVideoDriver* driver,
    chrono::ChMatrix<>& Z, // distribution matrix
    chrono::ChCoordsys<>& mpos, // center coordinates of the rectangle that measures flow
    double x_size, double y_size, // size of the rectangle
    irr::video::SColor mcol = irr::video::SColor(50, 80, 110, 110),
    bool use_Zbuffer = false
)
{
    driver->setTransform(irr::video::ETS_WORLD, irr::core::matrix4());
    irr::video::SMaterial mattransp;
    mattransp.ZBuffer = true;
    mattransp.Lighting = false;
    driver->setMaterial(mattransp);

    chrono::ChVector<> V1a(-x_size*0.5, y_size*0.5, 0);
    chrono::ChVector<> V2a(x_size*0.5, y_size*0.5, 0);
    irrlicht::ChIrrTools::drawSegment(driver, mpos.TransformLocalToParent(V1a), mpos.TransformLocalToParent(V2a), mcol, use_Zbuffer);
    chrono::ChVector<> V1b(-x_size*0.5, -y_size*0.5, 0);
    chrono::ChVector<> V2b(x_size*0.5, -y_size*0.5, 0);
    irrlicht::ChIrrTools::drawSegment(driver, mpos.TransformLocalToParent(V1b), mpos.TransformLocalToParent(V2b), mcol, use_Zbuffer);
    chrono::ChVector<> V1c(x_size*0.5, y_size*0.5, 0);
    chrono::ChVector<> V2c(x_size*0.5, -y_size*0.5, 0);
    irrlicht::ChIrrTools::drawSegment(driver, mpos.TransformLocalToParent(V1c), mpos.TransformLocalToParent(V2c), mcol, use_Zbuffer);
    chrono::ChVector<> V1d(-x_size*0.5, y_size*0.5, 0);
    chrono::ChVector<> V2d(-x_size*0.5, -y_size*0.5, 0);
    irrlicht::ChIrrTools::drawSegment(driver, mpos.TransformLocalToParent(V1d), mpos.TransformLocalToParent(V2d), mcol, use_Zbuffer);

    for (int iy = 0; iy < Z.GetColumns(); ++iy)
    {
        double mystep = y_size / Z.GetColumns();
        double my = -0.5*y_size + iy*mystep + 0.5*mystep;
        for (int ix = 0; ix < Z.GetRows(); ++ix)
        {
            double mxstep = x_size / Z.GetRows();
            double mx = -0.5*x_size + ix*mxstep + 0.5*mxstep;
            if (ix >0)
            {
                chrono::ChVector<> Vx1(mx - mxstep, my, Z(ix - 1, iy));
                chrono::ChVector<> Vx2(mx, my, Z(ix, iy));
                irrlicht::ChIrrTools::drawSegment(driver, mpos.TransformLocalToParent(Vx1), mpos.TransformLocalToParent(Vx2), mcol, use_Zbuffer);
            }
            if (iy >0)
            {
                chrono::ChVector<> Vy1(mx, my - mystep, Z(ix, iy - 1));
                chrono::ChVector<> Vy2(mx, my, Z(ix, iy));
                irrlicht::ChIrrTools::drawSegment(driver, mpos.TransformLocalToParent(Vy1), mpos.TransformLocalToParent(Vy2), mcol, use_Zbuffer);
            }
        }
    }
}

class ElectrostaticCoronaSeparator 
{ 

private:
    double h1 = 0;	//analytical parameter****ida
    double h2 = 0;    //analytical parameter****ida
    double j = 0;//analytical parameter****ida
    double f = 0;//analytical parameter****ida
public:

	// data for this type of asset 
	double drum_diameter = 0.320;
	double drum_width = 0.3;
	double electrode_diameter = 0.038;
	double U = -35000; // supplied high-voltage [v]
	double L = 0.267; //certer distance of rotating roll electrode and electrostatic pole *****ida
	double alpha = (CH_C_PI / 180) * 30; //angle of horizontal line and electrodes center line *****ida
	double epsilon = 8.85941e-12; // dielectric constant [F/m] *****ida 
	double epsilonO = 8.854187e-12; //vacuum permeability
	double epsilonR = 2.5; //relative permeability
	double eta = 0.0000181; // Air drag coefficent [N*s/m^2]
	double ro = 1.225;  //fluid density (air) [Kg/m^3]


    ChSystem mphysicalSystem;

    ChParticleEmitter emitter;
    std::shared_ptr<ChRandomParticlePositionRectangleOutlet> emitter_positions;
    std::shared_ptr<ChRandomParticleAlignment> emitter_rotations;

    double drumspeed_rpm = 44.8; // [rpm]
    double drumspeed_radss = drumspeed_rpm*((2.0*CH_C_PI) / 60.0); //[rad/s]

                                                                   //sphrad = 0.38e-3;
                                                                   //sphrad2 = 0.25e-3;
                                                                   //sphrad3 = 0.794e-3;

                                                                   // material surfaces
    double surface_drum_friction = 0.5;
    double surface_drum_rolling_friction = 0.0;
    double surface_drum_spinning_friction = 0.0;
    double surface_drum_restitution = 0;
    double surface_plate_friction = 0.2;
    double surface_plate_rolling_friction = 0;
    double surface_plate_spinning_friction = 0;
    double surface_plate_restitution = 0;
    std::shared_ptr<ChMaterialSurface> surface_particles;

    double max_particle_age = 2;

    double xnozzlesize = 0.182; //**from CAD, nozzle z width
    double znozzlesize = 0.1; //**from CAD, nozzle x width;

    double flowmeter_xmin = 0.28;
    double flowmeter_xmax = flowmeter_xmin + 0.3;
    double flowmeter_width = 0.2;
    double flowmeter_y = -0.1;
    int    flowmeter_bins = 25;


    // Coordinate systems with position and rotation of important items in the 
    // separator.

    double conv_thick = 0.01;
    double conveyor_length = 0.6;

    ChCoordsys<> conveyor_csys{ 0, -conv_thick, 0 };
    ChCoordsys<> drum_csys{ conveyor_length / 2, -(0.320*0.5) - conv_thick / 2,0 };

    ChCoordsys<> nozzle_csys{ 0, 0.01, 0 };
    ChCoordsys<> splitter1_csys{ conveyor_length / 2 + 0.2, -(0.320*0.5) - conv_thick / 2,0 };
    ChCoordsys<> splitter2_csys{ conveyor_length / 2 + 0.4, -(0.320*0.5) - conv_thick / 2,0 };
    ChCoordsys<> brush_csys{ conveyor_length / 2 - 0.10, -(0.320*0.5) - conv_thick / 2,0 };


    // set as true for saving log files each n frames
    bool save_dataset = false;
    bool save_irrlicht_screenshots = false;
    bool save_POV_screenshots = false;
    int saveEachNframes = 3;

    bool irr_cast_shadows = true;
    int totframes = 0;
    bool init_particle_speed = true;
    double particle_magnification = 3; // for larger visualization of particle
    std::string solidworks_py_modelfile = "../CAD_conveyor/conveyor_Ida"; // note! do not add ".py" after the filename
    std::string results_file = "output/results.txt";
    double timestep = 0.001;
    double Tmax = 5;
    bool splitters_collide = true;

    std::vector<std::shared_ptr<ChBody>> scanned_particles;








    ElectrostaticCoronaSeparator();


    ///
	/// Function that defines the forces on the debris ****ida
	///
    void apply_forces(ChSystem* msystem, // contains all bodies
                      ChCoordsys<>& drum_csys, // pos and rotation of drum 
                      double drumspeed, // speed of drum
                      int totframes);


    bool parse_particlescan(const char* filename);


    void create_debris_particlescan(double dt, double particles_second,
                                    ChSystem& mysystem,
                                    irrlicht::ChIrrApp* irr_application);


    void create_debris_particlescan_temp(double dt, double particles_second,
                                         ChSystem& mysystem,
                                         irrlicht::ChIrrApp* irr_application);

    /// Function that deletes old debris (to avoid infinite creation that fills memory)
    void purge_debris(ChSystem& mysystem, double max_age = 5.0);


    /// Function for drawing forces
    void DrawForces(irrlicht::ChIrrApp& application, double scalefactor = 1.0) const;

    /// Function to update trajectories. Must be
    /// called at each timestep
    void UpdateTrajectories(irrlicht::ChIrrApp& application) const;

    /// Function to draw trajectories. Must be
    /// called at each timestep
    void DrawTrajectories(irrlicht::ChIrrApp& application) const;
    int Setup();

    /// Main function of the simulator.
    /// Initialize the simulation, and
    /// Performs the simulation
    /// by running the loop of time integration
    int simulate();
};


#endif

