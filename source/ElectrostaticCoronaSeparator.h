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




/// Utility function that plots a matrix over a rectangle
static void drawDistribution(irr::video::IVideoDriver* driver,
                             const ChMatrix<>& Z, // distribution matrix
                             const ChCoordsys<>& mpos, // center coordinates of the rectangle that measures flow
                             double x_size, double y_size, // size of the rectangle
                             irr::video::SColor mcol = irr::video::SColor(50, 80, 110, 110),
                             bool use_Zbuffer = false
);

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
	const double epsilon = 8.85941e-12; // dielectric constant [F/m] *****ida 
	const double epsilonO = 8.854187e-12; //vacuum permeability
	const double epsilonR = 2.5; //relative permeability
	const double eta = 0.0000181; // Air drag coefficent [N*s/m^2]
	const double ro = 1.225;  //fluid density (air) [Kg/m^3]


    ChParticleEmitter emitter;
    std::shared_ptr<ChRandomParticlePositionRectangleOutlet> emitter_positions;
    std::shared_ptr<ChRandomParticleAlignment> emitter_rotations;

    //double drumspeed_rpm = 44.8; // [rpm]
    const double drumspeed_rpm_max = 100;
    double drumspeed_rpm = 50; // [rpm]
    double drumspeed_radss = drumspeed_rpm*(CH_C_2PI / 60.0); //[rad/s]

    std::shared_ptr<ChFunction_Const> drum_speed_function;

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

    const double conveyor_thick = 0.01;
    const double conveyor_length = 0.6;

    //ChCoordsys<> conveyor_csys{ 0, -conveyor_thick, 0 };
    //ChCoordsys<> drum_csys{ conveyor_length / 2, -(0.320*0.5) - conveyor_thick / 2, 0 };

    //ChCoordsys<> nozzle_csys{ 0, 0.01, 0 };
    //ChCoordsys<> splitter1_csys{ conveyor_length / 2 + 0.2, -(0.320*0.5) - conveyor_thick / 2,0 };
    //ChCoordsys<> splitter2_csys{ conveyor_length / 2 + 0.4, -(0.320*0.5) - conveyor_thick / 2,0 };
    //ChCoordsys<> brush_csys{ conveyor_length / 2 - 0.10, -(0.320*0.5) - conveyor_thick / 2,0 };


    ChCoordsys<> conveyor_csys;
    ChCoordsys<> drum_csys;

    ChCoordsys<> nozzle_csys;
    ChCoordsys<> splitter1_csys;
    ChCoordsys<> splitter2_csys;
    ChCoordsys<> brush_csys;


    // set as true for saving log files each n frames
    bool save_dataset = false;
    bool save_irrlicht_screenshots = false;
    bool save_POV_screenshots = false;
    int saveEachNframes = 3;


    int totframes = 0;
    bool init_particle_speed = true;
    double particle_magnification = 5; // for larger visualization of particle
    std::string results_file = "output/results.txt";
    double timestep = 0.001;
    double Tmax = 5;
    bool splitters_collide = true;

    std::vector<std::shared_ptr<ChBody>> scanned_particles;


    ElectrostaticCoronaSeparator(ChSystem& mphysicalSystem);


    ///
	/// Function that defines the forces on the debris ****ida
	///
    void apply_forces(ChSystem* msystem);

    /// Acquire particles information based on text file given by the spectrophotometric camera.
    bool AcquireParticleScan(const char* filename);

    /// Creates random bodies according to the last scan.
    void create_debris_particlescan(double dt, double particles_second,
                                    ChSystem& mysystem,
                                    irrlicht::ChIrrApp* irr_application);

    template <typename asset_type>
    bool get_asset(ChAssembly::IteratorBodies& body_iter, std::shared_ptr<asset_type>** desired_asset) const;


    void create_debris_particlescan_temp(double dt, double particles_second,
                                         ChSystem& mysystem,
                                         irrlicht::ChIrrApp* irr_application);

    /// Function that deletes old debris (to avoid infinite creation that fills memory)
    void purge_debris(ChSystem& mysystem, double max_age = 5.0) const;

    /// Function for drawing forces
    void DrawForces(irrlicht::ChIrrApp& application, double scalefactor = 1.0) const;

    /// Function to update trajectories. Must be
    /// called at each timestep
    void UpdateTrajectories(irrlicht::ChIrrApp& application) const;

    /// Function to draw trajectories. Must be
    /// called at each timestep
    void DrawTrajectories(irrlicht::ChIrrApp& application) const;

    /// Add bodies belonging to ECS to the \c system and their visual assets to \c application
    int Setup(ChSystem& system, irrlicht::ChIrrApp* application);

    /// Main function of the simulator.
    /// Initialize the simulation, and
    /// Performs the simulation
    /// by running the loop of time integration
    int RunSimulation(irrlicht::ChIrrApp& application);


    void SetDrumSpeed(double speed_rpm)
    {
        drumspeed_rpm = speed_rpm;
        drumspeed_radss = drumspeed_rpm*CH_C_2PI / 60;
    }
    double GetDrumSpeed() const { return drumspeed_rpm; }



};


#endif

