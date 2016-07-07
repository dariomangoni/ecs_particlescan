#ifndef ELECTROSTATICCORONASEPARATOR_H
#define ELECTROSTATICCORONASEPARATOR_H

#include "physics/ChSystem.h"
#include "ElectricParticleProperty.h"
#include <fstream>
#include "physics/ChSystem.h"
#include "physics/ChBodyEasy.h"
#include "physics/ChConveyor.h"
#include "physics/ChBodyAuxRef.h"
#include "core/ChFileutils.h"
#include "chrono_irrlicht/ChBodySceneNode.h"
#include "chrono_irrlicht/ChBodySceneNodeTools.h" 
#include "chrono_irrlicht/CHirrApp.h"
#include "core/ChRealtimeStep.h"
#include "core/ChMath.h"
#include "core/ChDistribution.h"
#include "collision/ChCCollisionSystemBullet.h"
#include "particlefactory/ChParticleEmitter.h"
#include "particlefactory/ChParticleRemover.h"
#include "particlefactory/ChParticleProcessor.h"

//#include "chrono_python/ChPython.h"
#include "chrono_postprocess/ChPovRay.h"

//#include "rapidjson/document.h"
//#include "rapidjson/prettywriter.h"
//#include "rapidjson/filereadstream.h"
//#include "rapidjson/filewritestream.h"

#include "ElectricParticleProperty.h"
#include "UserInterfaceEventReceiver.h"
#include "ElectrostaticCoronaSeparator.h"
#include "ProcessFlow.h"
//#include "ParserEmitter.h"
//#include "ParserElectricForcesCES.h"
//#include "ProcessFlow.h"


using namespace chrono;
using namespace particlefactory;



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

private:
	double h1 = 0;	//analytical parameter****ida
	double h2 = 0;    //analytical parameter****ida
	double j = 0;//analytical parameter****ida
	double f = 0;//analytical parameter****ida

public:

	ElectrostaticCoronaSeparator()
	{
        surface_particles = std::make_shared<ChMaterialSurface>();
        surface_particles->SetFriction(0.2f);
        surface_particles->SetRollingFriction(0);
        surface_particles->SetSpinningFriction(0);
        surface_particles->SetRestitution(0);

        // Init coordinate systems with position and rotation of important items in the 
        // simulator. These are initializad with constant values, but if loading the
        // SolidWorks model, they will be changed accordingly to what is found in the CAD 
        // file (see later, where the SolidWorks model is parsed). 
        /*
        //***ALEX disabled because initialized by SolidWorks file, anyway
        double conv_thick = 0.01;
        double conveyor_length = 0.6;
        conveyor_csys	= ChCoordsys<>( ChVector<>(0, -conv_thick, 0) ) ; // default position
        drum_csys		= ChCoordsys<>( ChVector<>(conveyor_length/2, -(drum_diameter*0.5)-conv_thick/2,0) );  // default position
        nozzle_csys		= ChCoordsys<>( ChVector<>(0, 0.01, 0) ); // default position
        splitter1_csys	= ChCoordsys<>( ChVector<>(conveyor_length/2+0.2, -(drum_diameter*0.5)-conv_thick/2,0) );  // default position
        splitter2_csys	= ChCoordsys<>( ChVector<>(conveyor_length/2+0.4, -(drum_diameter*0.5)-conv_thick/2,0) );  // default position
        brush_csys	= ChCoordsys<>( ChVector<>(conveyor_length/2-0.10, -(drum_diameter*0.5)-conv_thick/2,0) );  // default position
        */


        // Set small collision envelopes for objects that will be created from now on..
        collision::ChCollisionModel::SetDefaultSuggestedEnvelope(0.001);  //0.002
        collision::ChCollisionModel::SetDefaultSuggestedMargin(0.0005); //0.0008
                                                                        // Set contact breaking/merging tolerance of Bullet:
        collision::ChCollisionSystemBullet::SetContactBreakingThreshold(0.001);

        // Important! dt is small, and particles are small, so it's better to keep this small...
        mphysicalSystem.SetMaxPenetrationRecoverySpeed(0.15);// not needed in INT_TASORA, only for INT_ANITESCU
        mphysicalSystem.SetMinBounceSpeed(0.1);


        // In the following there is a default initialization of the 
        // particle creation system, based on ChParticleEmitter. 
        // This is a default configuration, that is _overridden_ if you 
        // call ParseSettings() and load a settings.ces file that contain different
        // configurations for the emitter.

        // ---Initialize the randomizer for positions
        emitter_positions = std::make_shared<ChRandomParticlePositionRectangleOutlet>();
        emitter_positions->OutletWidth() = 0.1;    // default x outlet size, from CAD;
        emitter_positions->OutletHeight() = 0.182; // default y outlet size, from CAD;
        emitter.SetParticlePositioner(emitter_positions);

        // ---Initialize the randomizer for alignments
        emitter_rotations = std::make_shared<ChRandomParticleAlignmentUniform>();
        emitter.SetParticleAligner(emitter_rotations);

        // ---Initialize the randomizer for creations, with statistical distribution

        // Create a ChRandomShapeCreator object (ex. here for metal particles)
        std::shared_ptr<ChRandomShapeCreatorSpheres> mcreator_metal(new ChRandomShapeCreatorSpheres);
        mcreator_metal->SetDiameterDistribution(std::make_shared<::ChMinMaxDistribution>(0.002, 0.003));

        // Optional: define a callback to be exectuted at each creation of a metal particle:
        class MyCreator_metal : public ChCallbackPostCreation
        {
            // Here do custom stuff on the just-created particle:
        public:
            virtual ~MyCreator_metal()
            {
            }

            void PostCreation(std::shared_ptr<ChBody> mbody, ChCoordsys<> mcoords, ChRandomShapeCreator& mcreator) override
            {
                // Attach some optional visualization stuff
                //std::shared_ptr<ChTexture> mtexture(new ChTexture);
                //mtexture->SetTextureFilename("../objects/pinkwhite.png");
                //mbody->AddAsset(mtexture);
                std::shared_ptr<ChColorAsset> mvisual(new ChColorAsset);
                mvisual->SetColor(ChColor(0.9f, 0.4f, 0.2f));
                mbody->AddAsset(mvisual);
                // Attach a custom asset. It will hold electrical properties
                std::shared_ptr<ElectricParticleProperty> electric_asset(new ElectricParticleProperty);
                electric_asset->e_fraction = ElectricParticleProperty::e_fraction_sphere;
                electric_asset->e_material = ElectricParticleProperty::e_mat_metal;
                electric_asset->conductivity = 58000000;
                electric_asset->birthdate = this->systemreference->GetChTime();
                ChVector<> Cradii; // use equivalent-inertia ellipsoid to get characteristic size:
                ChVector<> Ine = mbody->GetInertiaXX();
                Cradii.x = sqrt((5. / (2.*mbody->GetMass()))*(Ine.y + Ine.z - Ine.x));
                Cradii.y = sqrt((5. / (2.*mbody->GetMass()))*(Ine.x + Ine.z - Ine.y));
                Cradii.z = sqrt((5. / (2.*mbody->GetMass()))*(Ine.x + Ine.y - Ine.z));
                electric_asset->Cdim = Cradii*2.;
                mbody->AddAsset(electric_asset);
            }
            // here put custom data that might be needed by the callback:
            ChSystem* systemreference;
        };

        auto callback_metal = std::make_shared<MyCreator_metal>();
        callback_metal->systemreference = &this->mphysicalSystem;
        mcreator_metal->SetCallbackPostCreation(callback_metal.get());


        // Create a ChRandomShapeCreator object (ex. here for metal particles)
        auto mcreator_plastic = std::make_shared<ChRandomShapeCreatorSpheres>();
        mcreator_plastic->SetDiameterDistribution(std::make_shared<ChMinMaxDistribution>(0.002, 0.002));

        // Optional: define a callback to be exectuted at each creation of a plastic particle:
        class MyCreator_plastic : public ChCallbackPostCreation
        {
            // Here do custom stuff on the just-created particle:
        public:
            virtual ~MyCreator_plastic()
            {
            }

            void PostCreation(std::shared_ptr<ChBody> mbody, ChCoordsys<> mcoords, ChRandomShapeCreator& mcreator) override
            {
                // Attach some optional visualization stuff
                //std::shared_ptr<ChTexture> mtexture(new ChTexture);
                //mtexture->SetTextureFilename("../objects/bluwhite.png");
                //mbody->AddAsset(mtexture);
                std::shared_ptr<ChColorAsset> mvisual(new ChColorAsset);
                mvisual->SetColor(ChColor(0.3f, 0.6f, 0.7f));
                mbody->AddAsset(mvisual);
                // Attach a custom asset. It will hold electrical properties
                std::shared_ptr<ElectricParticleProperty> electric_asset(new ElectricParticleProperty);
                electric_asset->e_fraction = ElectricParticleProperty::e_fraction_sphere;
                electric_asset->e_material = ElectricParticleProperty::e_mat_plastic;
                electric_asset->conductivity = 0;
                electric_asset->birthdate = this->systemreference->GetChTime();
                ChVector<> Cradii;  // use equivalent-inertia ellipsoid to get characteristic size:
                ChVector<> Ine = mbody->GetInertiaXX();
                Cradii.x = sqrt((5. / (2.*mbody->GetMass()))*(Ine.y + Ine.z - Ine.x));
                Cradii.y = sqrt((5. / (2.*mbody->GetMass()))*(Ine.x + Ine.z - Ine.y));
                Cradii.z = sqrt((5. / (2.*mbody->GetMass()))*(Ine.x + Ine.y - Ine.z));
                electric_asset->Cdim = Cradii*2.;
                mbody->AddAsset(electric_asset);

                ++particlecounter;
                mbody->SetIdentifier(particlecounter);
            }
            // here put custom data of the callback
            ChSystem* systemreference;
            int particlecounter;
        };
        auto callback_plastic = std::make_shared<MyCreator_plastic>();
        callback_plastic->systemreference = &this->mphysicalSystem;
        callback_plastic->particlecounter = 0;
        mcreator_plastic->SetCallbackPostCreation(callback_plastic.get());


        // Create a parent ChRandomShapeCreator that 'mixes' the two generators above,
        // mixing them with a given percentual:
        auto mcreatorTot = std::make_shared<ChRandomShapeCreatorFromFamilies>();
        mcreatorTot->AddFamily(mcreator_metal, 0.4);	// 1st creator family, with percentual
        mcreatorTot->AddFamily(mcreator_plastic, 0.4);	// 2nd creator family, with percentual
        mcreatorTot->Setup();

        // Finally, tell to the emitter that it must use the 'mixer' above:
        emitter.SetParticleCreator(mcreatorTot);


        // ---Initialize the randomizer for velocities, with statistical distribution

        auto mvelo = std::make_shared<ChRandomParticleVelocityConstantDirection>();
        mvelo->SetDirection(-VECT_Y);
        mvelo->SetModulusDistribution(0.0);

        emitter.SetParticleVelocity(mvelo);
	}


		///
		/// Function that defines the forces on the debris ****ida
		///
	void apply_forces (	    ChSystem* msystem,		 // contains all bodies
							ChCoordsys<>& drum_csys, // pos and rotation of drum 
							double drumspeed,		 // speed of drum
							int totframes)		
	{
	    // Compute parameters on-the-fly (some parameters like L or U might have changed meanwhile..)
		h1 = (pow(L,2)+pow((drum_diameter/2),2)-((electrode_diameter/2),2))/(2*L); //analytical parameter****ida
		h2 = (pow(L,2)-pow((drum_diameter/2),2)+((electrode_diameter/2),2))/(2*L);//analytical parameter****ida
		j = sqrt(pow(h1,2)-pow((drum_diameter/2),2));//analytical parameter****ida
		f = U/log(((h1+j-(drum_diameter/2))*(h2+j-(electrode_diameter/2)))/((drum_diameter/2)+j-h1)*((electrode_diameter/2)+j-h2));//analytical parameter****ida

		// Loop on all bodies:
		for (unsigned int i=0; i<msystem->Get_bodylist()->size(); i++)
		{
			auto abody = (*msystem->Get_bodylist())[i];

			bool was_a_particle = false;
			std::shared_ptr<ElectricParticleProperty> electricproperties; // null by default

			// Fetch the ElectricParticleProperty asset from the list of 
			// assets that have been attached to the object, and retrieve the
			// custom data that have been stored. ***ALEX
			for (unsigned int na= 0; na< abody->GetAssets().size(); na++)
			{
				std::shared_ptr<ChAsset> myasset = abody->GetAssetN(na);
				if (auto myassetelectric = std::dynamic_pointer_cast<ElectricParticleProperty>(myasset))
				{
					// OK! THIS WAS A PARTICLE! ***ALEX
					was_a_particle = true;		
					electricproperties = myassetelectric;
				} 
			}

			// Do the computation of forces only on bodies that had 
			// the 'ElectricParticleProperty' attached.. **ALEX
			if(was_a_particle)
			{
				ChVector<> diam = electricproperties->Cdim; 
				double sigma =    electricproperties->conductivity;

				// Remember to reset 'user forces accumulators':
				abody->Empty_forces_accumulators();

				// initialize speed of air (steady, if outside fan stream): 
				ChVector<> abs_wind(0,0,0);

				// calculate the position of body COG with respect to the drum COG:
				ChVector<> mrelpos = drum_csys.TransformParentToLocal(abody->GetPos());
				double distx=mrelpos.x;
				double disty=mrelpos.y;
				ChVector<> velocity=abody->GetPos_dt();
				double velocityx=velocity.x;
				double velocityy=velocity.y;
				double velocityz=velocity.z;
				//ChVector <> rot_speed=abody->GetWvel_par();
				//double rot_speedz=rot_speed.z; //bisogna tirare fuori la componente attorno all'asse z globale della velocità di rotazione

				double velocity_norm_sq=velocity.Length2();

				//ChQuaternion<> rot_velocity=abody->GetRot_dt;
				
				// Polar coordinates of particles respect to the axis of the rotor, may be useful later **ALEX

				double distance = pow(distx*distx+disty*disty,0.5);
				double phi = atan2(disty,distx);
				double phi2 = atan2(-velocity.y,velocity.x);
				

			

				//
				// STOKES FORCES
				//


				double average_rad = 0.5* electricproperties->Cdim.Length(); // Approximate to sphere radius. Ida: this can be improved, by having Stokes forces for three Cdim x y z values maybe 

				ChVector<> StokesForce = electricproperties->StokesForce;
				
				electricproperties->StokesForce = (-6*CH_C_PI*eta*average_rad) * velocity;
		
				abody->Accumulate_force(StokesForce, abody->GetPos(), false);


	            

				//Calculating the analytical expressions of the electric field***ida

				double xuno=distx*cos(alpha)+ disty*sin(alpha);//analytical parameter****ida
				double yuno=disty*cos(alpha)- distx*sin(alpha);//analytical parameter****ida

				double Ex=(((j-h1+xuno)/(pow((j-h1+xuno),2)+pow(yuno,2))+((j+h1-xuno)/(pow((j+h1-xuno),2)+pow(yuno,2)))*f));//analytical expression of the electric field x direction***ida
				double Ey=((yuno/(pow((j-h1+xuno),2)+pow(yuno,2))-(yuno/(pow((j+h1-xuno),2)+pow(yuno,2)))*f));//analytical expression of the electric field y direction***ida
				double Ez=0;

				ChVector<> vE (Ex, Ey, Ez);
				double E = vE.Length();
				double Emax = -11.818*U-514.87;

	          
				
				//
				//===== METAL FORCES ==========
				//

				if (electricproperties->e_material == ElectricParticleProperty::e_mat_metal)
				{ 
					// charge the particle? (contact w. drum)
					if ((distx > 0) && (disty > 0))
					{
						if (electricproperties->chargeM == 0)
						{
						electricproperties->chargeM = (2./3.)*pow(CH_C_PI,3)*epsilon*pow(average_rad,2)*E;
						electricproperties->chargeM *= (1.0 - 0.3*ChRandom() );
						
						}
					}

					
					ChVector<> ElectricForce = electricproperties->ElectricForce;

					electricproperties->ElectricForce = 0.832 * electricproperties->chargeM * vE;
//GetLog() << "ElectricForce" << ElectricForce << "\n";
					// switch off electric forces if too out-of-plane
					if ((mrelpos.z > drum_width*0.5) || (mrelpos.z < -drum_width*0.5))
						ElectricForce = 0; 

					abody->Accumulate_force(ElectricForce, abody->GetPos(), false);


				} // end if material==metal


				//
				//===== PLASTIC FORCES ==========
				//

			    

				if (electricproperties->e_material == ElectricParticleProperty::e_mat_plastic) //forze sulle particelle non metalliche
				{
					

					// charge the particle? (contact w. drum)
					if ((distx > 0.04) && (disty > 0))
					{
						if (electricproperties->chargeP == 0)
						{
							electricproperties->chargeP = 3*CH_C_PI*epsilonO*pow(2*average_rad,2)*Emax*(epsilonR/(epsilonR+2)); // charge
							electricproperties->chargeP *= (1.0 - 0.3*ChRandom() );
						}
					} //15000000,750000,450000
					// discharge the particle? (contact w. blade)
					if (distx < -(drum_diameter*0.5 -0.009) && (disty > -(drum_diameter*0.5 + 0.009)) || sqrt(pow(distx,2)+ pow(disty,2))> (1.03*drum_diameter*0.5))
					{
						electricproperties->chargeP = 0; // charge
					}
					
					ChVector<> ElectricImageForce = electricproperties->ElectricImageForce;


					electricproperties->ElectricImageForce.x = -((pow( electricproperties->chargeP,2))/(4*CH_C_PI*epsilon*pow((2*average_rad),2))*cos(atan2(disty,distx)));
					electricproperties->ElectricImageForce.y = -((pow( electricproperties->chargeP,2))/(4*CH_C_PI*epsilon*pow((2*average_rad),2))*sin(atan2(disty,distx)));
					electricproperties->ElectricImageForce.z = 0;	
							
					
					// switch off electric forces if too out-of-plane
					if ((mrelpos.z > drum_width*0.5) || (mrelpos.z < -drum_width*0.5))
						ElectricImageForce = 0; 


					abody->Accumulate_force(ElectricImageForce, abody->GetPos(), false);
					

				   }  // end if material==plastic
				


				//ChVector<> DragForce;
				//DragForce.x = -CD*ro*velocity_norm_sq*CH_C_PI*diam.x*diam.y/2*cos(phi2);
				//DragForce.y = CD*ro*velocity_norm_sq*CH_C_PI*diam.x*diam.y/2*sin(phi2);
				//DragForce.z = 0;

				//abody->Accumulate_force( DragForce, abody->GetPos(), false);

				//ChVector<> LiftForce;
				//LiftForce.x = CL*ro*velocity_norm_sq*CH_C_PI*diam.x*diam.y/2*sin(phi2);
				//LiftForce.y = CL*ro*velocity_norm_sq*CH_C_PI*diam.x*diam.y/2*cos(phi2);
				//LiftForce.z = 0;	
			
				//abody->Accumulate_force(LiftForce, abody->GetPos(), false);
				

			} // end if(was_a_particle) , i.e. a body with electrical asset

		} // end for() loop on all bodies
	}

	

public:
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
    // simulator. These are initializad with constant values, but if loading the
    // SolidWorks model, they will be changed accordingly to what is found in the CAD 
    // file (see later, where the SolidWorks model is parsed). 

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









    ///	  
    /// Function that deletes old debris 
    /// (to avoid infinite creation that fills memory)
    ///
    void purge_debris(ChSystem& mysystem, double max_age = 5.0) const
    {
        for (unsigned int i = 0; i<mysystem.Get_bodylist()->size(); i++)
        {
            auto abody = (*mysystem.Get_bodylist())[i];

            bool to_delete = false;

            // Fetch the ElectricParticleProperty asset from the list of 
            // assets that have been attached to the object, and retrieve the
            // custom data that have been stored. ***ALEX
            for (unsigned int na = 0; na< abody->GetAssets().size(); na++)
            {
                std::shared_ptr<ChAsset> myasset = abody->GetAssetN(na);
                if (auto electricproperties = std::dynamic_pointer_cast<ElectricParticleProperty>(myasset))
                {
                    double particle_birthdate = electricproperties->birthdate;
                    double particle_age = mysystem.GetChTime() - particle_birthdate;
                    if (particle_age > max_age)
                    {
                        to_delete = true;
                    }
                }
            }

            if (to_delete)
            {
                mysystem.Remove(abody);

                i--; // this because if deleted, the rest of the array is shifted back one position..
            }
        }
    }


    ///
    /// Function for drawing forces
    ///
    void draw_forces(irrlicht::ChIrrApp& application, double scalefactor = 1.0) const
    {
        ChSystem* msystem = application.GetSystem();

        for (unsigned int i = 0; i<msystem->Get_bodylist()->size(); i++)
        {
            auto abody = (*msystem->Get_bodylist())[i];

            bool was_a_particle = false;

            // Fetch the ElectricParticleProperty asset from the list of 
            // assets that have been attached to the object, and retrieve the
            // custom data that have been stored. ***ALEX
            for (unsigned int na = 0; na< abody->GetAssets().size(); na++)
            {
                std::shared_ptr<ChAsset> myasset = abody->GetAssetN(na);
                if (auto electricproperties = std::dynamic_pointer_cast<ElectricParticleProperty>(myasset))
                {
                    // OK! THIS WAS A PARTICLE! ***ALEX
                    was_a_particle = true;
                }
            }

            // Do the computation of forces only on bodies that had 
            // the 'ElectricParticleProperty' attached.. **ALEX
            if (was_a_particle)
            {
                ChVector<> custom_force = abody->Get_accumulated_force();
                custom_force *= scalefactor;
                irrlicht::ChIrrTools::drawSegment(application.GetVideoDriver(),
                    abody->GetPos(),
                    abody->GetPos() + custom_force,
                    irr::video::SColor(255, 0, 0, 255));
            }
        }
    }

    ///
    /// Function to update trajectories. Must be
    /// called at each timestep
    ///
    void UpdateTrajectories(irrlicht::ChIrrApp& application) const
    {
        ChSystem* msystem = application.GetSystem();

        for (unsigned int i = 0; i<msystem->Get_bodylist()->size(); i++)
        {
            auto abody = (*msystem->Get_bodylist())[i];

            bool was_a_particle = false;

            // Fetch the ElectricParticleProperty asset from the list of 
            // assets that have been attached to the object, and retrieve the
            // custom data that have been stored. ***ALEX
            for (unsigned int na = 0; na< abody->GetAssets().size(); na++)
            {
                std::shared_ptr<ChAsset> myasset = abody->GetAssetN(na);
                if (auto trajectoryasset = std::dynamic_pointer_cast<ParticleTrajectory>(myasset))
                {
                    // OK! trajectory storage!	
                    trajectoryasset->positions.push_back(abody->GetPos());
                    trajectoryasset->speeds.push_back(abody->GetPos_dt());

                    // remove excessive amount of elements
                    while (trajectoryasset->positions.size() > trajectoryasset->max_points)
                        trajectoryasset->positions.pop_front();
                    while (trajectoryasset->speeds.size() > trajectoryasset->max_points)
                        trajectoryasset->speeds.pop_front();
                }
            }
        }
    }

    ///
    /// Function to draw trajectories. Must be
    /// called at each timestep
    ///
    void DrawTrajectories(irrlicht::ChIrrApp& application) const
    {
        ChSystem* msystem = application.GetSystem();

        for (unsigned int i = 0; i<msystem->Get_bodylist()->size(); i++)
        {
            auto abody = (*msystem->Get_bodylist())[i];

            ChVector<> pointA = abody->GetPos();

            bool was_a_particle = false;

            // Fetch the ElectricParticleProperty asset from the list of 
            // assets that have been attached to the object, and retrieve the
            // custom data that have been stored. ***ALEX
            for (unsigned int na = 0; na< abody->GetAssets().size(); na++)
            {
                std::shared_ptr<ChAsset> myasset = abody->GetAssetN(na);
                if (auto trajectoryasset = std::dynamic_pointer_cast<ParticleTrajectory>(myasset))
                {
                    int npoints = 0;
                    std::list< ChVector<> >::const_iterator iterator;
                    std::list< ChVector<> >::const_iterator iteratorspeed;
                    iteratorspeed = trajectoryasset->speeds.begin();
                    for (iterator = trajectoryasset->positions.begin(); iterator != trajectoryasset->positions.end(); ++iterator)
                    {
                        ChVector<> pointB = *iterator;
                        ChVector<> speed = *iteratorspeed;
                        if (npoints >0)
                        {
                            double scalarspeed = speed.Length();
                            double normalizedspeed = scalarspeed / 5.0;
                            irr::video::SColor mcol(255, (int)(255.*normalizedspeed), (int)(255.*normalizedspeed), (int)(255.*(1.0 - normalizedspeed)));
                            irrlicht::ChIrrTools::drawSegment(application.GetVideoDriver(),
                                pointA,
                                pointB,
                                mcol);
                        }
                        pointA = pointB;
                        ++npoints;
                        ++iteratorspeed;
                    }
                }
            }
        }
    }


    ///
    /// Main function of the simulator.
    /// Initialize the simulation, and
    /// Performs the simulation
    /// by running the loop of time integration
    ///
    int simulate()
    {

        // Create the Irrlicht visualization (open the Irrlicht device, 
        // bind a simple user interface, etc. etc.)
        irrlicht::ChIrrApp application(&mphysicalSystem, L"Conveyor belt", irr::core::dimension2d<irr::u32>(800, 600), false);

        // Change default font to something better
        //application.SetFonts("../objects/fonts/arial8.xml");

        // Easy shortcuts to add camera, lights, logo and sky in Irrlicht scene:
        application.AddTypicalLogo("../objects/");
        application.AddTypicalSky("../objects/skybox/");
        application.AddTypicalLights();
        application.AddTypicalCamera(irr::core::vector3df(1.5f, 0.4f, -1.0f), irr::core::vector3df(0.5f, 0.f, 0.f));
        if (irr_cast_shadows)
            application.AddLightWithShadow(irr::core::vector3df(-4.5f, 5.5f, 4.5f), irr::core::vector3df(0.f, 0.f, 0.f), 10, 1.2, 10.2, 30, 512, irr::video::SColorf(1.f, 0.9f, 0.9f));

        // This is for GUI tweaking of system parameters..
        UserInterfaceEventReceiver receiver(&application, this);
        // note how to add the custom event receiver to the default interface:
        application.SetUserEventReceiver(&receiver);



        // IMPORT A SOLIDWORK MODEL 

        // 1) create the Python engine. This is necessary in order to parse the files that 
        // have been saved using the SolidWorks add-in for Chrono::Engine.

        //ChPythonEngine my_python;

        // 2) loads the .py file (as saved from SolidWorks) and fill the system.
        //try
        //{
        //	my_python.ImportSolidWorksSystem(this->solidworks_py_modelfile.c_str(), mphysicalSystem);  // note, don't use the .py suffix in filename..
        //}
        //catch (ChException myerror)
        //{
        //	GetLog() << myerror.what();
        //}


        // 3) fetch coordinate values and objects from what was imported from CAD

        //ChCoordsys<> conveyor_csys = CSYSNORM;


        //std::shared_ptr<ChMarker> my_marker = mphysicalSystem.SearchMarker("centro_nastro");
        //if (!my_marker)
        //	GetLog() << "Error: cannot find centro_nastro marker from its name in the C::E system! \n";
        //else
        //	conveyor_csys = my_marker->GetAbsCoord();  // fetch both pos and rotation of CAD

        conveyor_csys = ChCoordsys<double>(ChVector<double>(0, 0, 0));

        //****Ida

        //my_marker = mphysicalSystem.SearchMarker("Splitter1");
        //if (!my_marker)
        //	GetLog() << "Error: cannot find Splitter1 marker from its name in the C::E system! \n";
        //else
        //	splitter1_csys = my_marker->GetAbsCoord();  // fetch both pos and rotation of CAD

        splitter1_csys = ChCoordsys<double>(ChVector<double>(0, 0, 0));

        //my_marker = mphysicalSystem.SearchMarker("Splitter2");
        //if (!my_marker)
        //	GetLog() << "Error: cannot find Splitter2 marker from its name in the C::E system! \n";
        //else
        //	splitter2_csys = my_marker->GetAbsCoord();  // fetch both pos and rotation of CAD

        splitter2_csys = ChCoordsys<double>(ChVector<double>(0, 0, 0));

        //my_marker = mphysicalSystem.SearchMarker("Spazzola");
        //if (!my_marker)
        //	GetLog() << "Error: cannot find Spazzola marker from its name in the C::E system! \n";
        //else
        //	brush_csys = my_marker->GetAbsCoord();  // fetch both pos and rotation of CAD

        brush_csys = ChCoordsys<double>(ChVector<double>(0, 0, 0));

        //***Ida


        //my_marker = mphysicalSystem.SearchMarker("centro_nozzle");
        //if (!my_marker)
        //	GetLog() << "Error: cannot find centro_nozzle marker from its name in the C::E system! \n";
        //else
        //	nozzle_csys = my_marker->GetAbsCoord();  // fetch both pos and rotation of CAD

        nozzle_csys = ChCoordsys<double>(ChVector<double>(0, 0, 0));

        emitter_positions->Outlet() = nozzle_csys;
        emitter_positions->Outlet().rot.Q_from_AngAxis(CH_C_PI_2, VECT_X); // rotate outlet 90° on x


                                                                           //my_marker = mphysicalSystem.SearchMarker("centro_cilindro");
                                                                           //if (!my_marker)
                                                                           //	GetLog() << "Error: cannot find centro_cilindro marker from its name in the C::E system! \n";
                                                                           //else
                                                                           //	drum_csys = my_marker->GetAbsCoord();  // fetch both pos and rotation of CAD

        drum_csys = ChCoordsys<double>(ChVector<double>(0, 0, 0));

        // fetch mrigidBodyDrum pointer! will be used for changing the friction, the collision family, and later to create the motor
        auto mrigidBodyDrum = std::dynamic_pointer_cast<ChBodyAuxRef>(mphysicalSystem.Search("drum-1"));
        if (!mrigidBodyDrum)
            GetLog() << "ERROR: cannot find drum-1 from its name in the C::E system! ! \n";
        else
        {
            mrigidBodyDrum->GetCollisionModel()->SetFamily(3);
            mrigidBodyDrum->GetCollisionModel()->SetFamilyMaskNoCollisionWithFamily(1);
            mrigidBodyDrum->GetCollisionModel()->SetFamilyMaskNoCollisionWithFamily(2);
            mrigidBodyDrum->GetMaterialSurface()->SetFriction(surface_drum_friction);
            mrigidBodyDrum->GetMaterialSurface()->SetRestitution(surface_drum_restitution);
            mrigidBodyDrum->GetMaterialSurface()->SetRollingFriction(surface_drum_rolling_friction);
            mrigidBodyDrum->GetMaterialSurface()->SetSpinningFriction(surface_drum_spinning_friction);
        }

        //***Ida

        auto mrigidBodySplitter1 = std::dynamic_pointer_cast<ChBodyAuxRef>(mphysicalSystem.Search("Splitter-10"));
        if (!mrigidBodySplitter1)
            GetLog() << "ERROR: cannot find Splitter-10 from its name in the C::E system! ! \n";
        else
        {
            mrigidBodySplitter1->SetBodyFixed(true);
            mrigidBodySplitter1->GetCollisionModel()->SetFamily(3); // rivedere 
            mrigidBodySplitter1->GetCollisionModel()->SetFamilyMaskNoCollisionWithFamily(1);// rivedere
            mrigidBodySplitter1->GetCollisionModel()->SetFamilyMaskNoCollisionWithFamily(2);// rivedere
            mrigidBodySplitter1->GetMaterialSurface()->SetFriction(0.1f);
            mrigidBodySplitter1->SetCollide(this->splitters_collide); // deactivate collision?
        }

        auto mrigidBodySplitter2 = std::dynamic_pointer_cast<ChBodyAuxRef>(mphysicalSystem.Search("Splitter2-1"));
        if (!mrigidBodySplitter2)
            GetLog() << "ERROR: cannot find Splitter2-1 from its name in the C::E system! ! \n";
        else
        {
            mrigidBodySplitter2->SetBodyFixed(true);
            mrigidBodySplitter2->GetCollisionModel()->SetFamily(3);// rivedere
            mrigidBodySplitter2->GetCollisionModel()->SetFamilyMaskNoCollisionWithFamily(1);// rivedere
            mrigidBodySplitter2->GetCollisionModel()->SetFamilyMaskNoCollisionWithFamily(2);// rivedere
            mrigidBodySplitter2->GetMaterialSurface()->SetFriction(0.1f);
            mrigidBodySplitter2->SetCollide(this->splitters_collide); // deactivate collision?
        }

        auto mrigidBodySpazzola = std::dynamic_pointer_cast<ChBodyAuxRef>(mphysicalSystem.Search("Spazzola-1"));
        if (!mrigidBodySpazzola)
            GetLog() << "ERROR: cannot find Spazzola-1 from its name in the C::E system! ! \n";
        else
        {
            mrigidBodySpazzola->GetCollisionModel()->SetFamily(1); // rivedere
            mrigidBodySpazzola->GetCollisionModel()->SetFamilyMaskNoCollisionWithFamily(2);// rivedere
            mrigidBodySpazzola->GetCollisionModel()->SetFamilyMaskNoCollisionWithFamily(3);// rivedere
            mrigidBodySpazzola->GetMaterialSurface()->SetFriction(0.9f);

        }

        auto mrigidBodyConveyor = std::dynamic_pointer_cast<ChBodyAuxRef>(mphysicalSystem.Search("conveyor-1"));
        if (!mrigidBodyConveyor)
            GetLog() << "ERROR: cannot find conveyor from its name in the C::E system! ! \n";
        else
        {
            mrigidBodyConveyor->GetCollisionModel()->SetFamily(2);
            mrigidBodyConveyor->GetCollisionModel()->SetFamilyMaskNoCollisionWithFamily(1);
            mrigidBodyConveyor->GetCollisionModel()->SetFamilyMaskNoCollisionWithFamily(3);
            mrigidBodyConveyor->GetMaterialSurface()->SetFriction(surface_plate_friction);
            mrigidBodyConveyor->GetMaterialSurface()->SetRestitution(surface_plate_restitution);
            mrigidBodyConveyor->GetMaterialSurface()->SetRollingFriction(surface_plate_rolling_friction);
            mrigidBodyConveyor->GetMaterialSurface()->SetSpinningFriction(surface_plate_spinning_friction);
        }




        //
        // Create a truss (absolute fixed reference body, for connecting the rotating cyl.)
        //

        std::shared_ptr<ChBody> mtruss(new ChBody);
        mtruss->SetBodyFixed(true);

        // Finally, do not forget to add the body to the system:
        application.GetSystem()->Add(mtruss);

        //**Ida

        std::shared_ptr<ChBody> mtruss2(new ChBody);
        mtruss2->SetBodyFixed(true);

        // Finally, do not forget to add the body to the system:
        application.GetSystem()->Add(mtruss2);



        // 
        // Create a motor constraint between the cylinder and the truss
        //

        std::shared_ptr<ChLinkEngine> mengine;

        if (mrigidBodyDrum)
        {
            mengine = std::make_shared<ChLinkEngine>();
            std::shared_ptr<ChBody> mdrum(mrigidBodyDrum);
            mengine->Initialize(mdrum, mtruss, drum_csys);

            mengine->Set_eng_mode(ChLinkEngine::ENG_MODE_SPEED);
            if (auto mfun = std::dynamic_pointer_cast<ChFunction_Const>(mengine->Get_spe_funct()))
                mfun->Set_yconst(-drumspeed_radss);  // angular speed in [rad/s]

                                                     // Finally, do not forget to add the body to the system:
            application.GetSystem()->Add(mengine);
        }

        //***Ida

        std::shared_ptr<ChLinkEngine> mengine2;

        if (mrigidBodySpazzola)
        {
            mengine2 = std::make_shared<ChLinkEngine>();
            std::shared_ptr<ChBody> mSpazzola(mrigidBodySpazzola);
            mengine2->Initialize(mSpazzola, mtruss2, brush_csys);

            mengine2->Set_eng_mode(ChLinkEngine::ENG_MODE_SPEED);
            if (auto mfun = std::dynamic_pointer_cast<ChFunction_Const>(mengine2->Get_spe_funct()))
                mfun->Set_yconst(-drumspeed_radss); // angular speed in [rad/s]

                                                    // Finally, do not forget to add the body to the system:
            application.GetSystem()->Add(mengine2);


        }


        //
        // Create an (optional) exporter to POVray 
        // 

        postprocess::ChPovRay pov_exporter = postprocess::ChPovRay(&mphysicalSystem);

        if (save_POV_screenshots)
        {
            // Sets some file names for in-out processes.
            pov_exporter.SetTemplateFile("../objects/_template_POV.pov");
            pov_exporter.SetOutputScriptFile("rendering_frames.pov");

            // save the .dat files and the .bmp files
            // in two subdirectories, to avoid cluttering the current directory...
            ChFileutils::MakeDirectory("outputPOV");
            ChFileutils::MakeDirectory("animPOV");

            pov_exporter.SetOutputDataFilebase("outputPOV/my_state");
            pov_exporter.SetPictureFilebase("animPOV/picture");

            // optional: modify the POV default light
            pov_exporter.SetLight(ChVector<>(0.5f, 0.75f, 0.0f), ChColor(0.1f, 0.1f, 0.1f), true);

            pov_exporter.SetCamera(ChVector<>(0.5f, 0.75f, 0.5f), ChVector<>(0.2f, 0.6f, 0.f), 30, false);

            // optional: use SetCustomPOVcommandsScript() to add further POV commands,
            // ex. create an additional light, and an additional grid, etc. 
            // Remember the "\" char per each newline.

            pov_exporter.SetCustomPOVcommandsScript(" \
				light_source {   \
				  <0.5, 0.8, 0.2>  \
				  color rgb<1.7,1.7,1.7> \
				  area_light <0.4, 0, 0>, <0, 0, 0.4>, 5, 5 \
				  adaptive 1 \
				  jitter\
				} \
				//object{ Grid(0.5,0.01, rgb<0.9,0.9,0.9>, rgbt<1,1,1,1>) rotate <90, 0, 0>  } \
				//object{ Grid(0.1,0.04, rgb<1.5,1.5,1.5>, rgbt<1,1,1,1>) rotate <90, 0, 0> translate 0.001*z} \
			");

            // IMPORTANT! Tell to the POVray exporter that 
            // he must take care of converting the shapes of
            // all items (that have been added so far)!
            pov_exporter.AddAll();

            // IMPORTANT! Create the two .pov and .ini files for POV-Ray (this must be done
            // only once at the beginning of the simulation).
            pov_exporter.ExportScript();

        }


        //
        // For enabling Irrlicht visualization of assets (that have been added so far)
        //

        application.AssetBindAll();
        application.AssetUpdateAll();
        if (irr_cast_shadows)
            application.AddShadowAll();

        //
        // What to do by default on ALL newly created particles? 
        // A callback executed at each particle creation can be attached to the emitter..
        // 

        // a- define a class that implement your custom PostCreation method...
        class MyCreatorForAll : public ChCallbackPostCreation
        {
        public:
            virtual ~MyCreatorForAll()
            {
            }

            void PostCreation(std::shared_ptr<ChBody> mbody, ChCoordsys<> mcoords, ChRandomShapeCreator& mcreator) override
            {
                // Set the friction properties (using a shared ChSurfaceMaterial
                mbody->SetMaterialSurface(asurface_material);

                // Attach an asset to show trajectories
                //std::shared_ptr<ParticleTrajectory> massettraj(new ParticleTrajectory);
                //mbody->AddAsset(massettraj);

                // Enable Irrlicht visualization for all particles
                airrlicht_application->AssetBind(mbody);
                airrlicht_application->AssetUpdate(mbody);

                // Enable POV visualization
                if (apov_exporter)
                    apov_exporter->Add(mbody);

                // Disable gyroscopic forces for increased integrator stabilty
                mbody->SetNoGyroTorque(true);
            }

            irrlicht::ChIrrApp* airrlicht_application;
            postprocess::ChPovRay* apov_exporter;
            std::shared_ptr<ChMaterialSurface> asurface_material;
        };
        // b- create the callback object...
        MyCreatorForAll* mcreation_callback = new MyCreatorForAll;
        // c- set callback own data that he might need...
        mcreation_callback->airrlicht_application = &application;
        mcreation_callback->asurface_material = this->surface_particles;
        if (this->save_POV_screenshots)
            mcreation_callback->apov_exporter = &pov_exporter;
        else
            mcreation_callback->apov_exporter = nullptr;
        // d- attach the callback to the emitter!
        emitter.SetCallbackPostCreation(mcreation_callback);



        // 
        // PROCESS THE FLOW with these tools:
        // 


        // Create also a ChParticleProcessor configured as a
        // counter of particles that flow into a rectangle with a statistical distribution to plot:
        //  -create the trigger:
        double flowmeter_length = this->flowmeter_xmax - this->flowmeter_xmin;
        auto distrrectangle = std::make_shared<ChParticleEventFlowInRectangle>(flowmeter_length, flowmeter_width);
        distrrectangle->rectangle_csys = ChCoordsys<>(
            drum_csys.pos + ChVector<>(this->flowmeter_xmin + 0.5*flowmeter_length,
                this->flowmeter_y,
                0), // position of center rectangle
            Q_from_AngAxis(-CH_C_PI_2, VECT_X)); // rotate rectangle so that its Z is up
        distrrectangle->margin = 0.05;
        //  -create the counter, with 20x10 resolution of sampling, on x y
        //    This is defined in ProcessFlow.h and distinguishes plastic from metal
        auto countdistribution = std::make_shared<ProcessFlow>(this->flowmeter_bins, 1);
        //  -create the processor and plug in the trigger and the counter:
        ChParticleProcessor processor_distribution;
        processor_distribution.SetEventTrigger(distrrectangle);
        processor_distribution.SetParticleEventProcessor(countdistribution);


        // Create a remover, i.e. an object that takes care 
        // of removing particles that are inside or outside some volume.
        // The fact that particles are handled with shared pointers means that,
        // after they are removed from the ChSystem, they are also automatically
        // deleted if no one else is referencing them.
        auto distrrectangle2 = std::make_shared<ChParticleEventFlowInRectangle>(0.20, 0.30);
        distrrectangle2->rectangle_csys = distrrectangle->rectangle_csys;
        distrrectangle2->margin = 0.05;
        std::shared_ptr<ChParticleProcessEventRemove> removal_event(new ChParticleProcessEventRemove);
        ChParticleProcessor processor_remover;
        processor_remover.SetEventTrigger(distrrectangle2);
        processor_remover.SetParticleEventProcessor(removal_event);


        // 
        // THE SOFT-REAL-TIME CYCLE
        //

        application.SetStepManage(true);
        application.SetTimestep(this->timestep);

        application.GetSystem()->SetIntegrationType(ChSystem::INT_ANITESCU);
        application.GetSystem()->SetSolverType(ChSystem::SOLVER_SOR_MULTITHREAD);// SOLVER_SOR_MULTITHREAD or SOLVER_BARZILAIBORWEIN for max precision

        application.GetSystem()->Set_G_acc(ChVector<>(0, -9.81, 0));

        int savenum = 0;

        ChFileutils::MakeDirectory("screenshots");
        ChFileutils::MakeDirectory("output");

        application.GetSystem()->ShowHierarchy(GetLog());


        while (application.GetDevice()->run())
        {
            if (mphysicalSystem.GetChTime() > this->Tmax)
                break;

            application.GetVideoDriver()->beginScene(true, true, irr::video::SColor(255, 140, 161, 192));

            application.DrawAll();

            application.DoStep();

            if (!application.GetPaused())
            {

                totframes++;

                // Apply the forces caused by electrodes of the CES machine:

                apply_forces(&mphysicalSystem,		// contains all bodies
                    drum_csys,		 // pos and rotation of axis of drum (not rotating reference!)
                    drumspeed_radss, // speed of drum
                    totframes);



                if (receiver.checkbox_plotforces->isChecked())
                    draw_forces(application, 1000);

                if (receiver.checkbox_plottrajectories->isChecked())
                    DrawTrajectories(application);


                // Continuosly create debris that fall on the conveyor belt
                this->emitter.EmitParticles(mphysicalSystem, application.GetTimestep()); //***TEST***

                GetLog() << "Total mass=" << this->emitter.GetTotCreatedMass() << "   "
                    << "Total n.part=" << this->emitter.GetTotCreatedParticles() << "   "
                    << "Average kg/s=" << this->emitter.GetTotCreatedMass() / application.GetSystem()->GetChTime() << "\n";


                // Limit the max age (in seconds) of debris particles on the scene, 
                // deleting the oldest ones, for performance
                purge_debris(*application.GetSystem(), this->max_particle_age);


                // Use the processor to count particle flow in the rectangle section:
                processor_distribution.ProcessParticles(mphysicalSystem);

                // Continuosly check if some particle must be removed:
                processor_remover.ProcessParticles(mphysicalSystem);


                // Maybe the user played with the slider and changed the speed of drum...
                if (mengine)
                    if (auto mfun = std::dynamic_pointer_cast<ChFunction_Const>(mengine->Get_spe_funct()))
                        mfun->Set_yconst(-drumspeed_radss);  // angular speed in [rad/s]

                                                             // update the assets containing the trajectories, if any
                if (receiver.checkbox_plottrajectories->isChecked())
                    if (totframes % 20 == 0)
                        UpdateTrajectories(application);

                // Save data on file (each n integration steps, to avoid filling
                // the hard disk and to improve performance)
                if (totframes % saveEachNframes == 0)
                {
                    savenum++;

                    // Save log file as '.txt' files?

                    if (save_dataset == true)
                    {
                        char buffer[120];
                        sprintf(buffer, "output/esempio_output%05d.txt", savenum);
                        GetLog() << "\n saving dataset: " << buffer;
                        ChStreamOutAsciiFile file_for_output(buffer);
                        for (unsigned int i = 0; i<mphysicalSystem.Get_bodylist()->size(); i++)
                        {
                            auto abody = (*mphysicalSystem.Get_bodylist())[i];

                            // Fetch the ElectricParticleProperty asset from the list
                            for (unsigned int na = 0; na< abody->GetAssets().size(); na++)
                            {
                                std::shared_ptr<ChAsset> myasset = abody->GetAssetN(na);

                                if (auto electricproperties = std::dynamic_pointer_cast<ElectricParticleProperty>(myasset))
                                {
                                    // ok, its a particle!

                                    //double my_cond  = electricproperties->conductivity ;
                                    auto my_ElectricForce = electricproperties->ElectricForce;
                                    auto my_ElectricImageForce = electricproperties->ElectricImageForce;
                                    auto my_StokesForce = electricproperties->StokesForce;
                                    double rad = ((abody->GetMass()) * 3) / ((abody->GetDensity()) * 4 * CH_C_PI);
                                    ElectricParticleProperty::fraction_type fraction_identifier = electricproperties->e_fraction; // id will be 0=box, 1=cylinder, 2=sphere, 3=hull, 4=shavings, etc. (see enum)
                                    ElectricParticleProperty::material_type material_identifier = electricproperties->e_material; // id will be 0=plastic, 1=metal, 2=others (see enum)

                                                                                                                                  // Save on disk some infos...
                                    file_for_output << abody->GetIdentifier() << ", "
                                        << fraction_identifier << ", "
                                        << abody->GetPos().x << ", "
                                        << abody->GetPos().y << ", "
                                        << abody->GetPos().z << ", "
                                        << abody->GetDensity() << ", "
                                        //<< my_cond << ", "
                                        << abody->GetMass() << ", "
                                        << pow(rad, 1.0 / 3) << "\n";
                                    //<< abody->GetPos_dt().x << ", "
                                    //<< abody->GetPos_dt().y << ", "
                                    //<< abody->GetPos_dt().z << ", "
                                    //<< my_StokesForce << ", "
                                    //<< my_ElectricImageForce << ", "
                                    //<< my_ElectricForce << "\n";


                                }
                            }
                        }
                    }

                    // Save Irrlicht screenshots?

                    if (save_irrlicht_screenshots == true)
                    {
                        irr::video::IImage* image = application.GetVideoDriver()->createScreenShot();
                        char buffer[120];
                        sprintf(buffer, "screenshots/screenshot%05d.bmp", savenum);
                        GetLog() << "\n saving screenshot: " << buffer;
                        if (image)
                            application.GetVideoDriver()->writeImageToFile(image, buffer);
                        image->drop();
                    }

                    // Save POV screenshots?

                    if (save_POV_screenshots)
                    {
                        pov_exporter.ExportData();
                        GetLog() << "\n saving POV data n." << savenum;
                    }

                } // end saving code



            }

            // Just for fun, plot the distribution matrices, 
            // i.e. countdistribution->mmass_plastic etc.
            // In this case, normalize to integral , and scale on Z
            double yscalefactor_plastic;
            double totmass_plastic = 0;
            for (int ir = 0; ir< countdistribution->mmass_plastic.GetRows(); ++ir)
                for (int ic = 0; ic< countdistribution->mmass_plastic.GetColumns(); ++ic)
                    totmass_plastic += countdistribution->mmass_plastic(ir, ic);
            if (totmass_plastic == 0)
                yscalefactor_plastic = 0; // if not yet particle passed through sampling rectangle
            else
                yscalefactor_plastic = (0.002 * countdistribution->mmass_plastic.GetRows()*countdistribution->mmass_plastic.GetColumns()) / totmass_plastic;

            drawDistribution(application.GetVideoDriver(),
                countdistribution->mmass_plastic * yscalefactor_plastic,
                distrrectangle->rectangle_csys,
                distrrectangle->Xsize,
                distrrectangle->Ysize,
                irr::video::SColor(255, 255, 0, 0));

            double yscalefactor_metal;
            double totmass_metal = 0;
            for (int ir = 0; ir< countdistribution->mmass_metal.GetRows(); ++ir)
                for (int ic = 0; ic< countdistribution->mmass_metal.GetColumns(); ++ic)
                    totmass_metal += countdistribution->mmass_metal(ir, ic);
            if (totmass_plastic == 0)
                yscalefactor_metal = 0; // if not yet particle passed through sampling rectangle
            else
                yscalefactor_metal = (0.002 * countdistribution->mmass_metal.GetRows()*countdistribution->mmass_metal.GetColumns()) / totmass_metal;

            drawDistribution(application.GetVideoDriver(),
                countdistribution->mmass_metal * yscalefactor_metal,
                distrrectangle->rectangle_csys,
                distrrectangle->Xsize,
                distrrectangle->Ysize,
                irr::video::SColor(255, 0, 255, 255));


            application.GetVideoDriver()->endScene();

        }

        // At the end ot the T max simulation time, 
        // save output distributions to disk (non normalized for unit area/volume), 
        // they can be a nxm matrix of 2d bins or a n-vector of 1d bins
        GetLog() << "\n saving output distributions... \n ";

        ChStreamOutAsciiFile file_for_metal("out_distribution_metal.txt");
        countdistribution->mmass_metal.StreamOUTdenseMatlabFormat(file_for_metal);
        ChStreamOutAsciiFile file_for_plastic("out_distribution_plastic.txt");
        countdistribution->mmass_plastic.StreamOUTdenseMatlabFormat(file_for_plastic);



        GetLog() << "\n Simulation Terminated. \n ";

        return 0;
    }




};








class WasteSeparator
{




}; // end of class




#endif

