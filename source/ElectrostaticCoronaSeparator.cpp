#include "ElectrostaticCoronaSeparator.h"

ElectrostaticCoronaSeparator::ElectrostaticCoronaSeparator()
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
    collision::ChCollisionModel::SetDefaultSuggestedEnvelope(0.001); //0.002
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
    emitter_positions->OutletWidth() = 0.1; // default x outlet size, from CAD;
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
            Cradii.x = sqrt((5. / (2. * mbody->GetMass())) * (Ine.y + Ine.z - Ine.x));
            Cradii.y = sqrt((5. / (2. * mbody->GetMass())) * (Ine.x + Ine.z - Ine.y));
            Cradii.z = sqrt((5. / (2. * mbody->GetMass())) * (Ine.x + Ine.y - Ine.z));
            electric_asset->Cdim = Cradii * 2.;
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
            ChVector<> Cradii; // use equivalent-inertia ellipsoid to get characteristic size:
            ChVector<> Ine = mbody->GetInertiaXX();
            Cradii.x = sqrt((5. / (2. * mbody->GetMass())) * (Ine.y + Ine.z - Ine.x));
            Cradii.y = sqrt((5. / (2. * mbody->GetMass())) * (Ine.x + Ine.z - Ine.y));
            Cradii.z = sqrt((5. / (2. * mbody->GetMass())) * (Ine.x + Ine.y - Ine.z));
            electric_asset->Cdim = Cradii * 2.;
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
    mcreatorTot->AddFamily(mcreator_metal, 0.4); // 1st creator family, with percentual
    mcreatorTot->AddFamily(mcreator_plastic, 0.4); // 2nd creator family, with percentual
    mcreatorTot->Setup();

    // Finally, tell to the emitter that it must use the 'mixer' above:
    emitter.SetParticleCreator(mcreatorTot);


    // ---Initialize the randomizer for velocities, with statistical distribution

    auto mvelo = std::make_shared<ChRandomParticleVelocityConstantDirection>();
    mvelo->SetDirection(-VECT_Y);
    mvelo->SetModulusDistribution(0.0);

    emitter.SetParticleVelocity(mvelo);
}

void ElectrostaticCoronaSeparator::apply_forces(ChSystem* msystem, ChCoordsys<>& drum_csys, double drumspeed, int totframes)
{
    // Compute parameters on-the-fly (some parameters like L or U might have changed meanwhile..)
    h1 = (pow(L, 2) + pow((drum_diameter / 2), 2) - ((electrode_diameter / 2) , 2)) / (2 * L); //analytical parameter****ida
    h2 = (pow(L, 2) - pow((drum_diameter / 2), 2) + ((electrode_diameter / 2) , 2)) / (2 * L);//analytical parameter****ida
    j = sqrt(pow(h1, 2) - pow((drum_diameter / 2), 2));//analytical parameter****ida
    f = U / log(((h1 + j - (drum_diameter / 2)) * (h2 + j - (electrode_diameter / 2))) / ((drum_diameter / 2) + j - h1) * ((electrode_diameter / 2) + j - h2));//analytical parameter****ida

    // Loop on all bodies:
    for (unsigned int i = 0; i < msystem->Get_bodylist()->size(); i++)
    {
        auto abody = (*msystem->Get_bodylist())[i];

        bool was_a_particle = false;
        std::shared_ptr<ElectricParticleProperty> electricproperties; // null by default

        // Fetch the ElectricParticleProperty asset from the list of 
        // assets that have been attached to the object, and retrieve the
        // custom data that have been stored. ***ALEX
        for (unsigned int na = 0; na < abody->GetAssets().size(); na++)
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
        if (was_a_particle)
        {
            ChVector<> diam = electricproperties->Cdim;
            double sigma = electricproperties->conductivity;

            // Remember to reset 'user forces accumulators':
            abody->Empty_forces_accumulators();

            // initialize speed of air (steady, if outside fan stream): 
            ChVector<> abs_wind(0, 0, 0);

            // calculate the position of body COG with respect to the drum COG:
            ChVector<> mrelpos = drum_csys.TransformParentToLocal(abody->GetPos());
            double distx = mrelpos.x;
            double disty = mrelpos.y;
            ChVector<> velocity = abody->GetPos_dt();
            double velocityx = velocity.x;
            double velocityy = velocity.y;
            double velocityz = velocity.z;
            //ChVector <> rot_speed=abody->GetWvel_par();
            //double rot_speedz=rot_speed.z; //bisogna tirare fuori la componente attorno all'asse z globale della velocità di rotazione

            double velocity_norm_sq = velocity.Length2();

            //ChQuaternion<> rot_velocity=abody->GetRot_dt;

            // Polar coordinates of particles respect to the axis of the rotor, may be useful later **ALEX

            double distance = pow(distx * distx + disty * disty, 0.5);
            double phi = atan2(disty, distx);
            double phi2 = atan2(-velocity.y, velocity.x);


            //
            // STOKES FORCES
            //


            double average_rad = 0.5 * electricproperties->Cdim.Length(); // Approximate to sphere radius. Ida: this can be improved, by having Stokes forces for three Cdim x y z values maybe 

            ChVector<> StokesForce = electricproperties->StokesForce;

            electricproperties->StokesForce = (-6 * CH_C_PI * eta * average_rad) * velocity;

            abody->Accumulate_force(StokesForce, abody->GetPos(), false);


            //Calculating the analytical expressions of the electric field***ida

            double xuno = distx * cos(alpha) + disty * sin(alpha);//analytical parameter****ida
            double yuno = disty * cos(alpha) - distx * sin(alpha);//analytical parameter****ida

            double Ex = (((j - h1 + xuno) / (pow((j - h1 + xuno), 2) + pow(yuno, 2)) + ((j + h1 - xuno) / (pow((j + h1 - xuno), 2) + pow(yuno, 2))) * f));//analytical expression of the electric field x direction***ida
            double Ey = ((yuno / (pow((j - h1 + xuno), 2) + pow(yuno, 2)) - (yuno / (pow((j + h1 - xuno), 2) + pow(yuno, 2))) * f));//analytical expression of the electric field y direction***ida
            double Ez = 0;

            ChVector<> vE(Ex, Ey, Ez);
            double E = vE.Length();
            double Emax = -11.818 * U - 514.87;


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
                        electricproperties->chargeM = (2. / 3.) * pow(CH_C_PI, 3) * epsilon * pow(average_rad, 2) * E;
                        electricproperties->chargeM *= (1.0 - 0.3 * ChRandom());
                    }
                }


                ChVector<> ElectricForce = electricproperties->ElectricForce;

                electricproperties->ElectricForce = 0.832 * electricproperties->chargeM * vE;
                //GetLog() << "ElectricForce" << ElectricForce << "\n";
                // switch off electric forces if too out-of-plane
                if ((mrelpos.z > drum_width * 0.5) || (mrelpos.z < -drum_width * 0.5))
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
                        electricproperties->chargeP = 3 * CH_C_PI * epsilonO * pow(2 * average_rad, 2) * Emax * (epsilonR / (epsilonR + 2)); // charge
                        electricproperties->chargeP *= (1.0 - 0.3 * ChRandom());
                    }
                } //15000000,750000,450000
                // discharge the particle? (contact w. blade)
                if (distx < -(drum_diameter * 0.5 - 0.009) && (disty > -(drum_diameter * 0.5 + 0.009)) || sqrt(pow(distx, 2) + pow(disty, 2)) > (1.03 * drum_diameter * 0.5))
                {
                    electricproperties->chargeP = 0; // charge
                }

                ChVector<> ElectricImageForce = electricproperties->ElectricImageForce;


                electricproperties->ElectricImageForce.x = -((pow(electricproperties->chargeP, 2)) / (4 * CH_C_PI * epsilon * pow((2 * average_rad), 2)) * cos(atan2(disty, distx)));
                electricproperties->ElectricImageForce.y = -((pow(electricproperties->chargeP, 2)) / (4 * CH_C_PI * epsilon * pow((2 * average_rad), 2)) * sin(atan2(disty, distx)));
                electricproperties->ElectricImageForce.z = 0;


                // switch off electric forces if too out-of-plane
                if ((mrelpos.z > drum_width * 0.5) || (mrelpos.z < -drum_width * 0.5))
                    ElectricImageForce = 0;


                abody->Accumulate_force(ElectricImageForce, abody->GetPos(), false);
            } // end if material==plastic


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

bool ElectrostaticCoronaSeparator::parse_particlescan(const char* filename)
{
    std::ifstream particlefile;
    particlefile.open(filename, std::ifstream::in);

    std::cout << "File open? " << particlefile.is_open() << std::endl;


    std::string line;
    while (std::getline(particlefile, line))
    {
        std::istringstream iss(line);
        std::string field;
        size_t mat_ID, particle_ID, convex_hull_Npoint;
        double thickness, particle_mass, I1, I2, I3;
        std::vector<double> CHx;
        std::vector<double> CHy;

        try
        {
            getline(iss, field, ',');
            static_cast<std::stringstream>(field) >> mat_ID;
            getline(iss, field, ',');
            static_cast<std::stringstream>(field) >> particle_ID;
            getline(iss, field, ',');
            static_cast<std::stringstream>(field) >> thickness;
            getline(iss, field, ',');
            static_cast<std::stringstream>(field) >> particle_mass;
            getline(iss, field, ',');
            static_cast<std::stringstream>(field) >> I1;
            getline(iss, field, ',');
            static_cast<std::stringstream>(field) >> I2;
            getline(iss, field, ',');
            static_cast<std::stringstream>(field) >> I3;
            getline(iss, field, ',');
            static_cast<std::stringstream>(field) >> convex_hull_Npoint;
            CHx.resize(convex_hull_Npoint);
            CHy.resize(convex_hull_Npoint);

            //std::cout << std::endl;
            //std::cout << "ParticleID: " << particle_ID << std::endl;
            //std::cout << "MaterialID: " << mat_ID << std::endl;
            //std::cout << "Thickness: " << thickness << std::endl;
            //std::cout << "Mass: " << particle_mass << std::endl;
            //std::cout << "Inertia " << I1 << ", " << I2 << ", " << I3 << ", " << std::endl;

            for (auto CH_sel = 0; CH_sel < convex_hull_Npoint; CH_sel++)
            {
                getline(iss, field, ',');
                static_cast<std::stringstream>(field) >> CHx[CH_sel];
                getline(iss, field, ',');
                static_cast<std::stringstream>(field) >> CHy[CH_sel];
                //std::cout << "CH coord: " << CHx[CH_sel] << ", " << CHy[CH_sel] << std::endl;
            }

            scanned_particles.push_back(std::make_shared<ChBody>());
        }
        catch (...)
        {
            std::cout << "Misformatted input file: " << "filename" << std::endl;
            return false;
        }
    }

    return true;
}

void ElectrostaticCoronaSeparator::create_debris_particlescan(double dt, double particles_second, ChSystem& mysystem, irrlicht::ChIrrApp* irr_application)
{
    auto mmaterial = std::make_shared<ChMaterialSurface>();
    mmaterial->SetFriction(0.2f);

    auto mrigidBody = std::make_shared<ChBody>();
}

void ElectrostaticCoronaSeparator::create_debris_particlescan_temp(double dt, double particles_second, ChSystem& mysystem, irrlicht::ChIrrApp* irr_application)
{
    double sph_fraction = 0;//0.3; // 30% cubes
    double box_fraction = 0;//0.4; // 40% cylinders
    double cyl_fraction = 1 - box_fraction - sph_fraction;

    double density = 1820;
    double sphrad = 0.01;
    double cylhei = 0.035;
    double cylmass = 0.0095; //CH_C_PI*pow(sphrad,2)*cylhei*density;
    double sphmass = 0.0095;//1.3333*CH_C_PI*pow(sphrad,3)*density;
    double sphinertia = 0.4 * pow(sphrad, 2) * sphmass;
    double cylinertia = 0.0833 * (pow(cylhei, 2) + 3 * pow(sphrad, 2)) * cylmass;//0.0833*(pow(cylhei,2)+3*pow(sphrad,2))*cylmass;
    double cylinertia2 = 0.5 * pow(sphrad, 2) * cylmass; //0.5*pow(sphrad,2)*cylmass;

    float conductivity;

    double exact_particles_dt = dt * particles_second;
    double particles_dt = floor(exact_particles_dt);
    double remaind = exact_particles_dt - particles_dt;

    if (remaind > ChRandom()) particles_dt += 1;

    for (int i = 0; i < particles_dt; i++)
    {
        std::shared_ptr<ChBody> created_body;

        double rand_fract = ChRandom();
        double rand_mat = ChRandom();
        double plastic_fract = 0.3;
        if (rand_mat < plastic_fract)
        {
            conductivity = 0;
        }
        else
        {
            conductivity = 6670000;
        }

        if (rand_fract < sph_fraction)
        {
            auto mmaterial = std::make_shared<ChMaterialSurface>();
            mmaterial->SetFriction(0.2f);
            //mmaterial->SetDampingF(0.2f);
            //mmaterial->SetImpactC(0.8f);

            // Create a body
            std::shared_ptr<ChBody> mrigidBody;

            mrigidBody->SetPos(ChVector<>(-0.5 * xnozzlesize + ChRandom() * xnozzlesize + xnozzle, conv_thick / 2 + sphrad, -0.5 * znozzlesize + ChRandom() * znozzlesize));
            mrigidBody->SetMass(sphmass);
            mrigidBody->SetInertiaXX(ChVector<>(sphinertia, sphinertia, sphinertia));
            mrigidBody->SetMaterialSurface(mmaterial);
            //mrigidBody->GetCollisionModel()->SetFamily(5);
            //mrigidBody->GetCollisionModel()->SetFamilyMaskNoCollisionWithFamily(3);

            // Define a collision shape 
            mrigidBody->GetCollisionModel()->ClearModel();
            mrigidBody->GetCollisionModel()->AddSphere(sphrad);
            mrigidBody->GetCollisionModel()->BuildModel();
            mrigidBody->SetCollide(true);

            // Attach a visualization shape asset. 
            auto msphere = std::make_shared<ChSphereShape>();
            msphere->GetSphereGeometry().rad = sphrad;
            msphere->GetSphereGeometry().center = ChVector<>(0, 0, 0);
            mrigidBody->AddAsset(msphere);
            // Attach a visualization texture
            std::shared_ptr<ChTexture> mtexture(new ChTexture);
            mtexture->SetTextureFilename("../objects/bluwhite.png");
            mrigidBody->AddAsset(mtexture);

            // Attach a custom asset. Look how to create and add a custom asset to the object! ***ALEX
            std::shared_ptr<ElectricParticleProperty> electric_asset(new ElectricParticleProperty);
            electric_asset->Cdim = ChVector<>(2 * sphrad, 2 * sphrad, 2 * sphrad);
            electric_asset->conductivity = conductivity;
            electric_asset->e_fraction = ElectricParticleProperty::e_fraction_sphere;
            electric_asset->birthdate = mysystem.GetChTime();
            mrigidBody->AddAsset(electric_asset);

            // Finally, do not forget to add the body to the system:
            mysystem.Add(mrigidBody);

            // If Irrlicht is used, setup also the visualization proxy:
            if (irr_application)
            {
                irr_application->AssetBind(mrigidBody);
                irr_application->AssetUpdate(mrigidBody);
            }
            created_body = mrigidBody;
        }

        if ((rand_fract > sph_fraction) &&
            (rand_fract < box_fraction + sph_fraction))
        {
            double xscale = 1.3 * (1 - 0.8 * ChRandom()); // for oddly-shaped boxes..
            double yscale = 1.3 * (1 - 0.8 * ChRandom());
            double zscale = 1.3 * (1 - 0.8 * ChRandom());

            //	ChCollisionModel::SetDefaultSuggestedEnvelope(0.002);
            //	ChCollisionModel::SetDefaultSuggestedMargin  (0.002);

            auto mmaterial = std::make_shared<ChMaterialSurface>();
            mmaterial->SetFriction(0.4f);
            //mmaterial->SetImpactC(0.0f);

            // Create a body
            std::shared_ptr<ChBody> mrigidBody;

            mrigidBody->SetPos(ChVector<>(-0.5 * xnozzlesize + ChRandom() * xnozzlesize + xnozzle, ynozzle + i * 0.005, -0.5 * znozzlesize + ChRandom() * znozzlesize));
            mrigidBody->SetMass(sphmass);
            mrigidBody->SetInertiaXX(ChVector<>(sphinertia, sphinertia, sphinertia));
            mrigidBody->SetMaterialSurface(mmaterial);

            // Define a collision shape 
            mrigidBody->GetCollisionModel()->ClearModel();
            mrigidBody->GetCollisionModel()->AddBox(sphrad * 2 * xscale, sphrad * 2 * yscale, sphrad * 2 * yscale);
            mrigidBody->GetCollisionModel()->BuildModel();
            mrigidBody->SetCollide(true);

            // Attach a visualization shape asset. 
            std::shared_ptr<ChBoxShape> mbox;
            mbox->GetBoxGeometry().SetLengths(ChVector<>(sphrad * 2 * xscale, sphrad * 2 * yscale, sphrad * 2 * yscale));
            mrigidBody->AddAsset(mbox);
            // Attach a visualization texture
            std::shared_ptr<ChTexture> mtexture;
            mtexture->SetTextureFilename("../objects/bluwhite.png");
            mrigidBody->AddAsset(mtexture);

            // Attach a custom asset. Look how to create and add a custom asset to the object! ***ALEX
            std::shared_ptr<ElectricParticleProperty> electric_asset(new ElectricParticleProperty);
            electric_asset->Cdim = ChVector<>(2 * sphrad, 2 * sphrad, 2 * sphrad);
            electric_asset->conductivity = conductivity;
            electric_asset->e_fraction = ElectricParticleProperty::e_fraction_box;
            electric_asset->birthdate = mysystem.GetChTime();
            mrigidBody->AddAsset(electric_asset);

            // Finally, do not forget to add the body to the system:
            mysystem.Add(mrigidBody);

            // If Irrlicht is used, setup also the visualization proxy:
            if (irr_application)
            {
                irr_application->AssetBind(mrigidBody);
                irr_application->AssetUpdate(mrigidBody);
            }
            created_body = mrigidBody;
        }

        if (rand_fract > box_fraction + sph_fraction)
        {
            //	ChCollisionModel::SetDefaultSuggestedEnvelope(0.002);
            //	ChCollisionModel::SetDefaultSuggestedMargin  (0.002);

            auto mmaterial = std::make_shared<ChMaterialSurface>();
            mmaterial->SetFriction(0.4f);
            //mmaterial->SetImpactC(0.0f);

            // Create a body
            std::shared_ptr<ChBody> mrigidBody;

            mrigidBody->SetPos(ChVector<>(-0.5 * xnozzlesize + ChRandom() * xnozzlesize + xnozzle, ynozzle + i * 0.005, -0.5 * znozzlesize + ChRandom() * znozzlesize));
            mrigidBody->SetMass(cylmass);
            mrigidBody->SetInertiaXX(ChVector<>(cylinertia, cylinertia2, cylinertia));
            mrigidBody->SetMaterialSurface(mmaterial);

            // Define a collision shape 
            mrigidBody->GetCollisionModel()->ClearModel();
            mrigidBody->GetCollisionModel()->AddCylinder(sphrad, sphrad, cylhei);
            mrigidBody->GetCollisionModel()->BuildModel();
            mrigidBody->SetCollide(true);

            // Attach a visualization shape asset. 
            std::shared_ptr<ChCylinderShape> mcyl(new ChCylinderShape);
            mcyl->GetCylinderGeometry().rad = sphrad;
            mcyl->GetCylinderGeometry().p1 = ChVector<>(0, cylhei / 2, 0);
            mcyl->GetCylinderGeometry().p2 = ChVector<>(0, -cylhei / 2, 0);
            mrigidBody->AddAsset(mcyl);
            // Attach a visualization texture
            std::shared_ptr<ChTexture> mtexture(new ChTexture);
            mtexture->SetTextureFilename("../objects/bluwhite.png");
            mrigidBody->AddAsset(mtexture);

            // Attach a custom asset. Look how to create and add a custom asset to the object! ***ALEX
            std::shared_ptr<ElectricParticleProperty> electric_asset(new ElectricParticleProperty);
            electric_asset->Cdim = ChVector<>(sphrad * 2, cylhei, sphrad * 2);
            electric_asset->conductivity = conductivity;
            electric_asset->e_fraction = ElectricParticleProperty::e_fraction_cylinder;
            electric_asset->birthdate = mysystem.GetChTime();
            mrigidBody->AddAsset(electric_asset);

            // Finally, do not forget to add the body to the system:
            mysystem.Add(mrigidBody);

            // If Irrlicht is used, setup also the visualization proxy:
            if (irr_application)
            {
                irr_application->AssetBind(mrigidBody);
                irr_application->AssetUpdate(mrigidBody);
            }
            created_body = mrigidBody;
        }

        // This is an optional hack that largely affects the stability of the
        // simulation. 
        // In fact, if particles happen to spin too fast, the collision detection
        // starts to be very slow, and maybe also inaccurate; also, the time integration
        // could diverge. To get rid of this problem wihtout reducing too much the timestep, 
        // one can enable a limit on angular velocity and/or linear velocity. NOTE that 
        // this achieves greater stability at the cost of lower realism of the simulation, 
        // so it should not be abused. ***ALEX

        bool do_velocity_clamping = true;

        if (created_body && do_velocity_clamping)
        {
            created_body->SetLimitSpeed(true);
            created_body->SetMaxSpeed(100);
            created_body->SetMaxWvel(250);
        }
    }
}


// TODO: why check for age? Why don't check if the particle goes out of a scope? (z coordinate?)
void ElectrostaticCoronaSeparator::purge_debris(ChSystem& mysystem, double max_age)
{
    for (auto iter = mysystem.IterBeginBodies(); iter != mysystem.IterEndBodies(); ++iter)
    {
        if (auto electric_properties = std::dynamic_pointer_cast<ElectricParticleProperty>(*iter))
        {
            double particle_age = mysystem.GetChTime() - electric_properties->birthdate;
            if (particle_age > max_age)
            {
                mysystem.RemoveBody(*iter);
                ++iter;
            }
        }
    }
}

void ElectrostaticCoronaSeparator::DrawForces(irrlicht::ChIrrApp& application, double scalefactor) const
{
    ChSystem* msystem = application.GetSystem();

    for (unsigned int i = 0; i < msystem->Get_bodylist()->size(); i++)
    {
        auto abody = (*msystem->Get_bodylist())[i];

        bool was_a_particle = false;

        // Fetch the ElectricParticleProperty asset from the list of 
        // assets that have been attached to the object, and retrieve the
        // custom data that have been stored. ***ALEX
        for (unsigned int na = 0; na < abody->GetAssets().size(); na++)
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

void ElectrostaticCoronaSeparator::UpdateTrajectories(irrlicht::ChIrrApp& application) const
{
    ChSystem* msystem = application.GetSystem();

    for (unsigned int i = 0; i < msystem->Get_bodylist()->size(); i++)
    {
        auto abody = (*msystem->Get_bodylist())[i];

        bool was_a_particle = false;

        // Fetch the ElectricParticleProperty asset from the list of 
        // assets that have been attached to the object, and retrieve the
        // custom data that have been stored. ***ALEX
        for (unsigned int na = 0; na < abody->GetAssets().size(); na++)
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

void ElectrostaticCoronaSeparator::DrawTrajectories(irrlicht::ChIrrApp& application) const
{
    ChSystem* msystem = application.GetSystem();

    for (unsigned int i = 0; i < msystem->Get_bodylist()->size(); i++)
    {
        auto abody = (*msystem->Get_bodylist())[i];

        ChVector<> pointA = abody->GetPos();

        bool was_a_particle = false;

        // Fetch the ElectricParticleProperty asset from the list of 
        // assets that have been attached to the object, and retrieve the
        // custom data that have been stored. ***ALEX
        for (unsigned int na = 0; na < abody->GetAssets().size(); na++)
        {
            std::shared_ptr<ChAsset> myasset = abody->GetAssetN(na);
            if (auto trajectoryasset = std::dynamic_pointer_cast<ParticleTrajectory>(myasset))
            {
                int npoints = 0;
                std::list<ChVector<>>::const_iterator iterator;
                std::list<ChVector<>>::const_iterator iteratorspeed;
                iteratorspeed = trajectoryasset->speeds.begin();
                for (iterator = trajectoryasset->positions.begin(); iterator != trajectoryasset->positions.end(); ++iterator)
                {
                    ChVector<> pointB = *iterator;
                    ChVector<> speed = *iteratorspeed;
                    if (npoints > 0)
                    {
                        double scalarspeed = speed.Length();
                        double normalizedspeed = scalarspeed / 5.0;
                        irr::video::SColor mcol(255, (int)(255. * normalizedspeed), (int)(255. * normalizedspeed), (int)(255. * (1.0 - normalizedspeed)));
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

int ElectrostaticCoronaSeparator::Setup()
{
    
}

int ElectrostaticCoronaSeparator::simulate()
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

    emitter_positions->Outlet() = nozzle_csys;
    emitter_positions->Outlet().rot.Q_from_AngAxis(CH_C_PI_2, VECT_X); // rotate outlet 90° on x

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
            mfun->Set_yconst(-drumspeed_radss); // angular speed in [rad/s]

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
        drum_csys.pos + ChVector<>(this->flowmeter_xmin + 0.5 * flowmeter_length,
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

            apply_forces(&mphysicalSystem, // contains all bodies
                         drum_csys, // pos and rotation of axis of drum (not rotating reference!)
                         drumspeed_radss, // speed of drum
                         totframes);


            if (receiver.checkbox_plotforces->isChecked())
                DrawForces(application, 1000);

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
                    mfun->Set_yconst(-drumspeed_radss); // angular speed in [rad/s]

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
                    for (unsigned int i = 0; i < mphysicalSystem.Get_bodylist()->size(); i++)
                    {
                        auto abody = (*mphysicalSystem.Get_bodylist())[i];

                        // Fetch the ElectricParticleProperty asset from the list
                        for (unsigned int na = 0; na < abody->GetAssets().size(); na++)
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
        for (int ir = 0; ir < countdistribution->mmass_plastic.GetRows(); ++ir)
            for (int ic = 0; ic < countdistribution->mmass_plastic.GetColumns(); ++ic)
                totmass_plastic += countdistribution->mmass_plastic(ir, ic);
        if (totmass_plastic == 0)
            yscalefactor_plastic = 0; // if not yet particle passed through sampling rectangle
        else
            yscalefactor_plastic = (0.002 * countdistribution->mmass_plastic.GetRows() * countdistribution->mmass_plastic.GetColumns()) / totmass_plastic;

        drawDistribution(application.GetVideoDriver(),
                         countdistribution->mmass_plastic * yscalefactor_plastic,
                         distrrectangle->rectangle_csys,
                         distrrectangle->Xsize,
                         distrrectangle->Ysize,
                         irr::video::SColor(255, 255, 0, 0));

        double yscalefactor_metal;
        double totmass_metal = 0;
        for (int ir = 0; ir < countdistribution->mmass_metal.GetRows(); ++ir)
            for (int ic = 0; ic < countdistribution->mmass_metal.GetColumns(); ++ic)
                totmass_metal += countdistribution->mmass_metal(ir, ic);
        if (totmass_plastic == 0)
            yscalefactor_metal = 0; // if not yet particle passed through sampling rectangle
        else
            yscalefactor_metal = (0.002 * countdistribution->mmass_metal.GetRows() * countdistribution->mmass_metal.GetColumns()) / totmass_metal;

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
