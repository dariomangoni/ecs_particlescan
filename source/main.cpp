///////////////////////////////////////////////////
//
//   Corona Electrostatic Separator
//
//   This program is based on the following
//   libraries:
//   - ChronoEngine
//   - Irrlicht
// 
//  
// ------------------------------------------------ 
///////////////////////////////////////////////////


#include "ElectrostaticCoronaSeparator.h"

// Use the namespace of Chrono

using namespace chrono;
using namespace postprocess;

// Use the main namespaces of Irrlicht
using namespace irr;
using namespace core;
using namespace scene;
using namespace video;
using namespace io;
using namespace gui;
using namespace std;


bool parse_particlescan(const char *filename)
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
            getline(iss, field, ','); static_cast<std::stringstream>(field) >> mat_ID;
            getline(iss, field, ','); static_cast<std::stringstream>(field) >> particle_ID;
            getline(iss, field, ','); static_cast<std::stringstream>(field) >> thickness;
            getline(iss, field, ','); static_cast<std::stringstream>(field) >> particle_mass;
            getline(iss, field, ','); static_cast<std::stringstream>(field) >> I1;
            getline(iss, field, ','); static_cast<std::stringstream>(field) >> I2;
            getline(iss, field, ','); static_cast<std::stringstream>(field) >> I3;
            getline(iss, field, ','); static_cast<std::stringstream>(field) >> convex_hull_Npoint;
            CHx.resize(convex_hull_Npoint);
            CHy.resize(convex_hull_Npoint);

            std::cout << std::endl;
            std::cout << "ParticleID: " << particle_ID << std::endl;
            std::cout << "MaterialID: " << mat_ID << std::endl;
            std::cout << "Thickness: " << thickness << std::endl;
            std::cout << "Mass: " << particle_mass << std::endl;
            std::cout << "Inertia " << I1 << ", " << I2 << ", " << I3 << ", " << std::endl;

            for (auto CH_sel = 0; CH_sel < convex_hull_Npoint; CH_sel++)
            {
                getline(iss, field, ','); static_cast<std::stringstream>(field) >> CHx[CH_sel];
                getline(iss, field, ','); static_cast<std::stringstream>(field) >> CHy[CH_sel];
                std::cout << "CH coord: " << CHx[CH_sel] << ", " << CHy[CH_sel] << std::endl;
            }


        }
        catch (...)
        {
            std::cout << "Misformatted input file: " << "filename" << std::endl;
            return false;
        }


    }

    return true;
}


int main(int argc, char* argv[])
{
    GetLog() << "Executing simulator \n";
    // If the .exe is launched normally, by default it will parse the settings-file below, 
    // otherwise the user can launch it by command-line by passing the filename as argument.
    std::string ces_settings_filename("../CAD_conveyor/settings.ces");

    if (argc == 2)
        ces_settings_filename = argv[1];


    try
    {
        // Create a simulator object
        ElectrostaticCoronaSeparator separator;

        //// Load settings from file, if any
        //separator.ParseSettings(ces_settings_filename.c_str());

        // Initialize and execute the simulation
        separator.simulate();

    }
    catch (ChException me)
    {
        GetLog() << "\n\n Program aborted.\n\n";
        system("pause");
    }



    return 0;
}