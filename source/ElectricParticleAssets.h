#ifndef ELECTRICPARTICLEPROPERTY_H
#define ELECTRICPARTICLEPROPERTY_H
#include <core/ChVector.h>
#include <assets/ChAsset.h>

using namespace chrono;



class ElectricParticleProperty : public ChAsset
{ 
public:
	ChVector<> Cdim{1,1,1};
	double conductivity = 0;
	double birthdate = 0;
	double chargeM = 0;		//***ida + ale (coulomb, for plastic)
	double chargeP = 0;
	ChVector<> ElectricForce;
	ChVector<> StokesForce;
	ChVector<> ElectricImageForce;
    size_t particleID = 0;

    ElectricParticleProperty() {}

	enum fraction_type
	{
		e_fraction_box,
		e_fraction_cylinder,
		e_fraction_sphere,
		e_fraction_convexhull,
		e_fraction_shaving,
		e_fraction_others
	} e_fraction = e_fraction_others;


	enum material_type
	{
		e_mat_plastic = 0,
		e_mat_metal = 1,
		e_mat_other = 2,
        e_mat_copper = 3
	} e_material = e_mat_other;

    enum shape_type
    {
        e_box = 0,
        e_cylinder = 1,
        e_sphere = 2,
        e_hull = 3,
        e_shavings = 4,
        e_other
    } e_shape = e_other;


    void SetMaterial(material_type mat, ChBody* body = nullptr)
    {
        e_material = mat;

        switch (mat)
        {
        case e_mat_plastic:
            conductivity = 0;
            if (body)
                body->SetDensity(1300); // bakelite supposed
            break;
        case e_mat_metal:
            // alu supposed
            conductivity = 3690000;
            if (body)
                body->SetDensity(2700);
            break;
        case e_mat_other:
            break;
        case e_mat_copper:
            if (body)
                body->SetDensity(8960);
            conductivity = 6670000;
            break;
        default:
            break;
        }

    }
	
};


//
// This can be added to store the trajectory on a per-particle basis.
//
class ParticleTrajectory : public ChAsset
{
public:
    std::list< ChVector<> > positions;
    std::list< ChVector<> > speeds;
    size_t max_points;

    ParticleTrajectory()
    {
        max_points = 80;
    }
};











#endif

