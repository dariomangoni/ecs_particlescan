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


    void SetMaterial(material_type mat)
    {
        e_material = mat;

        switch (mat)
        {
        case ElectricParticleProperty::e_mat_plastic:
            conductivity = 0;
            break;
        case ElectricParticleProperty::e_mat_metal:
            break;
        case ElectricParticleProperty::e_mat_other:
            break;
        case ElectricParticleProperty::e_mat_copper:
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
    unsigned int max_points;

    ParticleTrajectory()
    {
        max_points = 80;
    }
};











#endif

