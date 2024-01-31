#include "pcontacts.h"

class BlobsParticlesCollisionDetection
{
public:
    /**
     * Fills the given contact structure with the generated
     * contact.
     */
    virtual unsigned addblobs(ParticleContact* contact,
        unsigned limit) const = 0;
};