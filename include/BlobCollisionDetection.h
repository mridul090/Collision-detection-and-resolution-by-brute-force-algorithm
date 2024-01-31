#pragma once
#ifndef BLOBCOLLISIONDETECTION_H
#define BLOBCOLLISIONDETECTION_H

#include "particle.h"
#include "pcontacts.h"

class BlobCollisionDetection
{
public:
    static unsigned detectCollisions(Particle* particles, unsigned numParticles, ParticleContact* contactArray, unsigned limit);
};

#endif // BLOBCOLLISIONDETECTION_H