#include "blobcollisiondetection.h"
#include "pcontacts.h"

unsigned BlobCollisionDetection::detectCollisions(Particle* particles, unsigned numParticles, ParticleContact* contactArray, unsigned limit)
{
    const static float restitution = 1.0f;
    unsigned used = 0;
    float Euclidean_distance;
    float TotalRedious;
    float penetration;

    for (unsigned iteration_i = 0; iteration_i < numParticles; iteration_i++)
    {
        for (unsigned iteration_j = iteration_i + 1; iteration_j < numParticles; iteration_j++)
        {
            Euclidean_distance = (particles[iteration_i].getPosition() - particles[iteration_j].getPosition()).magnitude();
            
            TotalRedious = particles[iteration_i].getRadius() + particles[iteration_j].getRadius();

            // Checking if there any collision happend 
            if (Euclidean_distance < TotalRedious){
                // Calculating how much the particles overlap with each other 
                penetration = TotalRedious - Euclidean_distance;
                
                // Initialized a new contacts is contactArray has enough space
                contactArray->particle[0] = particles + iteration_i;
                contactArray->particle[1] = particles + iteration_j;
                contactArray->contactNormal = (particles[iteration_i].getPosition() - particles[iteration_j].getPosition()).unit();// Calculate the direction along which the collision occurs
                contactArray->restitution = restitution;
                contactArray->penetration = penetration;
                contactArray++; used++;

               // Checking the limit of contact array
                if (used < limit){return used;}
            }
        }
    }

    return used;
}
