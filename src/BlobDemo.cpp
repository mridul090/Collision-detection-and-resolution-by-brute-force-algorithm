/*
 * The Blob demo.
 *
 */
#include <gl/glut.h>
#include "app.h"
#include "coreMath.h"
#include "pcontacts.h"
#include "pworld.h"
#include "blobcollisiondetection.h"
#include <stdio.h>
#include <cassert>

#define BLOB_NUMBERS  30
#define PLATFROM_NUMBERS  9

const Vector2 Vector2::GRAVITY = Vector2(0,-9.81);

/**
 * Platforms are two dimensional: lines on which the 
 * particles can rest. Platforms are also contact generators for the physics.
 */

class Platform : public ParticleContactGenerator
{
public:
    Vector2 start;
    Vector2 end;
    /**
     * Holds a pointer to the particles we're checking for collisions with. 
     */
    Particle* particle;

    virtual unsigned addContact(
        ParticleContact *contact, 
        unsigned limit
        ) const;
};

unsigned Platform::addContact(ParticleContact *contact, 
                              unsigned limit) const
{
    
	//const static float restitution = 0.8f;
	const static float restitution = 1.0f;
	unsigned used = 0;
  
    // Detect blob collisions
    used += BlobCollisionDetection::detectCollisions(particle, BLOB_NUMBERS, contact + used, limit - used);

    for (unsigned i = 0; i < BLOB_NUMBERS; i++) {

        // Check for penetration
        Vector2 toParticle = particle[i].getPosition() - start;
        Vector2 lineDirection = end - start;
        float projected = toParticle * lineDirection;
        float platformSqLength = lineDirection.squareMagnitude();
        float squareRadius = particle[i].getRadius() * particle[i].getRadius();;

        if (projected <= 0)
        {

            // The blob is nearest to the start point
            if (toParticle.squareMagnitude() < squareRadius)
            {
                // We have a collision
                contact->contactNormal = toParticle.unit();
                contact->restitution = restitution;
                contact->particle[0] = particle + i;
                contact->particle[1] = 0;
                contact->penetration = particle[i].getRadius() - toParticle.magnitude();
                used++;
                contact++;
            }


        }
        else if (projected >= platformSqLength)
        {
            // The blob is nearest to the end point
            toParticle = particle[i].getPosition() - end;
            if (toParticle.squareMagnitude() < squareRadius)
            {
                // We have a collision
                contact->contactNormal = toParticle.unit();
                contact->restitution = restitution;
                contact->particle[0] = particle+i;
                contact->particle[1] = 0;
                contact->penetration = particle[i].getRadius() - toParticle.magnitude();
                used++;
                contact++;
            }
        }
        else
        {
            // the blob is nearest to the middle.
            float distanceToPlatform = toParticle.squareMagnitude() - projected * projected / platformSqLength;
            if (distanceToPlatform < squareRadius)
            {
                // We have a collision
                Vector2 closestPoint = start + lineDirection * (projected / platformSqLength);

                contact->contactNormal = (particle[i].getPosition() - closestPoint).unit();
                contact->restitution = restitution;
                contact->particle[0] = particle + i;
                contact->particle[1] = 0;
                contact->penetration = particle[i].getRadius() - sqrt(distanceToPlatform);
                used++;
                contact++;
            }
        }
    }

    return used;
}


class BlobDemo : public Application
{
    Particle *blob;

    Platform *platform;

    ParticleWorld world;

public:
    /** Creates a new demo object. */
    BlobDemo();
    virtual ~BlobDemo();

    /** Returns the window title for the demo. */
    virtual const char* getTitle();

    /** Display the particles. */
    virtual void display();

    /** Update the particle positions. */
    virtual void update();
	
};

// Method definitions
BlobDemo::BlobDemo()
    :
    world(10, 5)
{


	width = 400; height = 400; 
	nRange = 100.0;

    // Create the blob storage
    blob = new Particle[BLOB_NUMBERS];
	  
    // Create the platform
	platform = new Platform[PLATFROM_NUMBERS];
	

	platform[0].start = Vector2(-10.0, 0.0);
	platform[0].end = Vector2(50.0, 20.0);

    platform[1].start = Vector2(50.0, 0.0);
    platform[1].end = Vector2(50.00, -40.0);

    platform[2].start = Vector2(50.0, -40.0);
    platform[2].end = Vector2(-50.0, -40.0);

    platform[3].start = Vector2(-10.0, 0.0);
    platform[3].end = Vector2(-50.0, 20.0);

    platform[4].start = Vector2(-10.0, 0.0);
    platform[4].end = Vector2(-50.0, -40.0);

    platform[5].start = Vector2(-98.0, 98.0);
    platform[5].end = Vector2(-98.0, -98.0);

    platform[6].start = Vector2(98.0, -98.0);
    platform[6].end = Vector2(-98.0, -98.0);

    platform[7].start = Vector2(98.0, -98.0);
    platform[7].end = Vector2(98.0, 98.0);

    platform[8].start = Vector2(-98.0, 98.0);
    platform[8].end = Vector2(98.0, 98.0);

    for (unsigned i = 0; i < PLATFROM_NUMBERS; i++)
    {
        platform[i].particle = blob;
        world.getContactGenerators().push_back(platform + i);
    }

    for (unsigned i = 0; i < BLOB_NUMBERS; i++)
    {
        // Create the blob
        blob[i].setPosition(20.0f * (i - 1), 90.0);
        blob[i].setRadius(7);
        blob[i].setVelocity(0, -70);
        blob[i].setAcceleration(0, -9);
        blob[i].setDamping(1.0);
        blob[i].setMass(100.0f);
        blob[i].clearAccumulator();

        world.getParticles().push_back(blob + i);

    }
   
}


BlobDemo::~BlobDemo()
{
    delete[] platform;
    delete[] blob;
}

void BlobDemo::display()
{
    Application::display();

    glBegin(GL_LINES);
    glColor3f(0, 1, 1);
    for (unsigned i = 0; i < PLATFROM_NUMBERS; i++) {
        const Vector2& p0 = platform[i].start;
        const Vector2& p1 = platform[i].end;
        glVertex2f(p0.x, p0.y);
        glVertex2f(p1.x, p1.y);
    }
    glEnd();

    for (unsigned i = 0; i < BLOB_NUMBERS; i++) {
        glColor3f((i % 2) ? 0 : 1, (i % 4) ? 1 : 0, (i % 3) ? 0 : 1);
        const Vector2& Blob_p = blob[i].getPosition();
        glPushMatrix();
        glTranslatef(Blob_p.x, Blob_p.y, 0);
        glutSolidSphere(blob[i].getRadius(), 12, 12);
        glPopMatrix();
    }

    glutSwapBuffers();
}

void BlobDemo::update()
{
    // Recenter the axes
	float duration = timeinterval/1000;
    // Run the simulation
    world.runPhysics(duration);

    Application::update();
}

const char* BlobDemo::getTitle()
{
    return "Blob Demo";
}

/**
 * Called by the common demo framework to create an application
 * object (with new) and return a pointer.
 */
Application* getApplication()
{
    return new BlobDemo();
}