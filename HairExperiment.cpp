#include <iostream>

#include "HairExperiment.h"

#include "btBulletDynamicsCommon.h"
#include "LinearMath/btVector3.h"
#include "LinearMath/btAlignedObjectArray.h"

#include "../CommonInterfaces/CommonRigidBodyBase.h"

constexpr int        strand_num             = 5;
constexpr int        bones_per_strand       = 3;
constexpr float      bone_radii             = 0.05;
constexpr float      bone_length            = 0.3;
constexpr float      bone_interval          = 0.05;

constexpr btScalar   head_radii             = 0.3;
const     btVector3  face_orient            = btVector3(0.0, 0.0, 1.0).normalize();
constexpr btScalar   strand_orient_interval = 180. / (strand_num - 1);

constexpr int        lut_size               = 64;

struct HairExperiment : public CommonRigidBodyBase
{
    // All rigid body pointers in this class will get its ownership transfered to
    // btDiscreteDynamicsWold soon after construnction.
    // Object life cycle lies with btDiscreteDynamicsWold.
    // See CommonRigidBodyBase::exitPhysics() 
    btRigidBody*                         head;
    btRigidBody*                         ground;
    btAlignedObjectArray<btRigidBody*>   hair_bones;
    
	HairExperiment(struct GUIHelperInterface* helper) : CommonRigidBodyBase(helper) {}
    
	virtual ~HairExperiment() {}
	virtual void initPhysics();
	virtual void renderScene();
	void resetCamera() {
		float dist = 4;
		float pitch = -35;
		float yaw = 52;
		float targetPos[3] = {0, 0, 0};
		m_guiHelper->resetCamera(dist, yaw, pitch, targetPos[0], targetPos[1], targetPos[2]);
	}

    void create_ground();
    void create_hair_strands();
    void create_head();
};

// pre-tick callback for kinematic bodies
void kinematicPreTickCallback(btDynamicsWorld* world, btScalar deltaTime);

void HairExperiment::create_ground() {
    btBoxShape* ground_shape = createBoxShape(btVector3(50., 50., 50.));
    m_collisionShapes.push_back(ground_shape);
    
    btTransform ground_transform;
    ground_transform.setIdentity();
    ground_transform.setOrigin(btVector3(0, -50, 0));
    
    btScalar ground_mass(0.);
    ground = createRigidBody(ground_mass, ground_transform, ground_shape, btVector4(0, 0, 1, 1));
}

void HairExperiment::create_hair_strands() {        
    // Re-using the same collision is better for memory usage and performance

    // this cylinder's axis is along y.
    // There are other variants with X/Z suffix, along different axis
    // HalfExtent parameters indicates length along corresponding axis
    btCylinderShapeZ* bone_shape = new btCylinderShapeZ(btVector3(bone_radii,
                                                                  0.0,
                                                                  bone_length / 2));
        
    m_collisionShapes.push_back(bone_shape);

    // Create Dynamic Objects
    // TODO: differentiate bone mass
    btScalar bone_mass(1.f);

    // Or should we try another entirely different option: Featherstone MultiBody? 
    for (int s = 0; s < strand_num; ++s) {
        for (int b = 0; b < bones_per_strand; ++b) {
            btTransform bone_trans;
            bone_trans.setIdentity();
            bone_trans.setOrigin(btVector3(
                                     0.5 * s,
                                     1,
                                     (bone_length + bone_interval) * b));

            // create rigid body with collisionShape & transform
            btRigidBody* bone = createRigidBody(bone_mass, bone_trans, bone_shape);
            bone->setDamping(0.7, 1.0);
            hair_bones.push_back(bone);
        }
            
        // add constraints to strands
        for (int b = 1; b < bones_per_strand; ++b) {
            btRigidBody& bone_a = *hair_bones[bones_per_strand * s + b - 1];
            btRigidBody& bone_b = *hair_bones[bones_per_strand * s + b];
            const btVector3& bone_a_com = bone_a.getCenterOfMassPosition();
            const btVector3& bone_b_com = bone_b.getCenterOfMassPosition();
            
            btVector3 joint_pos = (bone_a_com + bone_b_com) / 2;
            btTransform pivot_in_a;
            pivot_in_a.setIdentity();
            pivot_in_a.setOrigin(joint_pos - bone_a_com);

            btTransform pivot_in_b;
            pivot_in_b.setIdentity();
            pivot_in_b.setOrigin(joint_pos - bone_b_com);

            btGeneric6DofSpring2Constraint* hair_joint =
                new btGeneric6DofSpring2Constraint(bone_a,
                                                   bone_b,
                                                   pivot_in_a,
                                                   pivot_in_b);

            // lock all translations
            hair_joint->setLimit(0, 0, 0);
            hair_joint->setLimit(1, 0, 0);
            hair_joint->setLimit(2, 0, 0);

            // TODO: differentiate stiffness
            hair_joint->setLimit(3, -SIMD_PI / 2, SIMD_PI / 4);
            hair_joint->enableSpring(3, true);
            hair_joint->setStiffness(3, 1.2);
            hair_joint->setDamping(3, 10);    // This constraint damping is not effective as rigid body damping
            
            hair_joint->setLimit(4, -SIMD_PI / 2, SIMD_PI / 4);
            hair_joint->enableSpring(4, true);
            hair_joint->setStiffness(4, 1.2);
            hair_joint->setDamping(4, 10);
            
            hair_joint->setLimit(5, 0, 0);
            hair_joint->enableSpring(5, true);
            hair_joint->setStiffness(5, 1.2);
            hair_joint->setDamping(5, 10);
            
            // 'true' to disable collision between adjacent bones
            m_dynamicsWorld->addConstraint(hair_joint, true);
        }
    }
}

void HairExperiment::create_head() {
    btScalar head_mass(10.f);
        
    btTransform head_trans;
    head_trans.setIdentity();
    head_trans.setOrigin(btVector3(0.0, 2.0, 0.0));
    // head_trans.setRotation(btQuaternion(btVector3(1.0, 0.0, 0.0), 0));
        
    btSphereShape* head_shape = new btSphereShape(head_radii);
    head = createRigidBody(head_mass, head_trans, head_shape);
    // TODO: How does activation/deactivation work
    head->forceActivationState(DISABLE_DEACTIVATION);
        
    // add constraints to head and strand
    // TODO: maybe a hinge should look good?
    for (int s = 0; s < strand_num; ++s) {
        btRigidBody& strand_base = *hair_bones[bones_per_strand * s];
        btScalar strand_rad = (90. + strand_orient_interval * s) * SIMD_PI / 180.;
        btVector3 strand_orient = face_orient.rotate(btVector3(0., 1., 0.), strand_rad);
        btVector3 base_pos_in_head = strand_orient * (head_radii + bone_interval);
        
        btTransform head_pivot;
        head_pivot.setIdentity();
        head_pivot.setOrigin(base_pos_in_head);
        head_pivot.getBasis().setEulerZYX(0.0, strand_rad, 0.0);
        
        btVector3 base_pos_in_strand = btVector3(0, 0, -bone_length / 2 - bone_interval);
        btTransform strand_pivot;
        strand_pivot.setIdentity();
        strand_pivot.setOrigin(base_pos_in_strand);

        btGeneric6DofSpring2Constraint* head_strand_joint =
            new btGeneric6DofSpring2Constraint(*head,
                                               strand_base,
                                               head_pivot,
                                               strand_pivot);
        
        head_strand_joint->setLimit(0, 0, 0);
        head_strand_joint->setLimit(1, 0, 0);
        head_strand_joint->setLimit(2, 0, 0);
        
        head_strand_joint->setLimit(3, 0, 0);
        head_strand_joint->setLimit(4, 0, 0);
        head_strand_joint->setLimit(5, 0, 0);
        
        // 'true' to disable collision between adjacent bones
        m_dynamicsWorld->addConstraint(head_strand_joint, true);
    }

    // add dof6 cosnraints to head
    btTransform pivot_ground;
    pivot_ground.setIdentity();
    pivot_ground.setOrigin(btVector3(0.0, 2.0, 0.0));
    btTransform pivot_head;
    pivot_head.setIdentity();
    pivot_head.setOrigin(btVector3(0.0, 0.0, 0.0));
        
    btGeneric6DofSpring2Constraint* head_constraint = new btGeneric6DofSpring2Constraint(*ground,
                                                                                         *head,
                                                                                         pivot_ground,
                                                                                         pivot_head);
    // x-direction spring constraint
    head_constraint->setLimit(0, 1, -1);
    head_constraint->setEquilibriumPoint(0, -1);
    head_constraint->enableSpring(0, true);
    head_constraint->setStiffness(0, 100);
    head_constraint->setDamping(0, 10);
        
    // y-direction spring constraint
    head_constraint->setLimit(1, 1, -1);
    head_constraint->setEquilibriumPoint(1, 2);
    // TODO: why doesnt y-axis constraint work? Anything to do with gravity?
    // head_constraint->enableSpring(1, true);
    head_constraint->setStiffness(1, 100);
    head_constraint->setDamping(1, 10);

    // z-direction spring constraint
    head_constraint->setLimit(2, 1, -1);
    head_constraint->setEquilibriumPoint(2, -1);
    head_constraint->enableSpring(2, true);
    head_constraint->setStiffness(2, 100);
    head_constraint->setDamping(2, 10);

    // Lock all rotations
    head_constraint->setLimit(3, 0, 0);
    head_constraint->setLimit(4, 0, 0);
    head_constraint->setLimit(5, 0, 0);
        
    m_dynamicsWorld->addConstraint(head_constraint);
}    

void HairExperiment::initPhysics()
{
	m_guiHelper->setUpAxis(1);

	createEmptyDynamicsWorld();
	//m_dynamicsWorld->setGravity(btVector3(0,0,0));
	m_guiHelper->createPhysicsDebugDrawer(m_dynamicsWorld);

	if (m_dynamicsWorld->getDebugDrawer()) {
		m_dynamicsWorld->getDebugDrawer()->setDebugMode(btIDebugDraw::DBG_DrawWireframe + btIDebugDraw::DBG_DrawContactPoints);
    }

    create_ground();

    create_hair_strands();

    create_head();

	m_guiHelper->autogenerateGraphicsObjects(m_dynamicsWorld);
}

void kinematicPreTickCallback(btDynamicsWorld* world, btScalar delta_time) {
    btRigidBody* head = reinterpret_cast<btRigidBody*>(world->getWorldUserInfo());
    btTransform trans;
	btVector3 linear_vel(0, 0, 0);
	btVector3 ang_vel(0, 3, 0);
    btTransformUtil::integrateTransform(head->getWorldTransform(), linear_vel, ang_vel, delta_time, trans);
    head->getMotionState()->setWorldTransform(trans);
}

void HairExperiment::renderScene()
{
	CommonRigidBodyBase::renderScene();
}

CommonExampleInterface* HairExperimentCreateFunc(CommonExampleOptions& options)
{
	return new HairExperiment(options.m_guiHelper);
}

B3_STANDALONE_EXAMPLE(HairExperimentCreateFunc)
