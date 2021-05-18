#include <iostream>
#include <queue>

#include "hair_experiment.h"

#include "btBulletDynamicsCommon.h"
#include "LinearMath/btVector3.h"
#include "LinearMath/btAlignedObjectArray.h"

#include "../CommonInterfaces/CommonRigidBodyBase.h"

#include "gltf_parser.h"
#include "math_util.h"
#include "print_helper.h"
#include "skeleton.h"

const     btVector3  up_direction           = btVector3(0.0, 1.0, 0.0);

constexpr int        strand_num             = 5;
constexpr int        bones_per_strand       = 4;
constexpr float      bone_radii             = 0.02;
constexpr float      bone_length            = 0.2;
constexpr float      bone_interval          = 0.001;

constexpr btScalar   head_radii             = 0.3;
// z axis of head frame
const     btVector3  face_orient            = btVector3(-1.0, 0.0, 0.0).normalize();
const     btScalar   head_tilt              = MathUtil::deg2rad(0.0);
// y axis of head frame
const     btVector3  head_v_axis            = MathUtil::find_head_axis(face_orient, up_direction, head_tilt);
// x axis of head frame
const     btVector3  head_h_axis            = head_v_axis.cross(face_orient);
const     btMatrix3x3 head_basis            = btMatrix3x3(head_h_axis.x(), head_v_axis.x(), face_orient.x(),
                                                          head_h_axis.y(), head_v_axis.y(), face_orient.y(),
                                                          head_h_axis.z(), head_v_axis.z(), face_orient.z());

const     btScalar   strands_longi_span    = MathUtil::deg2rad(90);
const     btScalar   strand_longi_interval = strands_longi_span / (strand_num - 1);
const     btScalar   strand_lat            = MathUtil::deg2rad(60.0);

const     std::string gltf_filename        = "./model/Girl.gltf";
const     std::set<int> dynamic_joints     = {28, 29, 30, 31, // "hairA1"
                                              18, 19, 20, 21, // "hairB1"
                                              23, 24, 25, 26, // "hairC1"
                                              33, 34, 35, 36, // "joint1"
                                              
                                              100, 101, 102,
                                              96, 97, 98,
                                              104, 105, 106,
                                              116, 117, 118,
                                              108, 109, 110,
                                              112, 113, 114};


struct HairExperiment : public CommonRigidBodyBase
{
    // All rigid body pointers in this class will get its ownership transfered to
    // btDiscreteDynamicsWold soon after construnction.
    // Object life cycle lies with btDiscreteDynamicsWold.
    // See CommonRigidBodyBase::exitPhysics() 
    btRigidBody*                         head;
    btRigidBody*                         ground;
    btAlignedObjectArray<btRigidBody*>   hair_bones;
    Skeleton                             init_skel;
    
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

    void create_skeleton();
    
    btCompoundShape* add_bone_as_child_shape(const Joint& j_a,
                                             const Joint& j_b,
                                             btCompoundShape* comp_shape);

    btRigidBody* create_bone_from_joints(const Joint& j_a,
                                         const Joint& j_b,
                                         btScalar mass);

    btPoint2PointConstraint* create_p2p_constraint(btRigidBody& rb_a,
                                              btRigidBody& rb_b,
                                              const Joint& pivot);
    
    btGeneric6DofSpring2Constraint* create_spring_constraint(btRigidBody& rb_a,
                                                             btRigidBody& rb_b,
                                                             const Joint& pivot);
    
    btRigidBody* create_joint_as_sphere(const Joint& j);
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

btCompoundShape* HairExperiment::add_bone_as_child_shape(const Joint& j_a, const Joint& j_b, btCompoundShape* comp_shape) {
    btVector3 a2b = j_b.t_model - j_a.t_model;
    
    btScalar len_a2b = a2b.length();
    if (len_a2b == 0.0) {
        return nullptr;
    }
    
    // center of mass
    btVector3 com = j_b.t_model.lerp(j_a.t_model, 0.5);
    
    btTransform trans;
    trans.setIdentity();
    trans.setOrigin(com);
    trans.setRotation(MathUtil::rot_between(btVector3(0.0, 0.0, 1.0), a2b));

    // create an y-axis cylinder
    btCylinderShapeZ* shape = new btCylinderShapeZ(btVector3(bone_radii,
                                                             0.0,
                                                             (len_a2b - bone_interval) / 2));

    comp_shape->addChildShape(trans, shape);

    return comp_shape;
}

btRigidBody* HairExperiment::create_bone_from_joints(const Joint& j_a,
                                                     const Joint& j_b,
                                                     btScalar mass) {
    btVector3 a2b = j_b.t_model - j_a.t_model;
    
    btScalar len_a2b = a2b.length();
    if (len_a2b == 0.0) {
        return nullptr;
    }
    
    // center of mass
    btVector3 com = j_b.t_model.lerp(j_a.t_model, 0.5);
    
    btTransform trans;
    trans.setIdentity();
    trans.setOrigin(com);
    trans.setRotation(MathUtil::rot_between(btVector3(0.0, 0.0, 1.0), a2b));

    // create an y-axis cylinder
    btCylinderShapeZ* shape = new btCylinderShapeZ(btVector3(bone_radii,
                                                             0.0,
                                                             (len_a2b - bone_interval) / 2));
    m_collisionShapes.push_back(shape);

    btRigidBody* bone = createRigidBody(mass, trans, shape);
    return bone;
}

btPoint2PointConstraint* HairExperiment::create_p2p_constraint(btRigidBody& rb_a,
                                                          btRigidBody& rb_b,
                                                          const Joint& pivot) {
    btVector3 pivot_a = rb_a.getCenterOfMassTransform().inverse() * pivot.t_model;
    btVector3 pivot_b = rb_b.getCenterOfMassTransform().inverse() * pivot.t_model;

    return new btPoint2PointConstraint(rb_a, rb_b, pivot_a, pivot_b);
}

btGeneric6DofSpring2Constraint* HairExperiment::create_spring_constraint(btRigidBody& rb_a,
                                                                         btRigidBody& rb_b,
                                                                         const Joint& pivot) {
    const btTransform& trans_a = rb_a.getCenterOfMassTransform();
    const btTransform& trans_a_inv = trans_a.inverse();
    const btTransform& trans_b = rb_b.getCenterOfMassTransform();
    
    btVector3 t_a_pivot = pivot.t_model - trans_a.getOrigin();
    
    btVector3 z_axis_a = btTransform(trans_a.getRotation(), btVector3(0.0, 0.0, 0.0)) * btVector3(0.0, 0.0, 1.0);
    btQuaternion r_a_pivot = MathUtil::rot_between(z_axis_a, t_a_pivot);
    
    btTransform pivot_trans = btTransform(r_a_pivot, t_a_pivot) * trans_a;

    // above computations are in model space, below computations in rigidbody local space
    
    btTransform pivot_frame_a = trans_a.inverse() * pivot_trans;
    btTransform pivot_frame_b = trans_b.inverse() * pivot_trans;
    
    btGeneric6DofSpring2Constraint* constraint =
        new btGeneric6DofSpring2Constraint(rb_a, rb_b, pivot_frame_a, pivot_frame_b);
    
    // lock all translations
    constraint->setLinearLowerLimit(btVector3(0.0, 0.0, 0.0));
    constraint->setLinearUpperLimit(btVector3(0.0, 0.0, 0.0));

    // lock xy rotations TODO: why?
    constraint->setAngularLowerLimit(btVector3(0.0, 0.0, 1.0));
    constraint->setAngularUpperLimit(btVector3(0.0, 0.0, -1.0));
    
    constraint->enableSpring(3, false);
    constraint->setStiffness(3, 100.0);
    constraint->setDamping(3, 10.0);

    constraint->enableSpring(4, false);
    constraint->setStiffness(4, 100.0);
    constraint->setDamping(4, 10.0);

    constraint->enableSpring(5, false);
    
    return constraint;
}


btRigidBody* HairExperiment::create_joint_as_sphere(const Joint& j) {
    btScalar mass(0.0);
    btTransform trans;
    trans.setIdentity();
    trans.setOrigin(j.t_model);
    
    btSphereShape* shape = new btSphereShape(0.02);

    btRigidBody* joint = createRigidBody(mass, trans, shape);

    return joint;
}

void HairExperiment::create_skeleton() {
    init_skel = std::move(GLTFParser::gen_skeleton_from_gltf(gltf_filename));

    for (auto& j : init_skel) {
        std::cout << j.first << " = " << j.second << std::endl;
    }

    btCompoundShape* skel_shape = new btCompoundShape();
    m_collisionShapes.push_back(skel_shape);

    std::map<int, btRigidBody*> dynamic_rb_map;
    
    for (auto p : init_skel) {
        int id = p.first;
        Joint& j_c = p.second;
        if (j_c.parent < 0) {
            continue;
        }
        Joint& j_p = init_skel[j_c.parent];

        auto iter = dynamic_joints.find(j_p.id);
        if (iter != dynamic_joints.end()) {
            // found a dynamic root
            dynamic_rb_map[j_p.id] = create_bone_from_joints(j_p, j_c, 0.1);
            dynamic_rb_map[j_p.id]->setDamping(0.7, 1.0);
        } else {                    
            add_bone_as_child_shape(j_p, j_c, skel_shape);
        }
    }

    btScalar skel_mass(10.0);
    btTransform skel_trans;
    skel_trans.setIdentity();
    
    // TODO: center of mass looks weird
    btRigidBody* skel = createRigidBody(skel_mass, skel_trans, skel_shape);
    skel->setDamping(0.7, 0.5);

    // Attach dynamics bones
    for (int id : dynamic_joints) {
        Joint& j_c = init_skel[id];
        auto iter = dynamic_rb_map.find(j_c.parent);
        if (iter != dynamic_rb_map.end()) {
            m_dynamicsWorld->addConstraint(create_spring_constraint(*iter->second,
                                                             *dynamic_rb_map[id],
                                                             j_c),
                                           true);
        } else {
            m_dynamicsWorld->addConstraint(create_spring_constraint(*skel,
                                                             *dynamic_rb_map[id],
                                                             j_c),
                                           true);
        }
    }
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
    btScalar bone_mass(.3f);

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
            
            hair_joint->setLimit(4, 1, -1);
            hair_joint->enableSpring(4, true);
            hair_joint->setStiffness(4, 1.2);
            hair_joint->setDamping(4, 10);
            
            hair_joint->setLimit(5, 1, -1);
            
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
    head_trans.setBasis(head_basis);
        
    btSphereShape* head_shape = new btSphereShape(head_radii);
    head = createRigidBody(head_mass, head_trans, head_shape);
    // TODO: How does activation/deactivation work
    head->forceActivationState(DISABLE_DEACTIVATION);

    // add constraints to head and strand
    // TODO: maybe a hinge should look good?
    for (int s = 0; s < strand_num; ++s) {
        btRigidBody& strand_base = *hair_bones[bones_per_strand * s];
        btScalar strand_longi_in_head = SIMD_PI  - strands_longi_span / 2 + strand_longi_interval * s;
        btVector3 strand_orient_in_head = btVector3(0.0, 0.0, 1.0)
            .rotate(btVector3(0.0, 1.0, 0.0), strand_longi_in_head)
            .rotate(btVector3(1.0, 0.0, 0.0), strand_lat);
        
        btVector3 pivot_pos_in_head = strand_orient_in_head * (head_radii + bone_interval);
        btScalar equator_sin = strand_orient_in_head.y();
        btScalar equator_cos = sqrt(1 - equator_sin * equator_sin);
        btTransform pivot_in_head;
        pivot_in_head.setIdentity();
        pivot_in_head.setOrigin(pivot_pos_in_head);
        pivot_in_head.setBasis(btMatrix3x3(btQuaternion(btVector3(0.0, 1.0, 0.0), strand_longi_in_head)) *
                               btMatrix3x3(1,   0.0,         0.0,
                                           0.0, equator_cos, equator_sin,
                                           0.0, -equator_sin,equator_cos));
        
        btVector3 pivot_pos_in_strand = btVector3(0, 0, -bone_length / 2 - bone_interval);
        btTransform pivot_in_strand;
        pivot_in_strand.setIdentity();
        pivot_in_strand.setOrigin(pivot_pos_in_strand);

        btGeneric6DofSpring2Constraint* head_strand_joint =
            new btGeneric6DofSpring2Constraint(*head,
                                               strand_base,
                                               pivot_in_head,
                                               pivot_in_strand);
        
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
    pivot_head.setBasis(head_basis.inverse());
        
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
    
	m_guiHelper->createPhysicsDebugDrawer(m_dynamicsWorld);

	if (m_dynamicsWorld->getDebugDrawer()) {
		m_dynamicsWorld->getDebugDrawer()->setDebugMode(btIDebugDraw::DBG_DrawWireframe + btIDebugDraw::DBG_DrawContactPoints);
    }

    create_ground();
    
    // create_hair_strands();

    // create_head();
    
    create_skeleton();

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
