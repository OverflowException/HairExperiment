#include <algorithm>
#include <iostream>
#include <queue>

#define TINYGLTF_IMPLEMENTATION
#define STB_IMAGE_IMPLEMENTATION
#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "gltf_parser.h"

Skeleton GLTFParser::gen_skeleton_from_gltf(const std::string& filename,
                                            const std::string& root_name) {
    tinygltf::TinyGLTF gltf_ctx;
    tinygltf::Model model;
    std::string err;
    std::string warn;
    
    if (!gltf_ctx.LoadASCIIFromFile(&model, &err, &warn, filename)) {
        std::cout << "Load gltf fail. err = " << err << std::endl;
        return Skeleton();
    }

    // find root node id
    int root_id = 0;
    if (root_name == "") {
        // Use scene root
        assert(model.scenes.size() >= 1);
        auto& scene = model.scenes[0];
    
        assert(scene.nodes.size() >= 1);
        root_id = scene.nodes[0];
    } else {
        // Use custom root
        auto iter = std::find_if(model.nodes.begin(),
                                 model.nodes.end(),
                                 [&](const tinygltf::Node& n) {
                                     return n.name == root_name;
                                 });

        // root_name not found
        if (iter == model.nodes.end()) {
            return Skeleton();
        }
        
        root_id = iter - model.nodes.begin();
    }

    // add root node to skeleton
    Skeleton skel;
    skel[root_id] = std::move(GLTFParser::joint_from_gltf_node(model, skel, root_id, -1));
    
    // parent node id : child node id
    std::queue<std::pair<int, int>> node_queue;
    for (int id : model.nodes[root_id].children) {
        if (is_joint(model, id)) {
            node_queue.push(std::make_pair(root_id, id));
        }
    }

    while (!node_queue.empty()) {
        auto p = node_queue.front();
        int p_id = p.first; // parent id
        int c_id = p.second; // child id
        if (is_joint(model, c_id)) {
            skel[c_id] = std::move(joint_from_gltf_node(model, skel, c_id, p_id));
            
            for (int id : model.nodes[c_id].children) {
                node_queue.push(std::make_pair(c_id, id));
            }
        }
        
        node_queue.pop();
    }
    
    return skel;
}

Joint GLTFParser::joint_from_gltf_node(const tinygltf::Model& model,
                                       const Skeleton&        skel,
                                       int                    id,
                                       int                    parent) {
    const tinygltf::Node& node = model.nodes.at(id);
    Joint j;
        
    j.id = id;
    j.parent = parent;
    j.children.assign(node.children.begin(), node.children.end());
        
    assert(node.translation.size() == 3);
    assert(node.rotation.size() == 4);
    // TODO: get local btTransform from gltf node
    j.t_local = btVector3(btScalar(node.translation[0]),
                          btScalar(node.translation[1]),
                          btScalar(node.translation[2]));
        
    j.r_local = btQuaternion(btScalar(node.rotation[0]),
                             btScalar(node.rotation[1]),
                             btScalar(node.rotation[2]),
                             btScalar(node.rotation[3]));
        
    j.trans_local = btTransform(j.r_local, j.t_local);

    if (parent >= 0) {
        j.trans_model = skel.at(parent).trans_model * j.trans_local;
    } else {
        // root joint
        j.trans_model.setIdentity();
    }

    j.t_model = j.trans_model.getOrigin();
    j.r_model = j.trans_model.getRotation();
    
    return j;
}
