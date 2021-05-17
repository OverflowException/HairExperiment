#pragma once

#include "skeleton.h"
#include "tiny_gltf/tiny_gltf.h"

class GLTFParser {
public:
    static Skeleton gen_skeleton_from_gltf(const std::string& filename,
                                           const std::string& root_name = "");

private:
    inline static bool is_joint(const tinygltf::Model& model, int node_id) {
        return model.nodes[node_id].mesh == -1;
    }

    static Joint joint_from_gltf_node(const tinygltf::Model& model,
                               const Skeleton&        skeleton,
                               int                    id,
                               int                    parent);
};

