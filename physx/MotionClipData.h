//
// Created by lasagnaphil on 2019-03-10.
//

#ifndef MOTION_EDITING_POSEDATA_H
#define MOTION_EDITING_POSEDATA_H

#include <string>
#include <vector>
#include <unordered_map>
#include <glm/vec3.hpp>
#include <glm/mat4x4.hpp>
#include <glm/ext.hpp>
#include <span.hpp>
#include <optional>
#include <stack>

#include "Storage.h"

struct PoseState {
    glm::vec3 rootPos;
    std::vector<glm::quat> jointRot;
};

struct PoseTreeNode {
    std::string name;
    glm::vec3 offset;
    uint32_t parent;
    std::vector<uint32_t> childJoints;
    bool isEndSite = false;

    float boneWidthX, boneWidthZ;
};

struct PoseTree {
    std::vector<PoseTreeNode> allNodes;

    uint32_t numJoints = 0;
    uint32_t numNodes = 0;

    const PoseTreeNode& operator[](uint32_t idx) const { return allNodes[idx]; }
    PoseTreeNode& operator[](uint32_t idx) { return allNodes[idx]; }

    const PoseTreeNode& getRootNode() const {
        return allNodes[0];
    }

    PoseTreeNode& getRootNode() {
        return allNodes[0];
    }

    uint32_t getChildIdx(const PoseTreeNode& node, const std::string& name) {
        for (uint32_t childIdx : node.childJoints) {
            if (allNodes[childIdx].name == name) {
                return childIdx;
            }
        }
        return (uint32_t)-1;
    }

    const PoseTreeNode& getParentOfNode(const PoseTreeNode& node) const {
        return allNodes[node.parent];
    }

    PoseTreeNode& getParentOfNode(const PoseTreeNode& node) {
        return allNodes[node.parent];
    }

    template <typename Fun>
    void iterateChildrenOfNode(const PoseTreeNode& node, Fun fun) const {
        for (uint32_t childIdx : node.childJoints) {
            fun(allNodes[childIdx]);
        }
    }

    template <typename Fun>
    void iterateChildrenOfNode(PoseTreeNode& node, Fun fun) {
        for (uint32_t childIdx : node.childJoints) {
            fun(allNodes[childIdx]);
        }
    }

    template <class Fun>
    void iterateDFS(Fun fun) {
        std::stack<uint32_t> indices;
        indices.push(0);
        while (!indices.empty()) {
            uint32_t idx = indices.top();
            indices.pop();
            PoseTreeNode& node = allNodes[idx];
            fun(node);
            for (int childIdx : node.childJoints) {
                indices.push(childIdx);
            }
        }
    }
};

struct MotionClipData {
    PoseTree poseTree;
    std::vector<PoseState> poseStates;

    // Motion data
    enum class ChannelType : uint8_t {
        Xrot, Yrot, Zrot, Xpos, Ypos, Zpos
    };

    uint32_t numChannels = 0;
    uint32_t numFrames = 0;

    float frameTime;

    static bool loadFromFile(const std::string &filename, MotionClipData &data, float scale = 1.0f);

    void print() const;

    PoseState& getFrameState(uint32_t frameIdx) { return poseStates[frameIdx]; }
    const PoseState& getFrameState(uint32_t frameIdx) const { return poseStates[frameIdx]; }

    glm::vec3& getRootPos(uint32_t frameIdx) { return poseStates[frameIdx].rootPos; }
    const glm::vec3& getRootPos(uint32_t frameIdx) const { return poseStates[frameIdx].rootPos; }

    glm::quat& getJointRot(uint32_t frameIdx, uint32_t jointIdx) { return poseStates[frameIdx].jointRot[jointIdx]; }
    const glm::quat& getJointRot(uint32_t frameIdx, uint32_t jointIdx) const { return poseStates[frameIdx].jointRot[jointIdx]; }

private:
    void printRecursive(uint32_t jointID, int depth) const;
};

#endif //MOTION_EDITING_POSEDATA_H
