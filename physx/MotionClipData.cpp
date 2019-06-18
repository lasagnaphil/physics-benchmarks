//
// Created by lasagnaphil on 2019-03-10.
//

#include "MotionClipData.h"

#include <fstream>
#include <sstream>
#include <iostream>

#define GLM_ENABLE_EXPERIMENTAL
#include <glm/gtx/euler_angles.hpp>
#include <glm/gtx/quaternion.hpp>

std::unordered_map<std::string, glm::vec2> initBoneWidths() {
    std::unordered_map<std::string, glm::vec2> m;
    m["LHipJoint"] = {0.05f, 0.05f};
    m["LeftUpLeg"] = {0.05f, 0.05f};
    m["LeftLeg"] = {0.05f, 0.05f};
    m["LeftFoot"] = {0.05f, 0.05f};
    m["LeftToeBase"] = {0.05f, 0.05f};
    m["RHipJoint"] = {1.0f, 1.0f};
    m["RightUpLeg"] = {0.05f, 0.05f};
    m["RightLeg"] = {0.05f, 0.05f};
    m["RightFoot"] = {0.05f, 0.05f};
    m["RightToeBase"] = {0.05f, 0.05f};
    m["LowerBack"] = {0.05f, 0.05f};
    m["Spine"] = {0.05f, 0.05f};
    m["Spine1"] = {0.05f, 0.05f};
    m["Neck"] = {0.05f, 0.05f};
    m["Neck1"] = {0.05f, 0.05f};
    m["Head"] = {0.05f, 0.05f};
    m["LeftShoulder"] = {0.05f, 0.05f};
    m["LeftArm"] = {0.05f, 0.05f};
    m["LeftForeArm"] = {0.05f, 0.05f};
    m["LeftHand"] = {0.05f, 0.05f};
    m["LeftFingerBase"] = {0.05f, 0.05f};
    m["LeftHandIndex1"] = {0.05f, 0.05f};
    m["RightShoulder"] = {0.05f, 0.05f};
    m["RightArm"] = {0.05f, 0.05f};
    m["RightForeArm"] = {0.05f, 0.05f};
    m["RightHand"] = {0.05f, 0.05f};
    m["RightFingerBase"] = {0.05f, 0.05f};
    m["RThumb"] = {0.05f, 0.05f};
    return m;
}

static std::unordered_map<std::string, glm::vec2> boneWidths = initBoneWidths();

std::optional<MotionClipData::ChannelType> stringToChannelType(const std::string& name) {
    switch (name[0]) {
        case 'X':
            if (name == "Xposition") return MotionClipData::ChannelType::Xpos;
            else if (name == "Xrotation") return MotionClipData::ChannelType::Xrot;
            else return {};
        case 'Y':
            if (name == "Yposition") return MotionClipData::ChannelType::Ypos;
            else if (name == "Yrotation") return MotionClipData::ChannelType::Yrot;
            else return {};
        case 'Z':
            if (name == "Zposition") return MotionClipData::ChannelType::Zpos;
            else if (name == "Zrotation") return MotionClipData::ChannelType::Zrot;
            else return {};
        default:
            return {};
    }
}

bool MotionClipData::loadFromFile(const std::string &filename, MotionClipData &data, float scale) {
    std::ifstream file(filename);
    std::string line;
    std::string keyword;

    enum class ParseState { Root, Joint, EndSite, Motion };
    ParseState state = ParseState::Root;
    int lineCount = 0;

    uint32_t curJointID = {};
    uint32_t childJointID = {};

    std::istringstream iss;
    auto newLine = [&]() -> std::istringstream& {
        std::getline(file, line);
        lineCount++;
        iss = std::istringstream(line);
        return iss;
    };

    // temporary storage for end sites
    std::vector<PoseTreeNode> endSites;

    // type of each channel
    std::vector<ChannelType> channelTypeData;

    bool finished = false;
    bool error = false;
    while (!error && !finished) {
        switch (state) {
            case ParseState::Root: {
                PoseTreeNode curJoint;
                curJoint.isEndSite = false;
                curJointID = 0; // root joint id

                curJoint.parent = 0;

                newLine() >> keyword;

                newLine() >> keyword >> curJoint.name;

                newLine() >> keyword;
                if (keyword != "{") {
                    error = true;
                    break;
                }
                newLine() >> keyword >> curJoint.offset.x >> curJoint.offset.y >> curJoint.offset.z; // OFFSET
                curJoint.offset *= scale;
                int channels;
                std::string channelNames[6];
                newLine() >> keyword >> channels
                          >> channelNames[0] >> channelNames[1] >> channelNames[2]
                          >> channelNames[3] >> channelNames[4] >> channelNames[5];
                if (channels != 6) {
                    std::cerr << "Only root node with 6 channels supported!" << std::endl;
                    error = true;
                    break;
                }
                for (int i = 0; i < 6; i++) {
                    if (auto channelType = stringToChannelType(channelNames[i])) {
                        channelTypeData.push_back(*channelType);
                    }
                    else {
                        std::cerr << "Invalid channel type." << std::endl;
                        error = true;
                        break;
                    }
                }
                data.numChannels += 6; data.poseTree.numJoints++; data.poseTree.numNodes++; // CHANNELS
                childJointID = curJointID;

                newLine() >> keyword;
                if (keyword == "JOINT") {
                    state = ParseState::Joint;
                }
                else if (keyword == "End") {
                    state = ParseState::EndSite;
                }
                if (boneWidths.find(curJoint.name) != boneWidths.end()) {
                    glm::vec2 widths = boneWidths[curJoint.name];
                    curJoint.boneWidthX = widths.x;
                    curJoint.boneWidthZ = widths.y;
                }
                else {
                    curJoint.boneWidthX = 0.05f;
                    curJoint.boneWidthZ = 0.05f;
                }
                data.poseTree.allNodes.push_back(curJoint);
                break;
            }
            case ParseState::Joint: {
                PoseTreeNode childJoint;
                childJoint.isEndSite = false;
                childJointID = data.poseTree.allNodes.size();
                PoseTreeNode& curJoint = data.poseTree.allNodes[curJointID];

                iss >> childJoint.name;
                newLine() >> keyword;
                if (keyword != "{") {
                    error = true;
                    break;
                }
                newLine() >> keyword >> childJoint.offset.x >> childJoint.offset.y
                          >> childJoint.offset.z; // OFFSET
                childJoint.offset *= scale;

                int channels;
                std::vector<std::string> channelNames;
                newLine() >> keyword >> channels;
                if (channels > 0 && channels <= 6) {
                    channelNames.resize(channels);
                }
                else {
                    fprintf(stderr, "Invalid number of channels. (%d)", channels);
                    error = true;
                    break;
                }
                for (int c = 0; c < channels; c++) {
                    iss >> channelNames[c];
                }
                for (int c = 0; c < channels; c++) {
                    if (auto channelType = stringToChannelType(channelNames[c])) {
                        channelTypeData.push_back(*channelType);
                    }
                    else {
                        fprintf(stderr, "Invalid channel type.");
                        error = true;
                        break;
                    }
                }
                if (error) break;
                data.numChannels += channels; data.poseTree.numJoints++; data.poseTree.numNodes++; // CHANNELS
                curJoint.childJoints.push_back(childJointID);
                childJoint.parent = curJointID;
                curJointID = childJointID;

                newLine() >> keyword;
                if (keyword == "JOINT") {
                    state = ParseState::Joint;
                }
                else if (keyword == "End") {
                    state = ParseState::EndSite;
                }

                if (boneWidths.find(childJoint.name) != boneWidths.end()) {
                    glm::vec2 widths = boneWidths[childJoint.name];
                    childJoint.boneWidthX = widths.x;
                    childJoint.boneWidthZ = widths.y;
                }
                else {
                    childJoint.boneWidthX = 0.05f;
                    childJoint.boneWidthZ = 0.05f;
                }
                data.poseTree.allNodes.push_back(childJoint);
                break;
            }
            case ParseState::EndSite: {
                PoseTreeNode childJoint;
                childJoint.isEndSite = true;
                curJointID = childJointID;
                uint32_t endSiteID = endSites.size();
                PoseTreeNode& curJoint = data.poseTree.allNodes[curJointID];

                childJoint.name = "End Site";
                newLine() >> keyword;
                if (keyword != "{") {
                    error = true;
                    break;
                }
                newLine() >> keyword >> childJoint.offset.x >> childJoint.offset.y >> childJoint.offset.z;
                childJoint.offset *= scale;
                data.poseTree.numNodes++;
                // The most significant bit tells if the id is an end site or not
                curJoint.childJoints.push_back(endSiteID + (1 << 31));
                childJoint.parent = curJointID;
                curJointID = childJointID;

                newLine() >> keyword;
                if (keyword != "}") {
                    error = true;
                    break;
                }
                newLine() >> keyword;
                while (keyword == "}") {
                    childJointID = curJointID;
                    curJointID = data.poseTree.allNodes[curJointID].parent;
                    newLine() >> keyword;
                }
                if (keyword == "JOINT") {
                    state = ParseState::Joint;
                }
                else if (keyword == "MOTION") {
                    state = ParseState::Motion;
                }

                childJoint.boneWidthX = 0.05f;
                childJoint.boneWidthZ = 0.05f;
                endSites.push_back(childJoint);
                break;
            }
            case ParseState::Motion: {
                newLine() >> keyword >> data.numFrames;
                std::string _;
                newLine() >> keyword >> _ >> data.frameTime;

                data.poseStates.resize(data.numFrames);
                float num;
                glm::quat rot = glm::identity<glm::quat>();
                int rotCount = 0;
                for (int f = 0; f < data.numFrames; f++) {
                    auto& poseState = data.poseStates[f];
                    newLine();
                    for (int c = 0; c < data.numChannels; c++) {
                        iss >> num;
                        if (!iss) {
                            error = true;
                            break;
                        }
                        if (c < 3) {
                            poseState.rootPos[c] = num * scale;
                        }
                        else {
                            switch (channelTypeData[c]) {
                                case ChannelType::Xrot:
                                    rot = glm::rotate(rot, glm::radians(num), {1, 0, 0});
                                    rotCount++;
                                    break;
                                case ChannelType::Yrot:
                                    rot = glm::rotate(rot, glm::radians(num), {0, 1, 0});
                                    rotCount++;
                                    break;
                                case ChannelType::Zrot:
                                    rot = glm::rotate(rot, glm::radians(num), {0, 0, 1});
                                    rotCount++;
                                    break;
                                default:
                                    // Ignore Xposition, Yposition, Zposition
                                    break;
                            }
                            if (rotCount == 3) {
                                poseState.jointRot.push_back(rot);
                                rot = glm::identity<glm::quat>();
                                rotCount = 0;
                            }
                        }
                    }
                }
                finished = true;
                break;
            }
        }

    }

    if (error) {
        return false;
    }

    // Now we combine the joint data and end site data into one std::vector.
    // Body node indices need to be in 0...numJoints-1,
    // and end site indices need to be in numJoints...numNodes-1.
    // So need to make sure the highest bit of the end site ids are set back to zero,
    // and then add numJoints to set the index in the right location.
    data.poseTree.allNodes.reserve(data.poseTree.allNodes.size() + endSites.size());
    data.poseTree.allNodes.insert(data.poseTree.allNodes.end(), endSites.begin(), endSites.end());
    for (uint32_t i = 0; i < data.poseTree.numJoints; i++) {
        auto& joint = data.poseTree.allNodes[i];
        for (auto& childJointID : joint.childJoints) {
            if (childJointID & (1 << 31)) {
                childJointID = (childJointID & ~(1 << 31)) + data.poseTree.numJoints;
            }
        }
    }

    return true;
}

void MotionClipData::print() const {
    printRecursive(0 /* root joint id */, 0);
}

void MotionClipData::printRecursive(uint32_t jointID, int depth) const {
    const PoseTreeNode& joint = poseTree.allNodes[jointID];
    if (!joint.isEndSite) {
        for (int i = 0; i < depth; i++) { std::cout << "    "; }
        std::cout << "Name: " << joint.name << std::endl;
        for (int i = 0; i < depth; i++) { std::cout << "    "; }
        std::cout << "Offset: " << joint.offset.x << " " << joint.offset.y << " " << joint.offset.z << std::endl;
        for (auto child : joint.childJoints) {
            printRecursive(child, depth + 1);
        }
    }
    else {
        for (int i = 0; i < depth; i++) { std::cout << "    "; }
        std::cout << "End Site Offset: " << joint.offset.x << " " << joint.offset.y << " " << joint.offset.z << std::endl;
    }
}
