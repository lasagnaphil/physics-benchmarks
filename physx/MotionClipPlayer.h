//
// Created by lasagnaphil on 19. 5. 9.
//

#ifndef PHYSICS_BENCHMARKS_MOTIONCLIPPLAYER_H
#define PHYSICS_BENCHMARKS_MOTIONCLIPPLAYER_H

#include "MotionClipData.h"
#include "LineMesh.h"

struct GizmosRenderer;

struct MotionClipPlayer {
    MotionClipData* data;

    int currentFrameIdx = 0;
    bool isPlaying = false;
    bool shouldUpdate = false;

    Ref<LineMesh> trajectoryMesh;
    Ref<LineMaterial> trajectoryMat;

    MotionClipPlayer() : data(nullptr) {}
    MotionClipPlayer(MotionClipData* data) : data(data) {}

    void init();

    const PoseState& getPoseState() const {
        return data->getFrameState(currentFrameIdx);
    }

    PoseState& getPoseState() {
        return data->getFrameState(currentFrameIdx);
    }

    void play() {
        isPlaying = true;
    }

    void pause() {
        isPlaying = false;
    }

    void stop() {
        isPlaying = false;
        currentFrameIdx = 0;
    }

    void previousFrame() {
        currentFrameIdx--;
        if (currentFrameIdx == -1) {
            currentFrameIdx = data->numFrames - 1;
        }
        shouldUpdate = true;
    }

    void nextFrame() {
        currentFrameIdx++;
        if (currentFrameIdx >= data->numFrames) {
            currentFrameIdx = 0;
        }
        shouldUpdate = true;
    }

    void setFrame(uint32_t frameIdx) {
        assert(frameIdx >= 0 && frameIdx < data->numFrames);
        currentFrameIdx = frameIdx;
    }

    void update(float dt);

    void queueGizmosRender(GizmosRenderer& renderer, glm::mat4 modelMatrix);

    void drawImGui();
};

#endif //PHYSICS_BENCHMARKS_MOTIONCLIPPLAYER_H
