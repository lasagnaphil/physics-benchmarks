//
// Created by lasagnaphil on 19. 5. 9.
//

#include "MotionClipPlayer.h"

#include "GizmosRenderer.h"

void MotionClipPlayer::init() {
    std::vector<glm::vec3> rootPositions(data->numFrames);
    for (int i = 0; i < data->numFrames; i++) {
        rootPositions[i] = data->getRootPos(i);
    }

    trajectoryMesh = Resources::make<LineMesh>(rootPositions);
    trajectoryMesh->init();

    trajectoryMat = Resources::make<LineMaterial>();
    trajectoryMat->drawLines = true;
    trajectoryMat->drawPoints = true;
    trajectoryMat->lineColor = {1.0f, 0.0f, 0.0f, 1.0f};
    trajectoryMat->lineType = GL_LINE_STRIP;
    trajectoryMat->lineWidth = 2.0f;
    trajectoryMat->pointColor = {1.0f, 0.0f, 0.0f, 1.0f};
    trajectoryMat->pointSize = 4.0f;
}

void MotionClipPlayer::update(float dt) {
    static float frameUpdateLag = 0.0f;
    static float motionUpdateLag = 0.0f;
    if (isPlaying) {
        frameUpdateLag += dt;
        while (frameUpdateLag >= data->frameTime) {
            nextFrame();
            frameUpdateLag -= data->frameTime;
        }
    }
    else {
        frameUpdateLag = 0.0f;
    }
}

void MotionClipPlayer::queueGizmosRender(GizmosRenderer &renderer, glm::mat4 modelMatrix) {
    renderer.queueLine({trajectoryMesh, trajectoryMat, modelMatrix});
}

void MotionClipPlayer::drawImGui() {
    ImGui::Begin("Motion Clip Player");

    ImGui::Text("Current Frame: %d", currentFrameIdx);
    if (ImGui::ArrowButton("Left##PoseMesh", ImGuiDir_Left)) {
        previousFrame();
    }
    ImGui::SameLine();
    if (ImGui::ArrowButton("Right##PoseMesh", ImGuiDir_Right)) {
        nextFrame();
    }
    ImGui::SameLine();
    if (ImGui::Button("Play##PoseMesh")) {
        play();
    }
    ImGui::SameLine();
    if (ImGui::Button("Pause##PoseMesh")) {
        pause();
    }
    ImGui::SameLine();
    if (ImGui::Button("Stop##PoseMesh")) {
        stop();
    }

    if (ImGui::SliderInt("Frame##PoseMesh frame slider", &currentFrameIdx, 0, data->numFrames - 1)) {
        shouldUpdate = true;
    }

    ImGui::End();
}
