#include "brain_data.h"
#include "utils/math.h"

BrainData::BrainData()
{
    std::fill(std::begin(penalty), std::end(penalty), SUBSTITUTE);
    std::fill(std::begin(oppoPenalty), std::end(oppoPenalty), SUBSTITUTE);
    ball = GameObject{};
    tmBall = GameObject{};
    filteredBall = GameObject{};
    predictedBall = GameObject{};
    robotPoseToField = Pose2D{};
    robotPoseToOdom = Pose2D{};
    odomToField = Pose2D{};
    interceptBall = GameObject{};
    headPitch = 0.0;
    headYaw = 0.0;
    robotBallAngleToField = 0.0;
    std::fill(std::begin(receiveId), std::end(receiveId), 0);
}

vector<GameObject> BrainData::getMarkingsByType(set<string> types) {
    if (types.size() == 0) return getMarkings();

    // else
    vector<GameObject> res = {};
    auto markings = getMarkings();
    for (int i = 0; i < markings.size(); i++) {
        if (types.count(markings[i].label) > 0) res.push_back(markings[i]);
    }
    
    return res;
}

vector<FieldMarker> BrainData::getMarkersForLocator()
{
    vector<FieldMarker> res;
    auto markings = getMarkings();
    for (size_t i = 0; i < markings.size(); i++)
    {
        auto label = markings[i].label;
        auto x = markings[i].posToRobot.x;
        auto y = markings[i].posToRobot.y;
        auto confidence = markings[i].confidence;

        char markerType = '\0';
        if (label == "LCross")
            markerType = 'L';
        else if (label == "TCross")
            markerType = 'T';
        else if (label == "XCross")
            markerType = 'X';
        else if (label == "PenaltyPoint")
            markerType = 'P';

        if (markerType == '\0') continue; // 未知类型不入定位器, 避免未初始化的 markerType 进入匹配
        res.push_back(FieldMarker{markerType, x, y, confidence});
    }
    return res;
}

Pose2D BrainData::robot2field(const Pose2D &poseToRobot)
{
    Pose2D poseToField;
    transCoord(
        poseToRobot.x, poseToRobot.y, poseToRobot.theta,
        robotPoseToField.x, robotPoseToField.y, robotPoseToField.theta,
        poseToField.x, poseToField.y, poseToField.theta);
    poseToField.theta = toPInPI(poseToField.theta);
    return poseToField;
}

Pose2D BrainData::field2robot(const Pose2D &poseToField)
{
    Pose2D poseToRobot;
    double xfr, yfr, thetafr; // fr = field to robot
    yfr = sin(robotPoseToField.theta) * robotPoseToField.x - cos(robotPoseToField.theta) * robotPoseToField.y;
    xfr = -cos(robotPoseToField.theta) * robotPoseToField.x - sin(robotPoseToField.theta) * robotPoseToField.y;
    thetafr = -robotPoseToField.theta;
    transCoord(
        poseToField.x, poseToField.y, poseToField.theta,
        xfr, yfr, thetafr,
        poseToRobot.x, poseToRobot.y, poseToRobot.theta);
    return poseToRobot;
}
